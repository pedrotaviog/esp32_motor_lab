#include <stdio.h>
#include <string.h>
#include "driver/ledc.h"
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_adc/adc_oneshot.h"

#define PWM_GPIO        2                   // pino de saída PWM
#define PWM_FREQ_HZ     1000                // 1 kHz (ajuste conforme seu driver de potência)
#define PWM_RES_BITS    10                  // 10 bits -> duty 0..1023
#define PWM_TIMER       LEDC_TIMER_0
#define PWM_MODE        LEDC_LOW_SPEED_MODE
#define PWM_CHANNEL     LEDC_CHANNEL_0

// ===== Buffer circular: coleta contínua a cada 10 ms =====
#define BUF_N       2000    // ~20 s de histórico (2000 * 10 ms)
#define BATCH_SIZE  10      // envia 10 amostras por /data (100 ms)

typedef struct {
    float v;    // tensão medida (V)
    float d;    // duty (%) no instante da amostra
} sample_t;

static sample_t rb[BUF_N];
static volatile uint32_t rb_head = 0;      // próximo índice de escrita
static SemaphoreHandle_t rb_mutex = NULL;  // protege acesso ao buffer

static uint32_t duty_raw = 0; // duty atual (0..1023 p/ resolução de 10 bits)
static const char *TAG = "APP";

// HTML EMBUTIDO - Avisando ao linker para incluir o HTML no binário final
extern const uint8_t index_html_start[] asm("_binary_index_html_start"); // ponteiro para o início do HTML
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");   // ponteiro para o fim do HTML

// ==== ADC ====
static adc_oneshot_unit_handle_t adc1_handle; // handle do driver oneshot (API nova ESP-IDF)

void adc_init(void) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    // GPIO34 = ADC1_CHANNEL_3
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &config));
}

// ==== PWM ====
void pwm_init(void) {
    ledc_timer_config_t tcfg = {
        .speed_mode       = PWM_MODE,
        .duty_resolution  = PWM_RES_BITS,
        .timer_num        = PWM_TIMER,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    ledc_channel_config_t ccfg = {
        .gpio_num       = PWM_GPIO,
        .speed_mode     = PWM_MODE,
        .channel        = PWM_CHANNEL,
        .timer_sel      = PWM_TIMER,
        .duty           = 0,
        .hpoint         = 0,
        .intr_type      = LEDC_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ccfg));
}

static inline void pwm_set_duty_percent(float duty_percent) {
    if (duty_percent < 0) duty_percent = 0;
    if (duty_percent > 100) duty_percent = 100;
    uint32_t max_duty = (1U << PWM_RES_BITS) - 1U;
    duty_raw = (uint32_t)((duty_percent * max_duty / 100.0f) + 0.5f);
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty_raw));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL));
}

static inline float pwm_get_duty_percent(void) {
    uint32_t max_duty = (1U << PWM_RES_BITS) - 1U;
    return 100.0f * ((float)duty_raw / (float)max_duty);
}

// ==== HANDLERS HTTP ====

// envia index.html
esp_err_t root_get_handler(httpd_req_t *req) {
    const size_t index_html_size = (index_html_end - index_html_start);
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)index_html_start, index_html_size);
}

// envia as últimas BATCH_SIZE amostras (snapshot, sem bloquear)
esp_err_t data_get_handler(httpd_req_t *req) {
    float v_local[BATCH_SIZE];
    float d_local[BATCH_SIZE];

    // snapshot protegido por mutex
    if (xSemaphoreTake(rb_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{ \"samples\": [] }");
        return ESP_OK;
    }

    uint32_t h = rb_head;
    uint32_t start = (h + BUF_N - BATCH_SIZE) % BUF_N;
    for (int i = 0; i < BATCH_SIZE; i++) {
        uint32_t idx = (start + i) % BUF_N;
        v_local[i] = rb[idx].v;
        d_local[i] = rb[idx].d;
    }
    xSemaphoreGive(rb_mutex);

    // monta JSON (chunked)
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr_chunk(req, "{ \"samples\": [");

    for (int i = 0; i < BATCH_SIZE; i++) {
        char buf[64];
        snprintf(buf, sizeof(buf),
                 "{\"v\":%.3f,\"d\":%.1f}%s",
                 v_local[i], d_local[i],
                 (i < BATCH_SIZE - 1) ? "," : "");
        httpd_resp_sendstr_chunk(req, buf);
    }

    httpd_resp_sendstr_chunk(req, "] }");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}


// recebe /set?duty=XX.X para atualizar o PWM
esp_err_t set_get_handler(httpd_req_t *req) {
    char buf[64];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char param[16];
        if (httpd_query_key_value(buf, "duty", param, sizeof(param)) == ESP_OK) {
            float d = atof(param);
            pwm_set_duty_percent(d);
            ESP_LOGI(TAG, "Duty atualizado via /set: %.1f%%", d);
        }
    }
    char resp[64];
    snprintf(resp, sizeof(resp), "{ \"duty\": %.1f }", pwm_get_duty_percent());
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

// ==== SERVIDOR HTTP ====
httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_root = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &uri_root);

        httpd_uri_t uri_data = {
            .uri       = "/data",
            .method    = HTTP_GET,
            .handler   = data_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &uri_data);

        httpd_uri_t uri_set = {
            .uri       = "/set",
            .method    = HTTP_GET,
            .handler   = set_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &uri_set);
    }
    return server;
}

// ==== WIFI AP ====
void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_config = {
        .ap = {
            .ssid = "ESP32_AP",
            .ssid_len = strlen("ESP32_AP"),
            .channel = 1,
            .password = "12345678",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen("12345678") == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP iniciado. SSID:%s senha:%s", "ESP32_AP", "12345678");
}

// ==== TASKS ====

// Task de controle (ADC + log)
void task_control(void *pvParameters) {
    uint32_t i = 0;
    while (1) {
        int adc_raw = 0;
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &adc_raw);
        float voltage = adc_raw * 3.3f / 4095.0f;
        float duty = pwm_get_duty_percent();

        // grava no ring buffer
        if (xSemaphoreTake(rb_mutex, portMAX_DELAY) == pdTRUE) {
            rb[rb_head].v = voltage;
            rb[rb_head].d = duty;
            rb_head = (rb_head + 1) % BUF_N;
            xSemaphoreGive(rb_mutex);
        }

        // (opcional) log esparso para não travar I/O
        if ((i++ % 100) == 0) { // ~1 s
            ESP_LOGI("CTRL", "V=%.3f V | Duty=%.1f%% | head=%lu",
                     voltage, duty, (unsigned long)rb_head);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 10 ms entre amostras
    }
}


// Task do servidor (HTTP)
void task_server(void *pvParameters) {
    start_webserver();
    vTaskDelete(NULL);
}

// ==== MAIN ====
void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_softap();
    adc_init();
    pwm_init();
    pwm_set_duty_percent(40.0f);   // malha aberta inicial em 40%
    rb_mutex = xSemaphoreCreateMutex();
    configASSERT(rb_mutex != NULL);

    // Cria tasks fixadas em núcleos diferentes
    xTaskCreatePinnedToCore(task_control, "task_control", 4096, NULL, 5, NULL, 1); // Core 1
    xTaskCreatePinnedToCore(task_server,  "task_server",  8192, NULL, 6, NULL, 0); // Core 0
}
