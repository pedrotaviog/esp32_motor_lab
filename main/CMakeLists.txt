idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_wifi esp_event esp_netif nvs_flash esp_http_server esp_adc
    EMBED_FILES "index.html"
)
