# ESP32 Motor Lab

Projeto baseado no microcontrolador **ESP32**, desenvolvido para **acionamento e monitoramento de motor DC**.  
O sistema utiliza **PWM** para controle de potência e **ADC** para leitura da tensão do tacogerador, com visualização em tempo real por meio de uma **interface web**.

---

## 🚀 Funcionalidades

- Geração de sinal PWM via periférico LEDC  
- Leitura de tensão do tacogerador (ADC)  
- Servidor HTTP interno hospedando página web  
- Interface web para visualização e controle de duty cycle  
- Exportação de dados em CSV  

---

## ⚙️ Requisitos

- **ESP-IDF v5.x** configurado  
- **ESP32 DevKit V1**  
- **Circuitos externos adequados**:
  - **Circuito de acionamento** (ex.: optoacoplador para isolamento lógico);
  - **Circuito de potência** (ex.: MOSFETs para chaveamento da carga);
  - **Circuito de adequação de entrada** (ex.: divisor resistivo para leitura de tensão no ADC);
- **Fonte de alimentação estável** (de acordo com as especificações do motor e do circuito de potência);
- Navegador para acesso à interface web.

> ⚠️ **Aviso Importante:**  
> Este código foi desenvolvido com fins **didáticos e de demonstração**.  
> O sistema depende de circuitos externos de **acionamento, potência e adequação de sinal**, que podem variar conforme o projeto e os componentes utilizados.  
> 
> A ligação direta de motores, fontes ou sensores ao ESP32 **pode causar danos permanentes** se não houver o devido isolamento e dimensionamento elétrico.  
> 
> O autor **não se responsabiliza por danos ou mau funcionamento** decorrentes de implementações incorretas, modificações no código ou uso inadequado do hardware.  
> Use o projeto **como base de estudo** e **adapte os circuitos de forma segura** às suas necessidades específicas.

---

## 🔧 Como usar

1. Clone o repositório:
   ```bash
   git clone https://github.com/pedrotaviog/esp32_motor_lab.git
   cd esp32_motor_lab

2. Compile e grave no ESP32:

  idf.py set-target esp32
  idf.py build
  idf.py flash monitor

3. Conecte-se à rede Wi-Fi gerada pelo ESP32:

  SSID: ESP32_AP
  Senha: 12345678

4. Acesse a interface no navegador:

  http://192.168.4.1/