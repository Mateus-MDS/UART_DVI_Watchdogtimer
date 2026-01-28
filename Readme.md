# Telemetria UART + Watchdog com IHM (DVI/Serial) — Raspberry Pi Pico (RP2040)

Este repositório contém a implementação de um sistema embarcado distribuído em **duas Raspberry Pi Pico (RP2040)**:

- **Pico A (Transmissor):** controle IR + watchdog + telemetria binária via UART + diagnóstico em OLED.
- **Pico B (Receptor/IHM):** recepção e validação de telemetria via UART + exibição do estado (IHM) + proteção contra boot-loop e perda de comunicação.

> O repositório também inclui um **código base DVI** (fonte original) como referência de geração de vídeo em 640×480p.

---

## Visão Geral do Projeto (Parte 12)

### Objetivo
Demonstrar uma solução de IHM/telemetria utilizando:
- **Comunicação UART** entre duas placas,
- **Watchdog Timer (WDT)** para resiliência e recuperação automática,
- **Diagnóstico e supervisão** de falhas induzidas (travamentos controlados),
- **Exibição do estado do sistema** no receptor (IHM via terminal/serial) e no transmissor (OLED).

---

## Arquitetura do Sistema

### Pico A — Transmissor (`Transmissor.c`)
Responsabilidades principais:
- Receber comandos via **Serial (stdio)** para controlar estados do ar-condicionado via **IR**.
- Enviar **telemetria binária via UART0** (TX em **GP0**) a cada **500 ms**.
- Proteger o sistema com **Watchdog (timeout 5000 ms)**.
- Registrar informações persistentes em **Flash** (contadores e última falha).
- Exibir diagnóstico e estado no **OLED SSD1306** (I2C1).

### Pico B — Receptor/IHM (`hdmi.c`)
Responsabilidades principais:
- Receber pacotes via **UART0** (RX em **GP1**).
- Validar pacote por **Header/Footer + Checksum**.
- Exibir a telemetria recebida em formato de **IHM via terminal/serial** (espelho serial).
- Monitorar perda de comunicação (**timeout 2000 ms**).
- Proteger o receptor com **Watchdog (timeout 8000 ms)** e evitar **boot-loop**, aplicando carência e reboot controlado quando necessário.

---

## Comunicação UART (Protocolo de Telemetria)

A telemetria é enviada em formato binário fixo com 22 bytes:

- `header` = `0xAA`
- Campos: `ac_state`, `last_command`, `ir_pending`, `uptime_ms`, `wdt_resets`, `last_fault`, `ir_operations`
- `checksum` (soma dos bytes, exceto checksum e footer)
- `footer` = `0x55`

Isso garante sincronização, robustez contra ruído e validação simples no receptor.

---

## Simulação de Falhas (Validação do Watchdog)

O Transmissor implementa falhas propositalmente para validar a resiliência:

- **Falha 1 (F):** Loop infinito sem feed do WDT (`FALHA_LOOP_INFINITO = 0x01`)
- **Falha 2 (3):** Travamento ao processar “22°C” (`FALHA_TEMP_22C = 0x02`)
- **Falha 3 (U):** UART travada (loop transmitindo sem feed do WDT) (`FALHA_UART_TRAVADA = 0x03`)

O sistema registra a última falha e contadores de reset (Flash + scratch registers), permitindo diagnóstico pós-reboot.

---

## Conexões e Pinos

### UART0 (Telemetria)
- **Pico A TX → Pico B RX**
- Pico A: **GP0 (TX)**
- Pico B: **GP1 (RX)**
- Baud rate: **115200**

### IR (Pico A)
- IR out: **GPIO 18**
- LED onboard: **GPIO 25**

### OLED SSD1306 (Pico A)
- I2C1: **SDA = GP14**, **SCL = GP15**
- Endereço: **0x3C**

### LEDs BitDogLab (Pico A)
- LED_BOOT_RED: **GPIO 13**
- LED_OK_GREEN: **GPIO 11**
- LED_TRAVA_BLUE: **GPIO 12**

---

## Como Executar

### 1) Compilar e gravar Pico A (Transmissor)
- Arquivo: `Transmissor.c`
- Após gravar, abra o **Serial Monitor** para acessar o menu e enviar comandos.

### 2) Compilar e gravar Pico B (Receptor/IHM)
- Arquivo: `hdmi.c`
- Abra o Serial Monitor do Pico B para visualizar o “espelho” da telemetria.

---

# Base DVI (Referência) — IHM Digital via DVI com Raspberry Pi Pico

Este projeto base demonstra a implementação de uma **Interface Homem-Máquina (IHM)** utilizando o microcontrolador **RP2040** (Raspberry Pi Pico W). O sistema realiza a leitura de sinais analógicos e os projeta em tempo real em um monitor através de uma saída digital DVI gerada inteiramente via software.

## Visão Geral Técnica
O projeto utiliza uma arquitetura de processamento paralelo e máquinas de estado para superar a ausência de um controlador de vídeo dedicado no hardware original. Está focado na geração de vídeo em tempo real com conector de saída tipo HDMI.

### Arquitetura Dual-Core
Para garantir a estabilidade do sinal de vídeo a 60Hz, as tarefas são divididas entre os núcleos do processador:
* **Core 0 (Lógica e Aquisição):** Inicializa o hardware, realiza a leitura do canal 1 do ADC (GPIO 27) e gerencia os buffers de caracteres (`charbuf`) e cores (`colourbuf`).
* **Core 1 (Renderização em Tempo Real):** Dedicado exclusivamente à geração do sinal DVI, realizando a codificação TMDS e o envio dos dados para o monitor.

### Especificações de Vídeo
* **Resolução:** 640x480p a 60Hz.
* **Escalonamento Vertical (3x):** Implementação de uma lógica que triplica a altura da fonte original (8x8 para 8x24 pixels), garantindo legibilidade superior em telas LCD.
* **Codificação TMDS:** Uso de rotinas em **Assembly** (`tmds_encode_font_2bpp.S`) para converter dados RGB222 para o protocolo digital de forma ultra-rápida.

### Funcionalidades do Projeto
* **Leitura ADC:** Monitoramento contínuo de entrada analógica (0 a 4095) com exibição dinâmica na tela.
* **Interface Visual:** Moldura personalizada (`draw_border`) com texto centralizado e suporte a cores distintas para valores e prompts.

### Principais Características
* **Protocolo DVI via Software:** Utiliza a biblioteca `libdvi` para implementar o sinal **TMDS** através das máquinas de estado **PIO**, permitindo saída de vídeo digital sem hardware dedicado.
* **Otimização de Memória:** Configurado para gerenciar o stack do **Core 1**, garantindo que o processamento do vídeo ocorra de forma paralela no segundo núcleo.
* **Processamento de Fontes:** Inclui rotinas em Assembly (`tmds_encode_font_2bpp.S`) para codificação rápida, essencial para exibir dados com baixa latência.

---

## Vídeo Demonstrativo

**Clique [AQUI](https://www.youtube.com/watch?v=mi5Pt5lvZtY) para acessar o link do Vídeo Ensaio**

