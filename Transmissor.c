/**
 * Controle de ar condicionado via IR com Watchdog e Telemetria UART
 * Transmissor para sistema DVI - Pico A
 * 
 * Funcionalidades:
 * - Controle IR de ar condicionado VIA SERIAL
 * - Proteção via Watchdog Timer
 * - Telemetria via UART (GP0/TX) para Pico B (Receptor DVI)
 * - Display OLED com status do sistema
 * - Simulação de falhas via comandos serial
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "lib/custom_ir.h"
#include "lib/ssd1306.h"

#include "hardware/flash.h"
#include "hardware/sync.h"

// ===================== PINOS BITDOGLAB =====================
#define LED_BOOT_RED     13   // LED vermelho: indica boot/reset
#define LED_OK_GREEN     11   // LED verde: operação normal
#define LED_TRAVA_BLUE   12   // LED azul: falha/travamento

// ===================== PINOS IR =====================
#define IR_PIN           18   // Pino para saída IR
#define LED_PIN          25   // LED onboard do Pico

// ===================== PINOS UART =====================
#define UART_ID          uart0
#define UART_TX_PIN       0   // GP0 - TX para Pico B
#define UART_RX_PIN       1   // GP1 - RX (não usado, mas configurado)
#define UART_BAUD_RATE   115200

// ===================== DISPLAY =====================
#define I2C_PORT_DISP    i2c1
#define SDA_DISP         14
#define SCL_DISP         15
#define DISPLAY_ADDR     0x3C

// ===================== WATCHDOG =====================
#define WDT_TIMEOUT_MS   5000  // 5 segundos de margem segura

// Códigos de falha nos scratch registers
#define FALHA_LOOP_INFINITO    0x01  // Falha induzida por comando serial
#define FALHA_TEMP_22C         0x02  // Falha no comando de temperatura 22°C
#define FALHA_UART_TRAVADA     0x03  // Falha simulada de UART travada

// ===================== TELEMETRIA =====================
#define TELEMETRY_INTERVAL_MS  500  // Transmite a cada 500ms
#define TELEM_HEADER     0xAA
#define TELEM_FOOTER     0x55

// ===================== Dados Para Salvar na Flash =====================
#define FLASH_MAGIC 0xDEADBEEF
#define FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - 4096) // último setor

typedef struct {
    uint32_t magic;
    uint32_t boot_count;     // total de boots (power-on)
    uint32_t wdt_count;      // resets por watchdog
    uint32_t last_reset;     // 0x000 normal | 0x001 watchdog
    uint32_t last_fault;     // código da última falha
} persist_data_t;

static persist_data_t persist;

static void load_persist_data(void) {
    const persist_data_t *flash_data =
        (const persist_data_t *)(XIP_BASE + FLASH_OFFSET);

    if (flash_data->magic == FLASH_MAGIC) {
        persist = *flash_data;
    } else {
        // Primeira inicialização
        persist.magic = FLASH_MAGIC;
        persist.boot_count = 0;
        persist.wdt_count = 0;
        persist.last_reset = 0;
        persist.last_fault = 0;
    }
}

static void save_persist_data(void) {
    uint32_t ints = save_and_disable_interrupts();

    flash_range_erase(FLASH_OFFSET, 4096);
    flash_range_program(FLASH_OFFSET,
                        (uint8_t *)&persist,
                        sizeof(persist_data_t));

    restore_interrupts(ints);
}


// ===================== ESTADOS DO SISTEMA =====================
typedef enum {
    STATE_OFF,
    STATE_ON,
    STATE_TEMP_20,
    STATE_TEMP_22,
    STATE_FAN_1,
    STATE_FAN_2,
    STATE_MAX
} system_state_t;

// ===================== ESTRUTURA DE TELEMETRIA CORRIGIDA =====================
typedef struct __attribute__((packed)) {
    uint8_t header;           // 1 byte  → offset 0
    uint8_t ac_state;         // 1 byte  → offset 1
    uint8_t last_command;     // 1 byte  → offset 2
    uint8_t ir_pending;       // 1 byte  → offset 3
    uint32_t uptime_ms;       // 4 bytes → offset 4
    uint32_t wdt_resets;      // 4 bytes → offset 8
    uint32_t last_fault;      // 4 bytes → offset 12
    uint32_t ir_operations;   // 4 bytes → offset 16
    uint8_t checksum;         // 1 byte  → offset 20
    uint8_t footer;           // 1 byte  → offset 21
} telemetry_data_t;         // Total: 22 bytes

// ===================== VARIÁVEIS GLOBAIS =====================
static ssd1306_t ssd;
static uint32_t last_operation_time = 0;
static bool ir_operation_pending = false;
static system_state_t current_state = STATE_OFF;
static system_state_t last_display_state = STATE_MAX;
static system_state_t last_command_sent = STATE_OFF;
static uint32_t ir_operation_counter = 0;

// ===================== HELPERS GPIO =====================
static void init_gpio(void) {
    // LEDs de diagnóstico
    gpio_init(LED_BOOT_RED);
    gpio_init(LED_OK_GREEN);
    gpio_init(LED_TRAVA_BLUE);
    gpio_set_dir(LED_BOOT_RED, GPIO_OUT);
    gpio_set_dir(LED_OK_GREEN, GPIO_OUT);
    gpio_set_dir(LED_TRAVA_BLUE, GPIO_OUT);

    // LED onboard
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
}

// ===================== HELPERS UART =====================
static void init_uart_telemetry(void) {
    uart_init(UART_ID, UART_BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // Configura formato: 8 bits, sem paridade, 1 stop bit
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    
    printf("UART telemetria inicializada: %d baud, GP%d(TX)\n", 
           UART_BAUD_RATE, UART_TX_PIN);
}

// ===================== HELPERS DISPLAY =====================
static void init_display(ssd1306_t *ssd) {
    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_DISP);
    gpio_pull_up(SCL_DISP);

    ssd1306_init(ssd, WIDTH, HEIGHT, false, DISPLAY_ADDR, I2C_PORT_DISP);
    ssd1306_config(ssd);
}

static void draw_frame_base(ssd1306_t *ssd, bool cor) {
    ssd1306_fill(ssd, !cor);
    ssd1306_rect(ssd, 3, 3, 122, 60, cor, !cor);
    ssd1306_line(ssd, 3, 25, 123, 25, cor);
    ssd1306_line(ssd, 3, 37, 123, 37, cor);
}

// Tela de diagnóstico de boot
static void show_boot_diag(ssd1306_t *ssd, bool reboot_wdt, uint32_t count, uint32_t fault) {
    char line[22];
    draw_frame_base(ssd, true);

    ssd1306_draw_string(ssd, "IR+WDT+UART", 20, 6);
    ssd1306_draw_string(ssd, reboot_wdt ? "RST: WATCHDOG" : "RST: NORMAL", 10, 16);

    snprintf(line, sizeof(line), "CNT: %lu", (unsigned long)count);
    ssd1306_draw_string(ssd, line, 10, 28);
    
    snprintf(line, sizeof(line), "FLT: 0x%02lX", (unsigned long)fault);
    ssd1306_draw_string(ssd, line, 10, 40);
    
    snprintf(line, sizeof(line), "WDT: %dms", WDT_TIMEOUT_MS);
    ssd1306_draw_string(ssd, line, 10, 52);

    ssd1306_send_data(ssd);
}

// Tela de operação mostrando estado do AC
static void show_running_state(ssd1306_t *ssd, system_state_t state) {
    char line[22];
    draw_frame_base(ssd, true);

    ssd1306_draw_string(ssd, "AC+WDT+UART", 20, 6);
    
    // Mostra estado atual do AC
    switch (state) {
        case STATE_OFF:
            ssd1306_draw_string(ssd, "AC: OFF", 10, 16);
            break;
        case STATE_ON:
            ssd1306_draw_string(ssd, "AC: ON", 10, 16);
            break;
        case STATE_TEMP_20:
            ssd1306_draw_string(ssd, "AC: 20C", 10, 16);
            break;
        case STATE_TEMP_22:
            ssd1306_draw_string(ssd, "AC: 22C", 10, 16);
            break;
        case STATE_FAN_1:
            ssd1306_draw_string(ssd, "AC: FAN 1", 10, 16);
            break;
        case STATE_FAN_2:
            ssd1306_draw_string(ssd, "AC: FAN 2", 10, 16);
            break;
        default:
            ssd1306_draw_string(ssd, "AC: ???", 10, 16);
            break;
    }

    // Mostra estatísticas
    snprintf(line, sizeof(line), "OPS: %lu", (unsigned long)ir_operation_counter);
    ssd1306_draw_string(ssd, line, 10, 28);
    
    snprintf(line, sizeof(line), "RST: %lu", (unsigned long)watchdog_hw->scratch[0]);
    ssd1306_draw_string(ssd, line, 10, 40);
    
    ssd1306_draw_string(ssd, "TX: ATIVO", 10, 52);

    ssd1306_send_data(ssd);
}

// Tela de falha
static void show_fault_mode(ssd1306_t *ssd, const char* msg, const char* detail) {
    draw_frame_base(ssd, true);

    ssd1306_draw_string(ssd, "FALHA INDUZIDA", 12, 6);
    ssd1306_draw_string(ssd, msg, 10, 16);
    ssd1306_draw_string(ssd, detail, 10, 28);
    ssd1306_draw_string(ssd, "Aguard. reset", 10, 40);
    ssd1306_draw_string(ssd, "WDT ~5 seg...", 10, 52);

    ssd1306_send_data(ssd);
}

// ===================== TELEMETRIA =====================
static uint8_t calculate_checksum(telemetry_data_t *data) {
    uint8_t sum = 0;
    uint8_t *ptr = (uint8_t*)data;
    
    // Soma todos os bytes exceto o checksum e footer
    for (size_t i = 0; i < sizeof(telemetry_data_t) - 2; i++) {
        sum += ptr[i];
    }
    
    return sum;
}

static void send_telemetry(void) {
    telemetry_data_t telem = {0};
    
    // Preenche header e footer
    telem.header = TELEM_HEADER;
    telem.footer = TELEM_FOOTER;
    
    // Preenche dados do sistema
    telem.ac_state = current_state;
    telem.last_command = last_command_sent;
    telem.ir_pending = ir_operation_pending ? 1 : 0;
    telem.uptime_ms = to_ms_since_boot(get_absolute_time());
    telem.wdt_resets = persist.wdt_count;
    telem.last_fault = persist.last_fault;
    telem.ir_operations = ir_operation_counter;
    
    // Calcula checksum
    telem.checksum = calculate_checksum(&telem);
    
    // Transmite via UART em formato binário
    uart_write_blocking(UART_ID, (uint8_t*)&telem, sizeof(telemetry_data_t));
}

// ===================== CONTROLE IR COM PROTEÇÃO =====================
static bool execute_ir_command_safe(system_state_t new_state) {
    ir_operation_pending = true;
    last_operation_time = to_ms_since_boot(get_absolute_time());
    
    printf("Executando comando IR para estado: %d\n", new_state);
    
    // Atualiza último comando
    last_command_sent = new_state;
    
    // Feed do watchdog ANTES da operação IR
    watchdog_update();
    
    // ===== DEFEITO 2: TEMPERATURA 22°C =====
    if (new_state == STATE_TEMP_22) {
        printf("\n!!! FALHA NO COMANDO 22C !!!\n");
        printf("Sistema travara ao processar temperatura 22C\n");
        
        watchdog_update();   // garante margem
        persist.last_fault = FALHA_TEMP_22C;
        save_persist_data();   // Salva estado antes de travar
        watchdog_hw->scratch[1] = FALHA_TEMP_22C;
        show_fault_mode(&ssd, "CMD 22C", "Travamento IR");
        
        // Transmite telemetria com status de falha antes de travar
        send_telemetry();
        sleep_ms(50); // Garante envio
        
        // Loop infinito SEM watchdog_update()
        while (true) {
            gpio_put(LED_TRAVA_BLUE, 1);
            gpio_put(LED_PIN, 1);
            sleep_ms(150);
            gpio_put(LED_TRAVA_BLUE, 0);
            gpio_put(LED_PIN, 0);
            sleep_ms(150);
        }
    }
    
    // Executa comando IR apropriado para os demais estados
    switch (new_state) {
        case STATE_OFF:
            printf("Comando: DESLIGAR AC\n");
            turn_off_ac();
            gpio_put(LED_PIN, 0);
            break;
            
        case STATE_ON:
            printf("Comando: LIGAR AC\n");
            turn_on_ac();
            gpio_put(LED_PIN, 1);
            break;
            
        case STATE_TEMP_20:
            printf("Comando: TEMPERATURA 20C\n");
            set_temp_20c();
            gpio_put(LED_PIN, 1);
            break;
            
        case STATE_FAN_1:
            printf("Comando: VENTILADOR NIVEL 1\n");
            set_fan_level_1();
            gpio_put(LED_PIN, 1);
            break;
            
        case STATE_FAN_2:
            printf("Comando: VENTILADOR NIVEL 2\n");
            set_fan_level_2();
            gpio_put(LED_PIN, 1);
            break;
            
        default:
            printf("Estado invalido\n");
            ir_operation_pending = false;
            return false;
    }
    
    // Feed do watchdog APÓS a operação IR
    watchdog_update();
    
    // Delay para garantir transmissão completa
    sleep_ms(100);
    
    ir_operation_pending = false;
    current_state = new_state;
    ir_operation_counter++;
    
    printf("Comando IR executado (Total: %lu ops)\n", 
           (unsigned long)ir_operation_counter);
    
    // Transmite telemetria após comando bem-sucedido
    send_telemetry();
    
    return true;
}

// ===================== FUNÇÕES DE FALHA SIMULADA =====================
static void trigger_infinite_loop_fault(void) {
    printf("\n!!! FALHA 1: LOOP INFINITO !!!\n");
    printf("Sistema entrara em loop infinito sem feed do WDT\n");
    
    watchdog_update();   // garante margem
    persist.last_fault = FALHA_LOOP_INFINITO;
    save_persist_data();   // Salva estado antes de travar
    watchdog_hw->scratch[1] = FALHA_LOOP_INFINITO;
    show_fault_mode(&ssd, "LOOP INFINITO", "Cmd 'F'");
    
    // Transmite telemetria com status de falha
    send_telemetry();
    sleep_ms(50);
    
    // Loop infinito SEM watchdog_update()
    while (true) {
        gpio_put(LED_TRAVA_BLUE, 1);
        sleep_ms(200);
        gpio_put(LED_TRAVA_BLUE, 0);
        sleep_ms(200);
    }
}

static void trigger_uart_stuck_fault(void) {
    printf("\n!!! FALHA 3: UART TRAVADA !!!\n");
    printf("Sistema travara tentando transmitir infinitamente\n");
    
    watchdog_update();   // garante margem
    persist.last_fault = FALHA_UART_TRAVADA;
    save_persist_data();   // Salva estado antes de travar
    watchdog_hw->scratch[1] = FALHA_UART_TRAVADA;
    show_fault_mode(&ssd, "UART TRAVADA", "Cmd 'U'");
    
    // Transmite telemetria com status de falha
    send_telemetry();
    sleep_ms(50);
    
    // Loop infinito transmitindo dados inválidos SEM watchdog_update()
    while (true) {
        uart_puts(UART_ID, "XXXXXXXXXXXXXXXXXX");
        gpio_put(LED_TRAVA_BLUE, 1);
        sleep_ms(100);
        gpio_put(LED_TRAVA_BLUE, 0);
        sleep_ms(100);
    }
}

// ===================== PROCESSAMENTO DE COMANDOS SERIAL =====================
static void print_menu(void) {
    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║  MENU IR + WATCHDOG + TELEMETRIA       ║\n");
    printf("╠════════════════════════════════════════╣\n");
    printf("║  COMANDOS AC:                          ║\n");
    printf("║  [1] Ligar AC                          ║\n");
    printf("║  [2] Desligar AC                       ║\n");
    printf("║  [3] Temperatura 22C (FALHA!)          ║\n");
    printf("║  [4] Temperatura 20C                   ║\n");
    printf("║  [5] Ventilador Nivel 1                ║\n");
    printf("║  [6] Ventilador Nivel 2                ║\n");
    printf("║                                        ║\n");
    printf("║  SIMULAÇÃO DE FALHAS:                  ║\n");
    printf("║  [F] Loop Infinito (Falha 1)           ║\n");
    printf("║  [U] UART Travada (Falha 3)            ║\n");
    printf("║                                        ║\n");
    printf("║  UTILITÁRIOS:                          ║\n");
    printf("║  [S] Status do Sistema                 ║\n");
    printf("║  [0] Mostrar este Menu                 ║\n");
    printf("╚════════════════════════════════════════╝\n");
    printf("\nTelemetria: GP%d @ %d baud (a cada %dms)\n", 
           UART_TX_PIN, UART_BAUD_RATE, TELEMETRY_INTERVAL_MS);
    printf("Digite um comando: ");
}

static void print_status(void) {
    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║  STATUS DO SISTEMA                     ║\n");
    printf("╠════════════════════════════════════════╣\n");
    
    // Estado do AC
    printf("║  Estado AC: ");
    switch (current_state) {
        case STATE_OFF:     printf("DESLIGADO              ║\n"); break;
        case STATE_ON:      printf("LIGADO                 ║\n"); break;
        case STATE_TEMP_20: printf("20°C                   ║\n"); break;
        case STATE_TEMP_22: printf("22°C                   ║\n"); break;
        case STATE_FAN_1:   printf("VENTILADOR NIVEL 1     ║\n"); break;
        case STATE_FAN_2:   printf("VENTILADOR NIVEL 2     ║\n"); break;
        default:            printf("DESCONHECIDO           ║\n"); break;
    }
    
    printf("║  Operacoes IR: %-20lu║\n", (unsigned long)ir_operation_counter);
    printf("║  Uptime: %-27lu║\n", (unsigned long)(to_ms_since_boot(get_absolute_time()) / 1000));
    printf("║  Resets WDT: %-22lu║\n", (unsigned long)watchdog_hw->scratch[0]);
    
    uint32_t fault = watchdog_hw->scratch[1];
    printf("║  Ultima Falha: ");
    if (fault == 0) {
        printf("Nenhuma              ║\n");
    } else if (fault == FALHA_LOOP_INFINITO) {
        printf("Loop Infinito (0x01) ║\n");
    } else if (fault == FALHA_TEMP_22C) {
        printf("Cmd 22C (0x02)       ║\n");
    } else if (fault == FALHA_UART_TRAVADA) {
        printf("UART Travada (0x03)  ║\n");
    } else {
        printf("0x%02lX                  ║\n", (unsigned long)fault);
    }
    
    printf("║  Telemetria: ATIVA                     ║\n");
    printf("║  Watchdog: ATIVO (%dms)             ║\n", WDT_TIMEOUT_MS);
    printf("╚════════════════════════════════════════╝\n");
    printf("\n");
}

static void process_uart_input() {
    int ch = getchar_timeout_us(0);
    if (ch == PICO_ERROR_TIMEOUT) {
        return;
    }
    
    // Converte para maiúscula
    if (ch >= 'a' && ch <= 'z') {
        ch = ch - 'a' + 'A';
    }
    
    printf("%c\n", ch);
    
    system_state_t new_state = current_state;
    
    switch (ch) {
        // ===== COMANDOS AC =====
        case '1':
            new_state = STATE_ON;
            execute_ir_command_safe(new_state);
            break;
            
        case '2':
            new_state = STATE_OFF;
            execute_ir_command_safe(new_state);
            break;
            
        case '3':
            printf("AVISO: Este comando causara falha proposital!\n");
            new_state = STATE_TEMP_22;
            execute_ir_command_safe(new_state);
            break;
            
        case '4':
            new_state = STATE_TEMP_20;
            execute_ir_command_safe(new_state);
            break;
            
        case '5':
            new_state = STATE_FAN_1;
            execute_ir_command_safe(new_state);
            break;
            
        case '6':
            new_state = STATE_FAN_2;
            execute_ir_command_safe(new_state);
            break;
        
        // ===== SIMULAÇÃO DE FALHAS =====
        case 'F':
            printf("AVISO: Acionando falha de loop infinito!\n");
            trigger_infinite_loop_fault();
            break;
            
        case 'U':
            printf("AVISO: Acionando falha de UART travada!\n");
            trigger_uart_stuck_fault();
            break;
        
        // ===== UTILITÁRIOS =====
        case 'S':
            print_status();
            break;
            
        case '0':
            print_menu();
            break;
            
        default:
            printf("Comando invalido. Digite '0' para menu.\n");
            break;
    }
}

// ===================== MAIN =====================
int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("\n\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║  SISTEMA IR + WATCHDOG + TELEMETRIA    ║\n");
    printf("║  Raspberry Pi Pico - Transmissor A     ║\n");
    printf("╚════════════════════════════════════════╝\n\n");

    // 1) Inicializa GPIOs
    init_gpio();

    // 2) Inicializa UART para telemetria
    init_uart_telemetry();

    // 3) Inicializa display OLED
    init_display(&ssd);

    // 4) Indicação visual de boot (3 piscadas)
    for (int i = 0; i < 3; i++) {
        gpio_put(LED_BOOT_RED, 1);
        sleep_ms(120);
        gpio_put(LED_BOOT_RED, 0);
        sleep_ms(120);
    }

    // ===== BOOT DIAGNÓSTICO PERSISTENTE =====
    load_persist_data();

    bool reboot_wdt = watchdog_caused_reboot();

    persist.boot_count++;

    if (reboot_wdt) {
        persist.wdt_count++;
        persist.last_reset = 0x001; // WATCHDOG
        // última falha já foi escrita antes do travamento
        printf("⚠ Reset por WATCHDOG\n");
    } else {
        persist.last_reset = 0x000; // NORMAL
        persist.last_fault = 0x000; // limpa falha antiga
        printf("✓ Reset normal (power / manual)\n");
    }

    // Atualiza scratch (uso em tempo real)
    watchdog_hw->scratch[0] = persist.wdt_count;
    watchdog_hw->scratch[1] = persist.last_fault;

    // Grava UMA vez na flash
    save_persist_data();


    // Mostra diagnóstico no OLED
    show_boot_diag(&ssd, reboot_wdt, persist.wdt_count, persist.last_fault);
    sleep_ms(3000);

    // Inicializa sistema IR
    printf("Inicializando sistema IR...\n");
    if (!custom_ir_init(IR_PIN)) {
        printf("✗ ERRO: Falha ao inicializar sistema IR!\n");
        
        while (1) {
            gpio_put(LED_BOOT_RED, 1);
            sleep_ms(100);
            gpio_put(LED_BOOT_RED, 0);
            sleep_ms(100);
        }
    }
    printf("✓ Sistema IR inicializado\n");

    // ===== HABILITA WATCHDOG =====
    printf("Habilitando Watchdog (timeout: %dms)...\n", WDT_TIMEOUT_MS);
    watchdog_enable(WDT_TIMEOUT_MS, true);
    printf("✓ Watchdog ativo!\n\n");

    // Envia primeira telemetria (estado inicial)
    send_telemetry();
    printf("✓ Telemetria ativa!\n");

    // Mostra menu inicial
    print_menu();

    // ===== LOOP PRINCIPAL =====
    absolute_time_t next_display = make_timeout_time_ms(1000);
    absolute_time_t next_led = make_timeout_time_ms(500);
    absolute_time_t next_telemetry = make_timeout_time_ms(TELEMETRY_INTERVAL_MS);
    bool led_state = false;

    while (true) {
        // ===== PROCESSA COMANDOS SERIAL =====
        process_uart_input();

        // ===== TRANSMISSÃO PERIÓDICA DE TELEMETRIA =====
        if (absolute_time_diff_us(get_absolute_time(), next_telemetry) <= 0) {
            send_telemetry();
            next_telemetry = make_timeout_time_ms(TELEMETRY_INTERVAL_MS);
            watchdog_update();
        }

        // ===== LED DE HEARTBEAT =====
        if (absolute_time_diff_us(get_absolute_time(), next_led) <= 0) {
            led_state = !led_state;
            gpio_put(LED_OK_GREEN, led_state);
            next_led = make_timeout_time_ms(500);
        }

        // ===== ATUALIZA DISPLAY PERIODICAMENTE =====
        if (absolute_time_diff_us(get_absolute_time(), next_display) <= 0 || 
            last_display_state != current_state) {
            
            show_running_state(&ssd, current_state);
            last_display_state = current_state;
            next_display = make_timeout_time_ms(1000);
            
            watchdog_update();
        }

        // ===== FEED DO WATCHDOG =====
        watchdog_update();

        sleep_ms(10);
    }

    return 0;
}