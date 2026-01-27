/**
 * Receptor de Telemetria com DVI - Pico B
 * 
 * Funcionalidades:
 * - Core 0: Recepção de dados via UART (GP1/RX)
 * - Core 1: Renderização DVI estável (640x480p)
 * - Proteção via Watchdog Timer
 * - Exibição de telemetria do ar condicionado
 */

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdio.h"
#include <string.h> 
#include "pico/multicore.h"
#include "pico/bootrom.h" 
#include "pico/sem.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/vreg.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/structs/ssi.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "dvi.h"
#include "dvi_serialiser.h"
#include "./include/common_dvi_pin_configs.h"
#include "libtmds/tmds_encode_font_2bpp.h"

// Inclusão do arquivo de fonte
#include "./assets/font_teste.h"
#define FONT_N_CHARS 95
#define FONT_FIRST_ASCII 32

// AJUSTES PRINCIPAIS PARA DUPLICAÇÃO 3X (8x24)
#define FONT_CHAR_WIDTH 8
#define FONT_CHAR_HEIGHT 24
#define FONT_ORIGINAL_HEIGHT 8      
#define FONT_SCALE_FACTOR (FONT_CHAR_HEIGHT / FONT_ORIGINAL_HEIGHT)

#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_640x480p_60hz

// ===================== UART =====================
#define UART_ID          uart0
#define UART_RX_PIN       1   // GP1 - RX do Pico A
#define UART_TX_PIN       0   // GP0 - TX (não usado)
#define UART_BAUD_RATE   115200

// ===================== WATCHDOG =====================
#define WDT_TIMEOUT_MS   10000  // 10 segundos

// ===================== TELEMETRIA =====================
#define TELEM_HEADER     0xAA
#define TELEM_FOOTER     0x55

// Estados do AC (deve ser igual ao transmissor)
typedef enum {
    STATE_OFF,
    STATE_ON,
    STATE_TEMP_20,
    STATE_TEMP_22,
    STATE_FAN_1,
    STATE_FAN_2,
    STATE_MAX
} system_state_t;

// ===================== ESTRUTURA DE TELEMETRIA =====================
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

struct dvi_inst dvi0;

// Definições do terminal de caracteres
#define CHAR_COLS (FRAME_WIDTH / FONT_CHAR_WIDTH)   // 80
#define CHAR_ROWS (FRAME_HEIGHT / FONT_CHAR_HEIGHT) // 20

// Buffers para caracteres e cores
#define COLOUR_PLANE_SIZE_WORDS (CHAR_ROWS * CHAR_COLS * 4 / 32)
char charbuf[CHAR_ROWS * CHAR_COLS];
uint32_t colourbuf[3 * COLOUR_PLANE_SIZE_WORDS];

// ===================== VARIÁVEIS GLOBAIS DE TELEMETRIA =====================
static telemetry_data_t latest_telemetry = {0};
static bool telemetry_received = false;
static uint32_t last_telemetry_time = 0;
static uint32_t telemetry_packet_count = 0;

// ===================== FUNÇÕES DE CARACTERES =====================
static inline void set_char(uint x, uint y, char c) {
    if (x >= CHAR_COLS || y >= CHAR_ROWS)
        return;
    charbuf[x + y * CHAR_COLS] = c;
}

static inline void set_colour(uint x, uint y, uint8_t fg, uint8_t bg) {
    if (x >= CHAR_COLS || y >= CHAR_ROWS)
        return;
    uint char_index = x + y * CHAR_COLS;
    uint bit_index = char_index % 8 * 4;
    uint word_index = char_index / 8;
    for (int plane = 0; plane < 3; ++plane) {
        uint32_t fg_bg_combined = (fg & 0x3) | (bg << 2 & 0xc);
        colourbuf[word_index] = (colourbuf[word_index] & ~(0xfu << bit_index)) | (fg_bg_combined << bit_index);
        fg >>= 2;
        bg >>= 2;
        word_index += COLOUR_PLANE_SIZE_WORDS;
    }
}

// ===================== DESENHO DE BORDA =====================
void draw_border() {
    const uint8_t fg = 0x15; // Cinza
    const uint8_t bg = 0x00; // Preto

    // Cantos
    set_char(0, 0, '+');
    set_colour(0, 0, fg, bg);
    set_char(CHAR_COLS - 1, 0, '+');
    set_colour(CHAR_COLS - 1, 0, fg, bg);
    set_char(0, CHAR_ROWS - 1, '+');
    set_colour(0, CHAR_ROWS - 1, fg, bg);
    set_char(CHAR_COLS - 1, CHAR_ROWS - 1, '+');
    set_colour(CHAR_COLS - 1, CHAR_ROWS - 1, fg, bg);

    // Linhas horizontais
    for (uint x = 1; x < CHAR_COLS - 1; ++x) {
        set_char(x, 0, '-');
        set_colour(x, 0, fg, bg);
        set_char(x, CHAR_ROWS - 1, '-');
        set_colour(x, CHAR_ROWS - 1, fg, bg);
    }
    
    // Linhas verticais
    for (uint y = 1; y < CHAR_ROWS - 1; ++y) {
        set_char(0, y, '|');
        set_colour(0, y, fg, bg);
        set_char(CHAR_COLS - 1, y, '|');
        set_colour(CHAR_COLS - 1, y, fg, bg);
    }
}

// ===================== VALIDAÇÃO DE CHECKSUM =====================
static uint8_t calculate_checksum(telemetry_data_t *data) {
    uint8_t sum = 0;
    uint8_t *ptr = (uint8_t*)data;
    
    for (size_t i = 0; i < sizeof(telemetry_data_t) - 2; i++) {
        sum += ptr[i];
    }
    
    return sum;
}

// ===================== CONVERSÃO DE ESTADO PARA STRING =====================
const char* get_state_string(uint8_t state) {
    switch (state) {
        case STATE_OFF:     return "OFF";
        case STATE_ON:      return "ON";
        case STATE_TEMP_20: return "20C";
        case STATE_TEMP_22: return "22C";
        case STATE_FAN_1:   return "FAN1";
        case STATE_FAN_2:   return "FAN2";
        default:            return "???";
    }
}

const char* get_fault_string(uint32_t fault) {
    switch (fault) {
        case 0x00: return "NENHUMA";
        case 0x01: return "LOOP INF";
        case 0x02: return "CMD 22C";
        case 0x03: return "UART TRAV";
        default:   return "DESCONHEC";
    }
}

// ===================== ATUALIZAÇÃO DO DISPLAY =====================
void update_display() {
    // Limpa área interna (preserva borda)
    for (uint y = 1; y < CHAR_ROWS - 1; ++y) {
        for (uint x = 1; x < CHAR_COLS - 1; ++x) {
            set_char(x, y, ' ');
            set_colour(x, y, 0x00, 0x00);
        }
    }

    // Linha inicial para exibição
    int y = CHAR_ROWS / 2 - 2;
    int x_start = 2;

    // Cores
    const uint8_t fg_label = 0x3f;  // Branco
    const uint8_t bg_label = 0x00;  // Preto
    const uint8_t fg_value = 0x3C;  // Amarelo
    const uint8_t bg_value = 0x00;  // Preto

    if (!telemetry_received) {
        // Mensagem de aguardando dados
        const char *msg = "Aguardando telemetria...";
        int msg_len = strlen(msg);
        int x = (CHAR_COLS - msg_len) / 2;
        
        for (int i = 0; i < msg_len; ++i) {
            set_char(x + i, CHAR_ROWS / 2, msg[i]);
            set_colour(x + i, CHAR_ROWS / 2, 0x30, 0x00); // Vermelho
        }
        return;
    }

    // Buffer para formatação
    char line[80];

    // LINHA 1: RST
    snprintf(line, sizeof(line), "RST: %lu", (unsigned long)latest_telemetry.wdt_resets);
    int len = strlen(line);
    for (int i = 0; i < len; ++i) {
        set_char(x_start + i, y, line[i]);
        if (i < 4) {
            set_colour(x_start + i, y, fg_label, bg_label);
        } else {
            set_colour(x_start + i, y, fg_value, bg_value);
        }
    }

    // LINHA 2: Último Comando
    y++;
    const char* cmd_str = get_state_string(latest_telemetry.last_command);
    snprintf(line, sizeof(line), "Ultimo comando: %s", cmd_str);
    len = strlen(line);
    for (int i = 0; i < len; ++i) {
        set_char(x_start + i, y, line[i]);
        if (i < 16) {
            set_colour(x_start + i, y, fg_label, bg_label);
        } else {
            set_colour(x_start + i, y, fg_value, bg_value);
        }
    }

    // LINHA 3: Código Funcional (última falha)
    y++;
    const char* fault_str = get_fault_string(latest_telemetry.last_fault);
    snprintf(line, sizeof(line), "Codigo funcional: %s", fault_str);
    len = strlen(line);
    for (int i = 0; i < len; ++i) {
        set_char(x_start + i, y, line[i]);
        if (i < 18) {
            set_colour(x_start + i, y, fg_label, bg_label);
        } else {
            set_colour(x_start + i, y, fg_value, bg_value);
        }
    }

    // LINHA 4: Info adicional
    y++;
    snprintf(line, sizeof(line), "OPS IR: %lu  PKT: %lu", 
             (unsigned long)latest_telemetry.ir_operations,
             (unsigned long)telemetry_packet_count);
    len = strlen(line);
    for (int i = 0; i < len; ++i) {
        set_char(x_start + i, y, line[i]);
        set_colour(x_start + i, y, 0x0f, 0x00); // Cinza
    }
}

// ===================== CORE 1 - RENDERIZAÇÃO DVI =====================
void core1_main() {
    dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
    dvi_start(&dvi0);
    
    while (true) {
        for (uint y = 0; y < FRAME_HEIGHT; ++y) {
            uint font_row = (y % FONT_CHAR_HEIGHT) / FONT_SCALE_FACTOR;
            
            uint32_t *tmdsbuf;
            queue_remove_blocking(&dvi0.q_tmds_free, &tmdsbuf);
            
            for (int plane = 0; plane < 3; ++plane) {
                tmds_encode_font_2bpp(
                    (const uint8_t*)&charbuf[y / FONT_CHAR_HEIGHT * CHAR_COLS],
                    &colourbuf[y / FONT_CHAR_HEIGHT * (COLOUR_PLANE_SIZE_WORDS / CHAR_ROWS) + plane * COLOUR_PLANE_SIZE_WORDS],
                    tmdsbuf + plane * (FRAME_WIDTH / DVI_SYMBOLS_PER_WORD),
                    FRAME_WIDTH,
                    (const uint8_t*)&font_8x8[font_row * FONT_N_CHARS] - FONT_FIRST_ASCII
                );
            }
            
            queue_add_blocking(&dvi0.q_tmds_valid, &tmdsbuf);
        }
        
        // Feed do watchdog no Core 1
        watchdog_update();
    }
}

// ===================== RECEPÇÃO DE TELEMETRIA =====================
bool receive_telemetry_packet(telemetry_data_t *packet) {
    static uint8_t rx_buffer[sizeof(telemetry_data_t)];
    static int rx_index = 0;
    static bool synced = false;
    
    // Lê bytes disponíveis
    while (uart_is_readable(UART_ID)) {
        uint8_t byte = uart_getc(UART_ID);
        
        // Sincronização: procura pelo header
        if (!synced) {
            if (byte == TELEM_HEADER) {
                rx_buffer[0] = byte;
                rx_index = 1;
                synced = true;
            }
            continue;
        }
        
        // Acumula bytes
        rx_buffer[rx_index++] = byte;
        
        // Pacote completo?
        if (rx_index >= sizeof(telemetry_data_t)) {
            synced = false;
            rx_index = 0;
            
            // Valida footer
            if (rx_buffer[sizeof(telemetry_data_t) - 1] != TELEM_FOOTER) {
                continue;
            }
            
            // Copia para estrutura temporária
            telemetry_data_t temp;
            memcpy(&temp, rx_buffer, sizeof(telemetry_data_t));
            
            // Valida checksum
            uint8_t calc_checksum = calculate_checksum(&temp);
            if (temp.checksum != calc_checksum) {
                continue;
            }
            
            // Pacote válido!
            memcpy(packet, &temp, sizeof(telemetry_data_t));
            return true;
        }
    }
    
    return false;
}

// ===================== CORE 0 - MAIN =====================
int __not_in_flash("main") main() {
    stdio_init_all();
    sleep_ms(4000);
    
    printf("\n\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║  RECEPTOR DVI + WATCHDOG               ║\n");
    printf("║  Raspberry Pi Pico - Receptor B        ║\n");
    printf("╚════════════════════════════════════════╝\n\n");

    // ===== DIAGNÓSTICO DE REBOOT =====
    bool reboot_wdt = watchdog_caused_reboot();
    
    if (reboot_wdt) {
        watchdog_hw->scratch[0] = watchdog_hw->scratch[0] + 1;
        printf("⚠ AVISO: Reset por WATCHDOG!\n");
    } else {

        printf("✓ Boot normal\n");
    }
    
    printf("Resets por WDT: %lu\n\n", (unsigned long)watchdog_hw->scratch[0]);

    // ===== CONFIGURA DVI (UMA VEZ APENAS!) =====
    printf("Configurando DVI...\n");
    vreg_set_voltage(VREG_VSEL);
    sleep_ms(10);
    set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);
    
    dvi0.timing = &DVI_TIMING;
    dvi0.ser_cfg = picodvi_dvi_cfg;
    dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());
    printf("✓ DVI configurado (Clock: %lu kHz)\n", clock_get_hz(clk_sys) / 1000);

    // ===== CONFIGURA UART =====
    printf("Configurando UART...\n");
    uart_init(UART_ID, UART_BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);
    
    uint32_t baud_actual = uart_set_baudrate(UART_ID, UART_BAUD_RATE);
    printf("✓ UART configurada (Baud: %lu, GP%d/RX)\n\n", baud_actual, UART_RX_PIN);

    // Limpa buffer UART
    sleep_ms(100);
    while (uart_is_readable(UART_ID)) {
        uart_getc(UART_ID);
    }

    // Limpa tela
    for (uint y = 0; y < CHAR_ROWS; ++y) {
        for (uint x = 0; x < CHAR_COLS; ++x) {
            set_char(x, y, ' ');
            set_colour(x, y, 0x00, 0x00);
        }
    }

    // Desenha borda
    draw_border();
    update_display();

    // ===== HABILITA WATCHDOG =====
    printf("Habilitando Watchdog (%dms)...\n", WDT_TIMEOUT_MS);
    watchdog_enable(WDT_TIMEOUT_MS, true);
    printf("✓ Watchdog ativo!\n\n");

    // ===== INICIA CORE 1 (RENDERIZAÇÃO DVI) =====
    hw_set_bits(&bus_ctrl_hw->priority, BUSCTRL_BUS_PRIORITY_PROC1_BITS);
    multicore_launch_core1(core1_main);
    printf("✓ Core 1 iniciado\n\n");

    // ===== LOOP PRINCIPAL - CORE 0 (RECEPÇÃO UART) =====
    absolute_time_t next_display_update = make_timeout_time_ms(100);
    
    printf("Aguardando pacotes de telemetria...\n");
    
    while (true) {
        // Tenta receber pacote de telemetria
        if (receive_telemetry_packet(&latest_telemetry)) {
            telemetry_received = true;
            last_telemetry_time = to_ms_since_boot(get_absolute_time());
            telemetry_packet_count++;
            
            printf("PKT #%lu: STATE=%s, CMD=%s, RST=%lu, FLT=%s\n",
                   (unsigned long)telemetry_packet_count,
                   get_state_string(latest_telemetry.ac_state),
                   get_state_string(latest_telemetry.last_command),
                   (unsigned long)latest_telemetry.wdt_resets,
                   get_fault_string(latest_telemetry.last_fault));
        }
        
        // Atualiza display periodicamente
        if (absolute_time_diff_us(get_absolute_time(), next_display_update) <= 0) {
            update_display();
            next_display_update = make_timeout_time_ms(100);
        }
        
        // Verifica timeout de recepção
        if (telemetry_received && 
            (to_ms_since_boot(get_absolute_time()) - last_telemetry_time) > 2000) {
            printf("AVISO: Sem telemetria ha mais de 2 segundos!\n");
        }
        
        // Feed do watchdog no Core 0
        watchdog_update();
        
        sleep_ms(10);
    }

    return 0;
}