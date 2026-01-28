/**
 * Receptor de Telemetria com DVI + ESPELHO SERIAL
 * Correção: Proteção contra Boot Loop + Timeout de Comunicação
 */

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdio.h"
#include <string.h>
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "pico/time.h"

// ===================== CONFIGURAÇÕES =====================
#define UART_ID           uart0
#define UART_RX_PIN       1
#define UART_TX_PIN       0
#define UART_BAUD_RATE    115200

#define WDT_TIMEOUT_MS    8000
#define CARENCIA_RESET_MS 5000
#define TELEMETRY_TIMEOUT_MS 2000  // <-- NOVO

#define TELEM_HEADER      0xAA
#define TELEM_FOOTER      0x55

// ===================== ESTRUTURAS =====================
typedef struct __attribute__((packed)) {
    uint8_t header;
    uint8_t ac_state;
    uint8_t last_command;
    uint8_t ir_pending;
    uint32_t uptime_ms;
    uint32_t wdt_resets;
    uint32_t last_fault;
    uint32_t ir_operations;
    uint8_t checksum;
    uint8_t footer;
} telemetry_data_t;

static telemetry_data_t latest_telemetry;
static bool telemetry_received = false;
static uint32_t telemetry_packet_count = 0;
static bool alerta_wdt = false;
static absolute_time_t last_packet_time;

// ===================== FUNÇÕES AUXILIARES =====================
static uint8_t calculate_checksum(telemetry_data_t *data) {
    uint8_t sum = 0;
    uint8_t *ptr = (uint8_t*)data;
    for (size_t i = 0; i < sizeof(telemetry_data_t) - 2; i++) {
        sum += ptr[i];
    }
    return sum;
}

const char* get_state_string(uint8_t state) {
    switch (state) {
        case 0: return "OFF";
        case 1: return "ON";
        case 2: return "20C";
        case 3: return "22C";
        case 4: return "FAN1";
        case 5: return "FAN2";
        default: return "???";
    }
}

const char* get_fault_string(uint32_t fault) {
    switch (fault) {
        case 0x00: return "NENHUMA";
        case 0x01: return "LOOP INF";
        case 0x02: return "CMD 22C";
        case 0x03: return "UART TRAV";
        default:   return "ERRO CRITICO";
    }
}

// ===================== RECEPÇÃO UART =====================
bool receive_telemetry_packet(telemetry_data_t *packet) {
    static uint8_t rx_buffer[sizeof(telemetry_data_t)];
    static int rx_index = 0;
    static bool synced = false;

    while (uart_is_readable(UART_ID)) {
        uint8_t byte = uart_getc(UART_ID);

        if (!synced) {
            if (byte == TELEM_HEADER) {
                rx_buffer[0] = byte;
                rx_index = 1;
                synced = true;
            }
            continue;
        }

        rx_buffer[rx_index++] = byte;

        if (rx_index >= sizeof(telemetry_data_t)) {
            synced = false;
            rx_index = 0;

            if (rx_buffer[sizeof(telemetry_data_t) - 1] != TELEM_FOOTER)
                return false;

            telemetry_data_t temp;
            memcpy(&temp, rx_buffer, sizeof(temp));

            if (temp.checksum != calculate_checksum(&temp))
                return false;

            memcpy(packet, &temp, sizeof(temp));
            return true;
        }
    }
    return false;
}

// ===================== DISPLAY SERIAL =====================
void print_display_serial(void) {
    printf("\033[2J\033[H");
    printf("========================================\n");
    printf("        TELEMETRIA - RECEPTOR B         \n");
    printf("========================================\n");

    if (alerta_wdt) {
        uint32_t uptime = to_ms_since_boot(get_absolute_time());
        if (uptime < CARENCIA_RESET_MS) {
            printf("Sincronizando... (%lus restantes)\n",
                   (CARENCIA_RESET_MS - uptime) / 1000);
        }
        printf("----------------------------------------\n");
    }

    if (!telemetry_received) {
        printf("\n   Aguardando telemetria...\n\n");
        printf("----------------------------------------\n");
        return;
    }

    printf("RST TX: %lu\n", (unsigned long)latest_telemetry.wdt_resets);
    printf("Ultimo comando: %s\n", get_state_string(latest_telemetry.last_command));
    printf("Status Transmissor: %s\n", get_fault_string(latest_telemetry.last_fault));
    printf("OPS IR: %lu  PKTS: %lu\n",
           (unsigned long)latest_telemetry.ir_operations,
           (unsigned long)telemetry_packet_count);
    printf("----------------------------------------\n");
}

// ===================== MAIN =====================
int main() {
    stdio_init_all();
    sleep_ms(2000);

    if (watchdog_caused_reboot()) {
        alerta_wdt = true;
    }

    uart_init(UART_ID, UART_BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    watchdog_enable(WDT_TIMEOUT_MS, true);

    absolute_time_t next_update = make_timeout_time_ms(200);

    while (true) {

        if (receive_telemetry_packet(&latest_telemetry)) {
            telemetry_received = true;
            telemetry_packet_count++;
            last_packet_time = get_absolute_time();

            if (latest_telemetry.last_fault >= 0x01 &&
                latest_telemetry.last_fault <= 0x03) {

                if (to_ms_since_boot(get_absolute_time()) > CARENCIA_RESET_MS) {
                    watchdog_reboot(0, 0, 0);
                }
            }

            watchdog_update();
        }

        // --- TIMEOUT DE COMUNICAÇÃO ---
        if (telemetry_received &&
            absolute_time_diff_us(get_absolute_time(), last_packet_time) >
            TELEMETRY_TIMEOUT_MS * 1000) {

            telemetry_received = false;
        }

        if (absolute_time_diff_us(get_absolute_time(), next_update) <= 0) {
            print_display_serial();
            next_update = make_timeout_time_ms(200);
        }

        watchdog_update();
        sleep_ms(10);
    }
}
