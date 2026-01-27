/**
 * custom_ir.h
 * Interface para controle IR com DMA
 */

#ifndef CUSTOM_IR_H
#define CUSTOM_IR_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Inicializa o sistema IR com DMA
 * @param gpio_pin Pino GPIO para saída IR
 * @return true se inicializado com sucesso
 */
bool custom_ir_init(uint gpio_pin);

/**
 * Envia um sinal RAW via DMA
 * @param signal Array de timings em microsegundos
 * @param length Tamanho do array
 */
void send_raw_signal(const uint16_t* signal, size_t length);

/**
 * Funções de conveniência para controle do AC
 */
void turn_off_ac(void);
void turn_on_ac(void);
void set_temp_22c(void);
void set_temp_20c(void);
void set_fan_level_1(void);
void set_fan_level_2(void);

/**
 * Demonstração automática
 */
void ir_demo(void);

#endif // CUSTOM_IR_H