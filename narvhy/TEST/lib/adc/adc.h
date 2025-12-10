#ifndef ADC_H
#define ADC_H

#include <avr/io.h>

/**
 * @brief Inicializace ADC
 */
void adc_init(void);

/**
 * @brief Cteni hodnoty z analogoveho pinu
 * @param channel Cislo kanalu (0-7)
 * @return Hodnota z ADC (0-1023)
 */
uint16_t adc_read(uint8_t channel);

#endif