#include <adc.h>

/***********************************************************************
 * 
 * Example of usage Analog-to-digital conversion from examples/adc/src/main.c
 * on Tomas Fryza's GitHub
 *
 ***********************************************************************/

// -- Function definitions -------------------------------------------
void adc_init(void)
{
    // Select ADC voltage reference to "AVcc with external capacitor at AREF pin"
    ADMUX = ADMUX | (1<<REFS0);
    // Select input channel ADC0 (voltage divider pin)
    ADMUX = ADMUX & ~(1<<MUX3 | 1<<MUX2 | 1<<MUX1 | 1<<MUX0);
    // Enable ADC module
    ADCSRA = ADCSRA | (1<<ADEN);
    // Enable conversion complete interrupt
    //ADCSRA = ADCSRA | (1<<ADIE); nepotrebuju preruseni
    // Set clock prescaler to 128
    ADCSRA = ADCSRA | (1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0);
}

//Funkce pro mereni hodnoty z analogoveho pinu
uint16_t adc_read(uint8_t channel)
{
    // Vyber kanalu
    ADMUX &= 0xF0;
    // Piseme cislo kanalu
    ADMUX |= (channel & 0x0F);
    // Start mereni
    ADCSRA |= (1 << ADSC);
    // Cekame na dokonceni mereni
    while (ADCSRA & (1 << ADSC));
    //Vraceni
    return ADC;
}