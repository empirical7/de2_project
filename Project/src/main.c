/*
 * Combined main.c:
 * - Sharp optical dust sensor on ADC0
 * - CO2 sensor on ADC1
 * - DHT12 (I2C) temperature & humidity
 * - Altitude sensor (0x60) on I2C
 * - OLED display output
 * - UART debug output
 *
 * MCU: ATmega328P @ 16 MHz (Arduino Uno)
 */

// -- Includes -------------------------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <uart.h>
#include <gpio.h>
#include "timer.h"
#include <adc.h>
#include <oled.h>
#include <twi.h>

// -- Defines --------------------------------------------------------
#define SHORT_DELAY     40
#define LONG_DELAY      280     // Sharp dust sensor on ADC0 (A0)

// Sharp dust sensor LED pin (IR LED control)
#define LED_PIN         PD2
#define DUST_ADC_CH     0      // Sharp dust sensor on ADC0 (A0)

// CO2 sensor definitions
#define CO2_ADC_CH      1      // CO2 sensor on ADC1 (A1)
#define RL              20.0f  // load resistor (kOhm) - adjust to your HW
#define R0              35.0f  // calibration resistance
#define NUM_SAMPLES     5     // averaging for CO2 ADC

// DHT12 I2C address and memory positions
#define DHT_ADR         0x5c
#define DHT_HUM_MEM     0
#define DHT_TEMP_MEM    2

// Altitude sensor (e.g. MPL3115A2) I2C address and registers
#define ADD             0x60
#define ATD_REG         0x01   // altitude data starts here

// -- Global variables -----------------------------------------------
volatile uint8_t flag_update = 0;       // 1x per second – dust+CO2+OLED
volatile uint8_t flag_altitude = 0;     // 1x per 5s – altitude measurement

volatile uint8_t dht12_values[5];       // humidity & temperature from DHT12
uint8_t altitude_raw[3];                // raw altitude bytes
int16_t altitude_m = 0;                 // latest altitude in meters

// -- Functions ------------------------------------------------------

// Altitude sensor to altitude mode (from твой кода)
void mode_altitude(void)
{
    twi_start();
    twi_write((ADD << 1) | TWI_WRITE);
    twi_write(0x26);
    twi_write(0x00);
    twi_stop();

    twi_start();
    twi_write((ADD << 1) | TWI_WRITE);
    twi_write(0x26);
    twi_write(0xB8);
    twi_stop();

    twi_start();
    twi_write((ADD << 1) | TWI_WRITE);
    twi_write(0x26);
    twi_write(0xB9);
    twi_stop();
}

// CO2 calculation function 
float getCO2ppm(void)
{
    uint32_t sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        sum += adc_read(CO2_ADC_CH);
    }

    uint16_t raw = (uint16_t)(sum / NUM_SAMPLES);

    float V = raw * (5.0f / 1023.0f);

    // Guard – 
    if (V < 0.1f || V > 4.9f)
        return 0.0f;

    float RS = (5.0f - V) * RL / V;
    float ratio = RS / R0;

    float a_co2 = 40.0f;
    float b_co2 = -2.77f;

    return a_co2 * powf(ratio, b_co2);
}

// -- Interrupt service routines -------------------------------------
/*
 * Timer/Counter1 overflow interrupt
 */
ISR(TIMER1_OVF_vect)
{
    static uint8_t n_ovfs = 0;

    flag_update = 1;   
    n_ovfs++;
    if (n_ovfs >= 5)
    {
        n_ovfs = 0;
        
        //twi_readfrom_mem_into(DHT_ADR, DHT_HUM_MEM, dht12_values, 5);
        //mode_altitude();   
         //_delay_ms(2000);
       
        flag_altitude = 1;
    }
}

// -- Main -----------------------------------------------------------
int main(void)
{
    // UART init (debug output)
    uart_init(UART_BAUD_SELECT(115200, F_CPU));

    // I2C init (DHT12, OLED, altitude sensor)
    twi_init();

  
    // OLED init
    oled_init(OLED_DISP_ON);
    oled_clrscr();
    oled_charMode(NORMALSIZE);
    oled_gotoxy(0, 0);
    oled_puts("Dust+CO2+DHT+Alt");
    oled_gotoxy(0, 2);
    oled_puts("Init...");
    oled_display();

    // GPIO: LED pin for Sharp dust sensor
    gpio_mode_output(&DDRD, LED_PIN);
    gpio_write_high(&PORTD, LED_PIN);   // LED off (active low)

    // ADC init 
    adc_init();

    //altitude mode ini
    mode_altitude();

    // Timer1 overflow every ~1 s
    tim1_ovf_1sec();
    tim1_ovf_enable();

    // Enable global interrupts
    sei();

    // Variables for calculations and text buffers
    char uart_buffer[80];
    char oled_buf[20];
    char volt_str[10];
    char dust_str[10];
    char co2_str[16];
    char dht_str[8];

    uint16_t adc_value = 0;
    float voltage = 0.0f;
    float dustDensity = 0.0f;

    float co2_ppm = 0.0f;
    long co2_int = 0;
    int co2_dec = 0;

    uart_puts("Optical Dust + CO2 + DHT12 + Alt sensor started\r\n");

    // Main loop
    while (1)
    {
        // ----------  ----------
        if (flag_altitude)
        {
            flag_altitude = 0;

            
            twi_readfrom_mem_into(ADD, ATD_REG, altitude_raw, 3);

            int32_t tHeight = ((int32_t)altitude_raw[0] << 16) |
                              ((int32_t)altitude_raw[1] << 8)  |
                              (int32_t)altitude_raw[2];

            tHeight >>= 4;                         
            altitude_m = (int16_t)(tHeight / 16);  

            sprintf(uart_buffer, "Altitude: %d m\r\n", altitude_m);
            uart_puts(uart_buffer);
        }

        // ---------- dust + CO2 + OLED ----------
        if (flag_update)
        {
            flag_update = 0;

            // --- (Sharp dust sensor) ------------------------


            gpio_write_low(&PORTD, LED_PIN);
            _delay_us(LONG_DELAY);

            adc_value = adc_read(DUST_ADC_CH);

        
            _delay_us(SHORT_DELAY);

            //  LED
            gpio_write_high(&PORTD, LED_PIN);

            //  (Vref = 5 В, 10 bit ADC)
            voltage = adc_value * (5.0f / 1024.0f);

            //  dust density
            if (voltage > 0.1f)
            {
                dustDensity = 0.17f * voltage;   
            }
            else
            {
                dustDensity = 0.0f;
            }

            dtostrf(dustDensity, 4, 2, dust_str);  
            dtostrf(voltage,    4, 2, volt_str);

            // --- CO2 (MQ sensor) ---------------------------------
            co2_ppm = getCO2ppm();
            co2_int = (long)co2_ppm;
            co2_dec = (int)((co2_ppm - (float)co2_int) * 100.0f);
            if (co2_dec < 0) co2_dec = -co2_dec;   

            // UART output
            snprintf(uart_buffer, sizeof(uart_buffer),
                     "V: %s V | Dust: %s mg/m3 | CO2: %ld.%02d ppm\r\n",
                     volt_str, dust_str, co2_int, co2_dec);
            uart_puts(uart_buffer);

            // --- OLED output ---------------------------------------
            oled_clrscr();

            // Title
            oled_gotoxy(0, 0);
            oled_puts("Indoor Air Quality");

            // CO2
            oled_gotoxy(0, 4);
            sprintf(co2_str, "CO2:%ld.%02dppm", co2_int, co2_dec);
            oled_puts(co2_str);

            // dust
            oled_gotoxy(0, 5);
            sprintf(oled_buf, "D: %s mg/m3", dust_str);
            oled_puts(oled_buf);

            // teplota a vlhkost 
            oled_gotoxy(0, 2);
            oled_puts("T[C]:");
            oled_gotoxy(0, 3);
            oled_puts("H[%]:");

            // teplota (bite 2 and 3)
            oled_gotoxy(6, 2);
            sprintf(dht_str, "%u.%u", dht12_values[2], dht12_values[3]);
            oled_puts(dht_str);

            // vlhkost (bzte 0 and 1)
            oled_gotoxy(6, 3);
            sprintf(dht_str, "%u.%u", dht12_values[0], dht12_values[1]);
            oled_puts(dht_str);

            // výška 
            oled_gotoxy(0, 7);
            sprintf(oled_buf, "Alt: %d m", altitude_m);
            oled_puts(oled_buf);

            // buffer to display
            oled_display();
        }
    }

    // Never reached
    return 0;
}