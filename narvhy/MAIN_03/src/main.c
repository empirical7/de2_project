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
#define R0              42.0f  // calibration resistance
#define NUM_SAMPLES     50     // averaging for CO2 ADC

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
// Converts raw analog input into particles per milion (ppm for short)
float getCO2ppm(void)
{
    uint32_t sum = 0;
    //Averaging samples using for cycle
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        //adc_read function uses ADC to convert voltage on input to raw data 
        sum += adc_read(CO2_ADC_CH);
    }

    uint16_t raw = (uint16_t)(sum / NUM_SAMPLES);

    float V = raw * (5.0f / 1023.0f);

    // Guard – checking for unreliable data
    if (V < 0.1f || V > 4.9f)
        return 0.0f;
    // calculation of CO2 in air out of raw data using voltage on analog input and calibration resistance R0
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

        
        twi_readfrom_mem_into(DHT_ADR, DHT_HUM_MEM, dht12_values, 5);

       
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

    // I2C scan
    char uart_msg[20];
    uart_puts("Scanning I2C... ");
    for (uint8_t sla = 0; sla < 128; sla++)
    {
        if (twi_test_address(sla) == 0)
        {
            sprintf(uart_msg, "0x%x ", sla);
            uart_puts(uart_msg);
        }
    }
    uart_puts("Done\r\n");

    if (twi_test_address(ADD) != 0)
    {
        uart_puts("[ERROR] Altitude sensor not detected\r\n");
        
    }

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
        // ---------- Измерение высоты (раз в 5 сек) ----------
        if (flag_altitude)
        {
            flag_altitude = 0;

            // Настраиваем сенсор и запускаем измерение
            mode_altitude();

            // Ждём завершения конверсии (как в твоём коде)
            _delay_ms(4000);

            // Читаем 3 байта высоты с регистра 0x01
            twi_readfrom_mem_into(ADD, ATD_REG, altitude_raw, 3);

            int32_t tHeight = ((int32_t)altitude_raw[0] << 16) |
                              ((int32_t)altitude_raw[1] << 8)  |
                              (int32_t)altitude_raw[2];

            tHeight >>= 4;                         // 20-битное значение
            altitude_m = (int16_t)(tHeight / 16);  // делим по даташиту

            sprintf(uart_buffer, "Altitude: %d m\r\n", altitude_m);
            uart_puts(uart_buffer);
        }

        // ---------- Раз в секунду: пыль + CO2 + OLED ----------
        if (flag_update)
        {
            flag_update = 0;

            // --- Пыль (Sharp dust sensor) ------------------------

            // Включаем ИК-LED (active low)
            gpio_write_low(&PORTD, LED_PIN);
            _delay_us(LONG_DELAY);

            // Читаем ADC (канал пыли)
            adc_value = adc_read(DUST_ADC_CH);

            // Добиваем импульс до 320 мкс
            _delay_us(SHORT_DELAY);

            // Выключаем LED
            gpio_write_high(&PORTD, LED_PIN);

            // Переводим вольты (Vref = 5 В, 10 bit ADC)
            voltage = adc_value * (5.0f / 1024.0f);

            // Приблизительный расчёт dust density
            if (voltage > 0.1f)
            {
                dustDensity = 0.17f * voltage;   // коэффициент как у тебя
            }
            else
            {
                dustDensity = 0.0f;
            }

            dtostrf(dustDensity, 4, 2, dust_str);  // ширина 4, 2 знака
            dtostrf(voltage,    4, 2, volt_str);

            // --- CO2 (MQ sensor) ---------------------------------
            co2_ppm = getCO2ppm();
            co2_int = (long)co2_ppm;
            co2_dec = (int)((co2_ppm - (float)co2_int) * 100.0f);
            if (co2_dec < 0) co2_dec = -co2_dec;   // на всякий случай

            // UART вывод
            snprintf(uart_buffer, sizeof(uart_buffer),
                     "V: %s V | Dust: %s mg/m3 | CO2: %ld.%02d ppm\r\n",
                     volt_str, dust_str, co2_int, co2_dec);
            uart_puts(uart_buffer);

            // --- OLED вывод ---------------------------------------
            oled_clrscr();

            // Верхняя строка – название
            oled_gotoxy(0, 0);
            oled_puts("Dust CO2 DHT Alt");

            // CO2
            oled_gotoxy(0, 1);
            sprintf(co2_str, "CO2:%ld.%02dppm", co2_int, co2_dec);
            oled_puts(co2_str);

            // Напряжение
            oled_gotoxy(0, 3);
            sprintf(oled_buf, "V: %s V", volt_str);
            oled_puts(oled_buf);

            // Пыль
            oled_gotoxy(0, 5);
            sprintf(oled_buf, "D: %s mg/m3", dust_str);
            oled_puts(oled_buf);

            // DHT12: температура и влажность
            oled_gotoxy(0, 2);
            oled_puts("T[C]:");
            oled_gotoxy(0, 4);
            oled_puts("H[%]:");

            // Температура (байты 2 и 3)
            oled_gotoxy(6, 2);
            sprintf(dht_str, "%u.%u", dht12_values[2], dht12_values[3]);
            oled_puts(dht_str);

            // Влажность (байты 0 и 1)
            oled_gotoxy(6, 4);
            sprintf(dht_str, "%u.%u", dht12_values[0], dht12_values[1]);
            oled_puts(dht_str);

            // Высота — последняя строка
            oled_gotoxy(0, 7);
            sprintf(oled_buf, "Alt: %d m", altitude_m);
            oled_puts(oled_buf);

            // Вывод буфера на дисплей
            oled_display();
        }
    }

    // Never reached
    return 0;

}
