/*
 * The I2C (TWI) bus scanner tests all addresses and detects devices
 * that are connected to the SDA and SCL signals.
 * (c) 2023-2025 Tomas Fryza, MIT license
 *
 * Developed using PlatformIO and Atmel AVR platform.
 * Tested on Arduino Uno board and ATmega328P, 16 MHz.
 */

// -- Includes ---------------------------------------------
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <twi.h>            // I2C/TWI library for AVR-GCC
#include <uart.h>           // Peter Fleury's UART library
#include <stdio.h>          // C library. Needed for `sprintf`
#include <util/delay.h>
#include "timer.h"


#define ADD 0x60
#define atd 0x01
#define test 0x00

volatile uint8_t flag_update_uart = 0;
volatile uint8_t altitude_raw[5];
volatile uint8_t pressure_raw[5];
volatile uint8_t status;


void mode_altitude() {

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


void mode_pressure() {

    twi_start();
twi_write((ADD << 1) | TWI_WRITE);
twi_write(0x26);       // CTRL_REG1
twi_write(0x00);       // Standby
twi_stop();
_delay_ms(20);  

    twi_start();
twi_write((ADD << 1) | TWI_WRITE);
twi_write(0x26);
twi_write(0x38);
twi_stop();

twi_start();
twi_write((ADD << 1) | TWI_WRITE);
twi_write(0x26);
twi_write(0x39);  
twi_stop();
}
// -- Function definitions ---------------------------------
/*
 * Function: Main function where the program execution begins
 * Purpose:  Call function to test all I2C (TWI) combinations
 *           and send detected devices to UART.
 * Returns:  none
 */
int main(void)
{
    char uart_msg[10];

    // Initialize USART to asynchronous, 8-N-1, 115200 Bd
    // NOTE: Add `monitor_speed = 115200` to `platformio.ini`
    uart_init(UART_BAUD_SELECT(115200, F_CPU));

    sei();  // Needed for UART

    // I2C Scanner
    uart_puts("Scanning I2C... ");
    twi_init();

    for (uint8_t sla = 0; sla < 128; sla++) {
        if (twi_test_address(sla) == 0) {  // If ACK from Slave
            sprintf(uart_msg, "0x%x ", sla);
            uart_puts(uart_msg);
        }
    }

    if (twi_test_address(ADD) != 0)
    {
        uart_puts("[\x1b[31;1mERROR\x1b[0m] I2C device not detected\r\n");
        while (1);
    }
    uart_puts(" Done\r\n");

  


    // Set Timer 1
    // WRITE YOUR CODE HERE
    tim1_ovf_1sec();
    tim1_ovf_enable()

    // Update "minutes" and "hours" in RTC
    // WRITE YOUR CODE HERE

    // Test OLED display
    // WRITE YOUR CODE HERE

    while (1) {
 if (flag_update_uart == 1)
        {
             mode_altitude();
             _delay_ms(4000);
             twi_readfrom_mem_into(ADD, atd, altitude_raw, 3);
            // Display checksum
            int32_t tHeight = ((int32_t)altitude_raw[0] << 16) |
              ((int32_t)altitude_raw[1] << 8)  |
               (int32_t)altitude_raw[2];

            
               tHeight >>= 4;
            int16_t altitude = tHeight / 16;
        sprintf(uart_msg, "Altitude: %d m\r\n", altitude);
        uart_puts(uart_msg);

       

            // Do not print it again and wait for the new data
            flag_update_uart = 0;
        }
    }

    return 0;
}

ISR(TIMER1_OVF_vect)
{
    static uint8_t n_ovfs = 0;
     
    n_ovfs++;
    // Read the data every 5 secs
    if (n_ovfs >= 5)
    {
        n_ovfs = 0;
       

        flag_update_uart = 1;
    }
}
