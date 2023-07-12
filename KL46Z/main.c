/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "MKL46Z4.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "i2c.h"
#include "sensor.h"
#include "uart.h"
#include "adc.h"
#include "pit.h"
#include "gpio.h"
/*******************************************************************************
* Definitions
*******************************************************************************/

extern volatile uint32_t currentSpO2Value;
extern volatile uint32_t currentPulseRateValue;
double temp = 0;
double flat = 0;
double flon = 0;
char b[10];

/*******************************************************************************
* Codes
*******************************************************************************/
/* Main function */
int main(void) {
    uint8_t value_to_string[5] = {'\0'};
    uint16_t adc_result = 0;
    uint32_t i = 0;
    char c;
    gps *neo7 = (gps *)calloc(1, sizeof(gps));

    /* Initialize for all modude */
    init_SysTick_interrupt();
    i2c_init();
    init_max30100();
    init_uart0();
    init_uart1();
    init_uart2();
    adc_init();
    initPit();
    initGpio();
    
    while(1) {
        while (i <1000) {
                if (UART2->S1 & UART_S1_RDRF_MASK) {
                        i++;
                        c = UART2->D;

                        if(encode(neo7, c)) {
                                get_position(neo7, &flat, &flon);
                                sprintf(b, "%f", flat);
                                send_string_uart0("\nflat: ");
                                send_string_uart0(b);
                                sprintf(b, "%f", flon);
                                send_string_uart0("\nflon: ");
                                send_string_uart0(b);
                        }
                        send_character_uart0(c);
                }
        }
        read_max30100();
        adc_result = adc16_simple_read(1);
        temp = convert_temp(adc_result);
        if (temp < get_temp_env()) {
                temp = 0;
        }
    }

}

/*******************************************************************************
 * EOF
 ******************************************************************************/