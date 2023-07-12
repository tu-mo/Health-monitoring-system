/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "MKL46Z4.h"
#include "uart.h"
#include "gpio.h"

/*******************************************************************************
* Definitions
*******************************************************************************/
extern volatile uint32_t currentSpO2Value;
extern volatile uint32_t currentPulseRateValue;
extern double temp;
extern double flat;
extern double flon;
extern char b[10];

/*******************************************************************************
 * Codes
 ******************************************************************************/
/* Function is used to initialize PIT */
void initPit() {
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK; //Enable clock for PIT
    PIT->MCR &= ~(PIT_MCR_MDIS_MASK); // Clear MDIS fields --> Module Control Register
    PIT->MCR |= 1;
    PIT->CHANNEL[0].LDVAL = 0x1400000;

    PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK;
    NVIC_EnableIRQ(PIT_IRQn);
    PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
}

/*  Function is used to initialize PIT_IRQHandler handler*/
void PIT_IRQHandler(void) {
    if(PIT->CHANNEL[0].TFLG == 1) {
            PIT->CHANNEL[0].TFLG |= 1;
            
            sprintf(b, "%d", currentSpO2Value);
            send_string_uart1(b);
            send_string_uart1("\r\n");
            sprintf(b, "%d", currentPulseRateValue);
            send_string_uart1(b);
            send_string_uart1("\r\n");
            sprintf(b, "%.1f", temp);
            send_string_uart1(b);
            send_string_uart1("\r\n");
            sprintf(b, "%f", flat);
            send_string_uart1(b);
            send_string_uart1("\r\n");
            sprintf(b, "%f", flon);
            send_string_uart1(b);
            send_string_uart1("\r\n");
            
            if ((((currentPulseRateValue < 60) || currentPulseRateValue > 100) && (currentPulseRateValue != 0)) || 
                (((temp < 36) || (temp > 37.5)) && (temp != 0)) || 
                 (((currentSpO2Value < 95) || (currentSpO2Value > 100)) && (currentSpO2Value != 0))) {
                        change_state(1);
                 }
            else {
              change_state(0);
            }

    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/