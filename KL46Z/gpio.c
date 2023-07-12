/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <stdio.h>

#include "MKL46Z4.h"
/*******************************************************************************
* Codes
*******************************************************************************/
/* Function is used initialize GPIO */
void initGpio() {
        SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    /* Initialize PTE2*/
    /* Set the PTE2 pin multiplexer to GPIO mode */
    PORTE->PCR[2] = PORT_PCR_MUX(1);
    /* Set the pin's direction to output */ 
    FPTE->PDDR |= 1 << 2;
    /* Set the initial output state to low */
    FPTE->PDOR |= 0 << 2;
} 

/* Function is used to change state of buzzer */
void change_state(const uint8_t state) {
    FPTE->PDOR |= state << 2;
}
/*******************************************************************************
 * EOF
 ******************************************************************************/