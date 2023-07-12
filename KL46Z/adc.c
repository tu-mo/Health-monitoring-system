/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "MKL46Z4.h"
#include "adc.h"

/*******************************************************************************
 * Codes
 ******************************************************************************/
/* Function is used initialize ADC */
void adc_init() {
    /* Enable clock for PORTE */
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    /* Initialize for PTE16 to ADC mode*/
    PORTE->PCR[16] &= ~(PORT_PCR_MUX_MASK);
    PORTE->PCR[16] |= PORT_PCR_MUX(0);
    
    /* Enable clock for ADC0 */
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
    
    /* Initialize for ADC 
       Software trigger
       Clock div by 4, long sample time, single ended 12 bit, bus clock */
    ADC0->SC2 &= ~0x40; 
    ADC0->CFG1 = 0x40 | 0x10 | 0x04 | 0x00;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
