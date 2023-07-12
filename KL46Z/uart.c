/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "MKL46Z4.h"
#include "uart.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
volatile int32_t msTicks = 0;
volatile uint32_t micros = 0;

/*******************************************************************************
 * Codes
 ******************************************************************************/
/* Function is used to initialize systick interrupt*/
void init_SysTick_interrupt() {
    SysTick->LOAD = SystemCoreClock/1000; //configured the SysTick to count in 1ms
    /* Select Core Clock & Enable SysTick & Enable Interrupt */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |SysTick_CTRL_TICKINT_Msk
                    |SysTick_CTRL_ENABLE_Msk;
}

/* Function is used to initialize sysTick interrupt handler */
void SysTick_Handler (void) { // SysTick interrupt Handler
    msTicks++;
    ++micros;
}

/* Function is used to send initialize UART0 */
void init_uart0() {
    /* Enable clock for PORTA and Set the PTA1 and PTA2 pin multiplexer to UART0_RX and UARTO_TX mode*/
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
    PORTA->PCR[1] &= ~(PORT_PCR_MUX_MASK);
    PORTA->PCR[1] |= PORT_PCR_MUX(2);
    PORTA->PCR[2] &= ~(PORT_PCR_MUX_MASK);
    PORTA->PCR[2] |= PORT_PCR_MUX(2);

    /* Enable clock for UART0 */
    SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;

    /* UART0 clock source: MCGFLLCLK (20Mhz - default value)  */
    SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);

    /* Disable UART0 transmitter and receiver */
    UART0->C2 &= ~(UART0_C2_RE_MASK | UART0_C2_TE_MASK);

    /* Configure baud rate: 115200 (baudrate = clk / OSR/ SBR)
       OSR default = 16 -> SBR = 20M / 16 / 115200 = 11
       Default: 8 bitdata, 1 bitstop , none parity format */
    UART0->BDL = 0xB;

    /* Enable UART0 transmitter and receiver */
    UART0->C2 |= (UART0_C2_RE_MASK | UART0_C2_TE_MASK);//Enable TE and RE
}

/* Function is used to send initialize UART1 */
void init_uart1() {
    /* Enable clock for PORTE and Set the PTE1 and PTE0 pin multiplexer to UART0_RX and UARTO_TX mode*/
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    PORTE->PCR[1] &= ~(PORT_PCR_MUX_MASK);
    PORTE->PCR[1] |= PORT_PCR_MUX(3);
    PORTE->PCR[0] &= ~(PORT_PCR_MUX_MASK);
    PORTE->PCR[0] |= PORT_PCR_MUX(3);

    /* Enable clock for UART1 */
    SIM->SCGC4 |= SIM_SCGC4_UART1_MASK;

    /* UART1 clock source: Bus clock (10Mhz (10485760) - default value)  */

    /* Disable UART1 transmitter and receiver */
    UART1->C2 &= ~(UART_C2_RE_MASK | UART_C2_TE_MASK);

    /* Configure baud rate: 9600 (baudrate = clk / SBR/ 16)
       SBR = 10M / 16 / 9600 = 68
       Default: 8 bitdata, 1 bitstop , none parity format */
    UART1->BDL = 0x45;

    /* Enable UART0 transmitter and receiver */
    UART1->C2 |= (UART_C2_RE_MASK | UART_C2_TE_MASK);//Enable TE and RE

}

/* Function is used to send initialize UART2 */
void init_uart2() {
    /* Enable clock for PORTE and Set the PTE23 and PTE22 pin multiplexer to UART2_RX and UART2_TX mode*/
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    PORTE->PCR[23] &= ~(PORT_PCR_MUX_MASK);
    PORTE->PCR[23] |= PORT_PCR_MUX(4);
    PORTE->PCR[22] &= ~(PORT_PCR_MUX_MASK);
    PORTE->PCR[22] |= PORT_PCR_MUX(4);

    /* Enable clock for UART1 */
    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;

    /* UART1 clock source: Bus clock (10Mhz (10485760) - default value)  */

    /* Disable UART1 transmitter and receiver */
    UART2->C2 &= ~(UART_C2_RE_MASK | UART_C2_TE_MASK);

    /* Configure baud rate: 9600 (baudrate = clk / SBR/ 16)
       SBR = 10M / 16 / 9600 = 68
       Default: 8 bitdata, 1 bitstop , none parity format */
    UART2->BDL = 0x44;

    /* Enable UART0 transmitter and receiver */
    UART2->C2 |= (UART_C2_RE_MASK | UART_C2_TE_MASK);//Enable TE and RE

}

/* Function is used to send character by UART0 */
void send_character_uart0(const char character) {
    while (!(UART0->S1 & UART_S1_TDRE_MASK));
    UART0->D = character;
}

/* Function is used to send string by UART0 */
void send_string_uart0(const char *p_character) {
    while ((*p_character) != '\0') {
            send_character_uart0(*p_character);
            p_character++;
    }
}

/* Function is used to send character by UART1 */
void send_character_uart1(const char character) {
    while (!(UART1->S1 & UART_S1_TDRE_MASK));
    UART1->D = character;
}

/* Function is used to send string by UART1 */
void send_string_uart1(const char *p_character) {
    while ((*p_character) != '\0') {
            send_character_uart1(*p_character);
            p_character++;
    }
}

/* Function is used to delay program*/
void delay(const uint32_t ms) {
    msTicks = 0;
    while (msTicks < ms);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/