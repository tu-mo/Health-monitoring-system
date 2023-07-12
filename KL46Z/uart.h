#ifndef _UART_H_
#define _UART_H_

/*******************************************************************************
 * API
 ******************************************************************************/
/**
 * @brief Initialize UART0
 * 
 */
void init_uart0();

/**
 * @brief Send character by UART0
 * 
 * @param [in] mcharacter is character to send
 */
void send_character_uart0(const char character);

/**
 * @brief Send string by UART0
 * 
 * @param [in] p_character is string to send
 */
void send_string_uart0(const char * p_character);

/**
 * @brief Send character bu UART1
 * 
 * @param [in] character is character to send
 */
void send_character_uart1(const char character);

/**
 * @brief Send string by UART1
 * 
 * @param [in] p_character is string to send
 */
void send_string_uart1(const char * p_character);

/**
 * @brief Initialize UART1
 * 
 */
void init_uart1() ;

/**
 * @brief Initialize UART2
 * 
 */
void init_uart2();

/**
 * @brief Delay function
 * 
 * @param [in] ms is delay time
 */
void delay(const uint32_t ms);

/**
 * @brief Initialize systick interrupt
 * 
 */
void init_SysTick_interrupt();

#endif /* _UART_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/