#ifndef _GPIO_H_
#define _GPIO_H_

/**
 * @brief Initialize GPIO
 * 
 */
void initGpio();

/**
 * @brief Change state
 * 
 * @param [in] state is state to chance
 */
void change_state(const uint8_t state);

#endif /* _GPIO_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/