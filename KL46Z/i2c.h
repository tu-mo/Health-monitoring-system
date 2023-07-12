#ifndef _I2C_H_
#define _I2C_H_ 

/*******************************************************************************
 * API
 ******************************************************************************/
/**
 * @brief Initialize I2C
 * 
 */
void i2c_init(); 

/**
 * @brief Read data from slave
 * 
 * @param [in] slave_adress is address of slave 
 * @param [in] memory_adress is address of slave's memory want to read
 * @param [inout] databuffer is data of slave want to read
 * @param [in] datasize is size of data want to read
 */
void i2c_read(const uint8_t slave_adress, const uint8_t memory_adress, uint8_t * databuffer, const uint8_t datasize);

/**
 * @brief Write data to slave
 * 
 * @param [in] slaveadress is address of slave 
 * @param [in] memoryadress is address of slave's memory want to read
 * @param [inout] databuffer is data want to write to slave
 * @param [in] datasize is size of data want to write
 */
void i2c_write(const uint8_t slaveadress, const uint8_t memoryadress, uint8_t *databuffer, const uint8_t datasize);

#endif /* _ADC_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/