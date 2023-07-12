/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "MKL46Z4.h"
#include "i2c.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define READ_SLAVE_MASK 1
#define WRITE_SLAVE_MASK 0

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/**
 * @brief Wait tranfer finish
 * 
 */
static void I2C_WaitTransfer(void);

/**
 * @brief Wait ACK/NACK bit
 * 
 */
static void I2C_WaitBitACK(void);

/**
 * @brief Start transmission
 * 
 * @param [in] slave_adress is address of slave 
 * @param [in] memory_adress is address of slave's memory want to read/write
 */
static void i2c_start_transmit(const uint8_t slave_adress, const uint8_t memory_adress);

/*******************************************************************************
 * Codes
 ******************************************************************************/
/* Function is used to wait tranfer process finish */
static void I2C_WaitTransfer(void) {
    /* Wait IICIF bit is set */
    while ((I2C0->S & I2C_S_IICIF_MASK) == 0);

    /* Clear IICIF bit */
    I2C0->S |= I2C_S_IICIF_MASK;
}

/* Function is used to wait ACK/NACK bit */
static void I2C_WaitBitACK(void) {
    /* Wait receive bit ACK */
    while (I2C0->S & I2C_S_RXAK_MASK); 
}

/* Function is used to start transmission */
static void i2c_start_transmit(uint8_t slave_adress, uint8_t memory_adress) {
    /* Transmit mode: transmit
       Master mode: master. When the MST bit is changed from a 0 to a 1,
       a START signal is generated on the bus and master mode is selected. */
    I2C0->C1 |= I2C_C1_TX_MASK;
    I2C0->C1 |= I2C_C1_MST_MASK;

    /* Transmit slave address and register address */
    I2C0->D = (slave_adress << 1) | WRITE_SLAVE_MASK;
    I2C_WaitTransfer();
    I2C_WaitBitACK();

    /* Register to write adress */
    I2C0->D = memory_adress;
    I2C_WaitTransfer();
    I2C_WaitBitACK();
}

/* Function is used to read data from slave*/
void i2c_read(const uint8_t slave_adress, const uint8_t memory_adress, uint8_t * databuffer, const uint8_t datasize) {
    uint8_t i = 0;

    /* Start transmission */
    i2c_start_transmit(slave_adress, memory_adress);

    /* Repeat start condition */
    I2C0->C1 |= I2C_C1_RSTA_MASK;
    I2C0->D = ((slave_adress << 1) | READ_SLAVE_MASK);
    I2C_WaitTransfer();

    I2C0->C1 &= ~I2C_C1_TX_MASK;

    /* Last address cycle before switching to receive mode */
    (void) I2C0->D;

    for (i = 0; i < datasize; i++, databuffer++) {
            I2C_WaitTransfer();

            if (i == datasize - 1) { /* If the last byte to be read */
                    I2C0->C1 |= I2C_C1_TXAK_MASK;
                    I2C0->C1 &= ~I2C_C1_MST_MASK;
            }
            else if (i == datasize - 2) { /* If the second last byte to be read */
                    I2C0->C1 |= I2C_C1_TXAK_MASK;
            } else {
                /* Do nothing */
            }

            *databuffer = I2C0->D;
    }

    /* Restore default state */
    I2C0->C1 &= ~I2C_C1_RSTA_MASK;
    I2C0->C1 &= ~I2C_C1_TXAK_MASK;
}

/* Function is used to write data to slave */
void i2c_write(const uint8_t slaveadress, const uint8_t memoryadress, uint8_t * databuffer, const uint8_t datasize) { // databuffer is an array
    uint8_t i = 0;

    /* Start transmission */
    i2c_start_transmit(slaveadress, memoryadress);

    for (i = 0; i < datasize; i++, databuffer++) {
            I2C0->D = *databuffer;
            I2C_WaitTransfer();
            I2C_WaitBitACK();
    }

    /* STOP signal generated */
    I2C0->C1 &= ~I2C_C1_MST_MASK; 
}

/* Function is used to initialize I2C */
void i2c_init() {
    /* Enable clock for PORTC and Set the PTC8 and PTC9 pin multiplexer to I2C0_SCL and I2C0_SDA mode*/
    SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;  
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; 
    PORTC->PCR[8] |= PORT_PCR_MUX(2);  
    PORTC->PCR[9] |= PORT_PCR_MUX(2);

    /* I2C0 clock source: Bus clock (10Mhz (10485760) - default value)
       I2C0 baud rate = bus speed / (mul * scl)
       Config baudrate = 100Mhz -> choose mul = 2, scl = 48 (I2C_F = 0x04D) */
    I2C0->F |= I2C_F_MULT(1) | I2C_F_ICR(13);

    /* Enable I2C */
    I2C0->C1 |= I2C_C1_IICEN_MASK;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/