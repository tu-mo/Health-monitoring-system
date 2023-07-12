/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MAX30100_ADDRESS        0x57
#define MAX30100_FIFO_DATA      0x05
#define MAX30100_MODE_CONFIG    0x06
#define MAX30100_SPO2_CONFIG    0x07
#define MAX30100_LED_CONFIG     0x09

#define MAX30100_PART_ID        0xFF
#define EXPECTED_PART_ID        0x11
#define MAX30100_SPC_SPO2_HI_RES_EN     (1 << 6)

#define MAX30100_REG_FIFO_WRITE_POINTER         0x02
#define MAX30100_REG_FIFO_READ_POINTER          0x04
#define MAX30100_REG_FIFO_DATA                  0x05

enum Mode {
    MAX30100_MODE_HRONLY    = 0x02,
    MAX30100_MODE_SPO2_HR   = 0x03
};

enum LEDPulseWidth {
    MAX30100_SPC_PW_200US_13BITS    = 0x00,
    MAX30100_SPC_PW_400US_14BITS    = 0x01,
    MAX30100_SPC_PW_800US_15BITS    = 0x02,
    MAX30100_SPC_PW_1600US_16BITS   = 0x03
};

enum SamplingRate {
    MAX30100_SAMPRATE_50HZ      = 0x00,
    MAX30100_SAMPRATE_100HZ     = 0x01,
    MAX30100_SAMPRATE_167HZ     = 0x02,
    MAX30100_SAMPRATE_200HZ     = 0x03,
    MAX30100_SAMPRATE_400HZ     = 0x04,
    MAX30100_SAMPRATE_600HZ     = 0x05,
    MAX30100_SAMPRATE_800HZ     = 0x06,
    MAX30100_SAMPRATE_1000HZ    = 0x07
};

enum LEDCurrent {
    MAX30100_LED_CURR_0MA      = 0x00,
    MAX30100_LED_CURR_4_4MA    = 0x01,
    MAX30100_LED_CURR_7_6MA    = 0x02,
    MAX30100_LED_CURR_11MA     = 0x03,
    MAX30100_LED_CURR_14_2MA   = 0x04,
    MAX30100_LED_CURR_17_4MA   = 0x05,
    MAX30100_LED_CURR_20_8MA   = 0x06,
    MAX30100_LED_CURR_24MA     = 0x07,
    MAX30100_LED_CURR_27_1MA   = 0x08,
    MAX30100_LED_CURR_30_6MA   = 0x09,
    MAX30100_LED_CURR_33_8MA   = 0x0a,
    MAX30100_LED_CURR_37MA     = 0x0b,
    MAX30100_LED_CURR_40_2MA   = 0x0c,
    MAX30100_LED_CURR_43_6MA   = 0x0d,
    MAX30100_LED_CURR_46_8MA   = 0x0e,
    MAX30100_LED_CURR_50MA     = 0x0f
};
typedef struct _gps {
    uint32_t new_latitude;
    uint32_t latitude;
    uint32_t new_longitude;
    uint32_t longitude;
    uint8_t parity;
    bool is_checksum_term;
    uint8_t term[15];
    uint8_t sentence_type;
    uint8_t term_number;
    uint8_t term_offset;
    bool gps_data_good;
} gps;

enum GPS_TYPE{
    GPS_SENTENCE_GPGGA,
    GPS_SENTENCE_GPRMC,
    GPS_SENTENCE_OTHER
};

extern volatile uint32_t micros;
/*******************************************************************************
 * API
 ******************************************************************************/
/**
 * @brief Initialize Max30100
 * 
 * @return true is initialize success
 * @return false is initialize fail
 */
bool init_max30100();

/**
 * @brief Read Max30100 data
 * 
 */
void read_max30100();

/**
 * @brief Read data by adc
 * 
 * @param [in] adc_channel is channel to read data 
 * @return uint16_t is data is read
 */
uint16_t adc16_simple_read(const uint8_t adc_channel);

/**
 * @brief Convert adc value to temperature
 * 
 * @param [in] adc_val is adc value to convert 
 * @return double 
 */
double convert_temp(const double adc_val);

/**
 * @brief Get temperature of enviroment
 * 
 * @return double is temperature of enviroment
 */
double get_temp_env();
/**
 * @brief Encode gps data
 * 
 * @param [inout] g is gps data
 * @param [in] c is character to encode 
 * @return true if encode success
 * @return false if encode fail
 */
bool encode(gps *const g, const char c);

/**
 * @brief Get the position object
 * 
 * @param [in] g is gps data 
 * @param [out] latitude is latitude 
 * @param [out] longitude is longtitude
 */
void get_position(const gps *const g, double *const latitude, double *const longitude);

/*******************************************************************************
 * EOF
 ******************************************************************************/