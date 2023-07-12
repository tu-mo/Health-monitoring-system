/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "MKL46Z4.h"
#include "i2c.h"
#include "sensor.h"
#include "uart.h"

/*******************************************************************************
* Definitions
*******************************************************************************/
#define MEAN_FILTER_SIZE            15
#define PULSE_MIN_THRESHOLD         100
#define PULSE_MAX_THRESHOLD         2000
#define PULSE_GO_DOWN_THRESHOLD     1
#define PULSE_BPM_SAMPLE_SIZE       10

#define MAGIC_ACCEPTABLE_INTENSITY_DIFF         65000
#define RED_LED_CURRENT_ADJUSTMENT_NS           1000000

#define RESET_SPO2_EVERY_N_PULSES     100
#define COMBINE(sentence_type, term_number) (((unsigned)(sentence_type) << 5) | term_number)
enum pulse_state_machine {
PULSE_IDLE,
PULSE_TRACE_UP,
PULSE_TRACE_DOWN
};

static uint32_t IR = 0;
static uint32_t RED = 0;
static uint32_t IRcw = 0;
static uint32_t REDcw = 0;
static uint8_t redLEDCurrent = 5;
static uint8_t data_read[4] = {0};
static uint8_t data_write[1] = {0};

static uint32_t irf[2] = {0, 0};
static uint32_t redf[2] = {0, 0};
static uint32_t irACValueSqSum = 0;
static uint32_t redACValueSqSum = 0;
static uint16_t samplesRecorded = 0;
static uint16_t pulsesDetected = 0;
volatile uint32_t currentSpO2Value = 0;
volatile uint32_t currentPulseRateValue = 0;

static int32_t msum = 0;
static int32_t mvalues[MEAN_FILTER_SIZE] = {0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static int32_t mindex = 0;
static int32_t mcount = 0;

static uint8_t currentPulseDetectorState = PULSE_IDLE;
static uint32_t lastBeatThreshold = 0;
static uint32_t currentBPM;
static uint32_t valuesBPM[PULSE_BPM_SAMPLE_SIZE] = {0};
static uint32_t valuesBPMSum = 0;
static uint8_t valuesBPMCount = 0;
static uint8_t bpmIndex = 0;
static uint32_t lastREDLedCurrentCheck = 0;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/**
 * @brief Write to memory address of slave 
 * 
 * @param [in] memory_adress is address of memory want to write
 */
static void max30100_write_one_byte(const uint8_t memory_adress);

/**
 * @brief Read from memory address of slave 
 * 
 * @param [in] memory_adress is address of memory want to read
 */
static void max30100_read_one_byte(const uint8_t memory_adress);

/**
 * @brief DC remove
 * 
 * @param [in] value is old value
 * @param [out] cw is value to update
 * @return uint32_t is value after remove 
 */
static uint32_t DCRemove(const uint32_t value, uint32_t *const cw);

/**
 * @brief Filter
 * 
 * @param [in] value is old value
 * @param [out] filter_data is filter data
 * @return uint32_t is new value
 */
static uint32_t LowPassButterworthFilter(const uint32_t value, uint32_t *const filter_data);

/**
 * @brief mean filter
 * 
 * @param [in] m is old value
 * @return uint32_t is new value
 */
static uint32_t mean_diff(const uint32_t m);

/**
 * @brief detect pulse
 * 
 * @param sensor_value is value of sensor 
 * @return true if detect pulse
 * @return false if not
 */
static bool detectPulse(const uint32_t sensor_value);

/**
 * @brief Balance intesities
 * 
 */
static void BalanceIntesities();

/**
 * @brief Read fifo data
 * 
 */
static void readFifoData();

/**
 * @brief Convert character to hex
 * 
 * @param [in] a is input character
 * @return uint8_t is output
 */
static uint8_t from_hex(const char a);

/**
 * @brief Convert string to value
 * 
 * @param [in] str is input string
 * @return uint32_t is output value
 */
static uint32_t gpsatol(const char *const str);

/**
 * @brief Check character is digit
 * 
 * @param [in] c is input character
 * @return true if input is digit
 * @return false if not
 */
static bool gpsisdigit(const char c);

/**
 * @brief Parse gps value
 * 
 * @param [inout] g is gps value
 * @return uint32_t is output
 */
static uint32_t parse_degrees(gps *const g);

/**
 * @brief  Check term is complete
 * 
 * @param [inout] g is gps value
 * @return true if term is complete
 * @return false if not
 */
static bool term_complete(gps *const g);

/*******************************************************************************
 * Codes
 ******************************************************************************/
/* Function is used to write data to slave */
static void max30100_write_one_byte(const uint8_t memory_adress) {
    delay(100);
    i2c_write(MAX30100_ADDRESS, memory_adress, data_write, 1);
}

/* Function is used to read data from slave */
static void max30100_read_one_byte(const uint8_t memory_adress) {
    delay(100);
    i2c_read(MAX30100_ADDRESS, memory_adress, data_read, 4);
}

/* Function is used to initialize max30100 */
bool init_max30100() {
    bool res = true;

    /* Check if EXPECTED_PART_ID(0x11) is correct. */
    max30100_read_one_byte(MAX30100_PART_ID);
    if (data_read[0] != EXPECTED_PART_ID) {
            res = false;
    } else {
        /* Set to address 0x06(MAX30100_MODE_CONFIG) value 0x03(MAX30100_MODE_SPO2_HR) */
        data_write[0] = MAX30100_MODE_SPO2_HR;
        max30100_write_one_byte(MAX30100_MODE_CONFIG);

        /* Set to address 0x07(MAX30100_SPO2_CONFIG) value  0b00000111*/
        data_write[0] = (MAX30100_SAMPRATE_100HZ << 2) | MAX30100_SPC_PW_1600US_16BITS;
        max30100_write_one_byte(MAX30100_SPO2_CONFIG);

        /* Set to address 0x09(MAX30100_LED_CONFIG) value  0b10001111*/
        data_write[0] = (MAX30100_LED_CURR_27_1MA << 4) | MAX30100_LED_CURR_20_8MA;
        max30100_write_one_byte(MAX30100_LED_CONFIG);
    }
    return res;
}

/* Function is used to read fifo data */
static void readFifoData() {
    char mn[50];
    max30100_read_one_byte(MAX30100_FIFO_DATA);
    IR = data_read[0] << 8 | data_read[1];
    RED = data_read[2] << 8 | data_read[3];
}

/* Function is used to read data of max30100 */
void read_max30100() {
    readFifoData();

    uint32_t IRac = DCRemove(IR,&IRcw);
    uint32_t REDac = DCRemove(RED,&REDcw);

    IRac = mean_diff(IRac);
    REDac = mean_diff(REDac);

    IRac = LowPassButterworthFilter(IRac,irf);
    REDac = LowPassButterworthFilter(REDac,redf);

    irACValueSqSum += IRac * IRac;
    redACValueSqSum += REDac * REDac;
    samplesRecorded++;

    samplesRecorded++;
    detectPulse(IRac);
    if (data_read[0] > 0 && data_read[2] > 0) {
            pulsesDetected++;
            float red_log_rms = log( sqrt(redACValueSqSum/samplesRecorded) );
            float ir_log_rms = log( sqrt(irACValueSqSum/samplesRecorded) );
            float ratioRMS = 0.0f;
            if(red_log_rms != 0.0f && ir_log_rms != 0.0f) {
                    ratioRMS = red_log_rms / ir_log_rms;
            }
            currentSpO2Value = 110.0f - 14.0f * ratioRMS;
            currentPulseRateValue = data_read[0];
            if(pulsesDetected % RESET_SPO2_EVERY_N_PULSES == 0) {
                    irACValueSqSum = 0;
                    redACValueSqSum = 0;
                    samplesRecorded = 0;
            }
    } else {
            currentSpO2Value = 0;
            currentPulseRateValue = 0;
    }
    BalanceIntesities();
}

/* Function is used to remove DC */
static uint32_t DCRemove(const uint32_t value, uint32_t *const cw) {
    int32_t oldcw = *cw;
    *cw = value + 0.94 * *cw;
    return *cw - oldcw;
}

/* Function is used to mean value */
static uint32_t mean_diff(const uint32_t m) {
    uint32_t avg = 0;

    msum -= mvalues[mindex];
    mvalues[mindex] = m;
    msum += mvalues[mindex];

    mindex++;
    mindex = mindex % MEAN_FILTER_SIZE;

    if(mcount < MEAN_FILTER_SIZE) {
            mcount++;
    }

    avg = msum / mcount;
    return avg - m;
}

/* Function is used to filt value */
static uint32_t LowPassButterworthFilter(const uint32_t value, uint32_t *const filter_data) {
    filter_data[0] = filter_data[1];

    /* Fs = 100Hz and Fc = 10Hz */
    filter_data[1] = (2.452372752527856026e-1 * value) + (0.50952544949442879485 * filter_data[0]);

    return filter_data[0] + filter_data[1];
}

/* Function is used to detect pulse */
static bool detectPulse(const uint32_t sensor_value) {
    static uint32_t prev_sensor_value = 0;
    static uint8_t values_went_down = 0;
    static uint32_t currentBeat = 0;
    static uint32_t lastBeat = 0;

    if(sensor_value > PULSE_MAX_THRESHOLD) {
            currentPulseDetectorState = PULSE_IDLE;
            prev_sensor_value = 0;
            lastBeat = 0;
            currentBeat = 0;
            values_went_down = 0;
            lastBeatThreshold = 0;
            return false;
    }

    switch(currentPulseDetectorState) {
            case PULSE_IDLE:
                if(sensor_value >= PULSE_MIN_THRESHOLD) {
                        currentPulseDetectorState = PULSE_TRACE_UP;
                        values_went_down = 0;
                }
                break;

            case PULSE_TRACE_UP:
                if(sensor_value > prev_sensor_value) {
                        currentBeat = micros;
                        lastBeatThreshold = sensor_value;
                } else {
                        uint32_t beatDuration = currentBeat - lastBeat;
                        lastBeat = currentBeat;

                        uint32_t rawBPM = 0;
                        if(beatDuration > 0)
                                rawBPM = 60000000 / beatDuration;
                        valuesBPM[bpmIndex] = rawBPM;
                        valuesBPMSum = 0;
                        for(int i=0; i<PULSE_BPM_SAMPLE_SIZE; i++) {
                                valuesBPMSum += valuesBPM[i];
                        }
                        bpmIndex++;
                        bpmIndex = bpmIndex % PULSE_BPM_SAMPLE_SIZE;

                        if(valuesBPMCount < PULSE_BPM_SAMPLE_SIZE)
                                valuesBPMCount++;

                        currentBPM = valuesBPMSum / valuesBPMCount;
                        currentPulseDetectorState = PULSE_TRACE_DOWN;
                        return true;
                }
                break;
            case PULSE_TRACE_DOWN:
                if(sensor_value < prev_sensor_value) {
                        values_went_down++;
                }


                if(sensor_value < PULSE_MIN_THRESHOLD) {
                        currentPulseDetectorState = PULSE_IDLE;
                }
                break;
    }

    prev_sensor_value = sensor_value;
    return false;
}

/* Function is used to balance intesities */
static void BalanceIntesities() {
    if( micros - lastREDLedCurrentCheck >= RED_LED_CURRENT_ADJUSTMENT_NS) {
            if( IRcw - REDcw > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent < MAX30100_LED_CURR_50MA) {
                    redLEDCurrent++;
                    data_write[0] = (redLEDCurrent << 4) | MAX30100_LED_CURR_20_8MA;
                    max30100_write_one_byte(MAX30100_SPO2_CONFIG);
            } else if(REDcw - IRcw > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent > 0) {
                    redLEDCurrent--;
                    data_write[0] = (redLEDCurrent << 4) | MAX30100_LED_CURR_20_8MA;
                    max30100_write_one_byte(MAX30100_SPO2_CONFIG);
            }

            lastREDLedCurrentCheck = micros;
    }
}

/* Function is used to read adc value */
uint16_t adc16_simple_read(const uint8_t adc_channel) {
    ADC0->SC1[0] = ADC_SC1_DIFF(0) | ADC_SC1_ADCH(adc_channel); // start conversionz
    while(!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {};                       // wait for conversion complete

    return ADC0->R[0];
}

/* Function is used to convert adc value to temperature */
double convert_temp(const double adc_val) {
    double voltage = adc_val * 3.3 / 4096;

    return voltage * 100;
}

/* Function is used to get temperature of enviroment */
double get_temp_env() {
  double adc_result = 0;
  
  for (int i = 0; i < 20; i++) {
    adc_result = adc16_simple_read(1);
  }
  
  return convert_temp(adc_result);
}

/* Function is used to convert character to hex */
static uint8_t from_hex(const char a) {
    if (a >= 'A' && a <= 'F')
            return a - 'A' + 10;
    else if (a >= 'a' && a <= 'f')
            return a - 'a' + 10;
    else
            return a - '0';
}

/* Function is used to conver string to value */
static uint32_t gpsatol(const char *str) {
    long ret = 0;
    while (gpsisdigit(*str))
            ret = 10 * ret + *str++ - '0';
    return ret;
}

/* Function is used to check character is digit */
static bool gpsisdigit(const char c) {
    return c >= '0' && c <= '9';
}

/* Function is used to parse gps value */
static uint32_t parse_degrees(gps *const g) {
    char *p;
    uint32_t left_of_decimal = gpsatol(g->term);
    uint32_t hundred1000ths_of_minute = (left_of_decimal % 100UL) * 100000UL;
    for (p=g->term; gpsisdigit(*p); ++p);
    if (*p == '.') {
            unsigned long mult = 10000;
            while (gpsisdigit(*++p)) {
                    hundred1000ths_of_minute += mult * (*p - '0');
                    mult /= 10;
            }
    }
    return (left_of_decimal / 100) * 1000000 + (hundred1000ths_of_minute + 3) / 6;
}

/* Function is used to check term is complete */
static bool term_complete(gps *const g) {
    if (g->is_checksum_term) {
            uint8_t checksum = 16 * from_hex(g->term[0]) + from_hex(g->term[1]);
            if (checksum == g->parity) {
                if (g->gps_data_good) {
                        switch(g->sentence_type) {
                                case GPS_SENTENCE_GPRMC:
                                        g->latitude  = g->new_latitude;
                                        g->longitude = g->new_longitude;
                                        break;
                                case GPS_SENTENCE_GPGGA:
                                        g->latitude  = g->new_latitude;
                                        g->longitude = g->new_longitude;
                                        break;
                        }
                        return true;
                }
            }
            return false;
    }

    if (g->term_number == 0) {
            if (strcmp(g->term, "GPRMC") == 0)
                    g->sentence_type = GPS_SENTENCE_GPRMC;
            else if (strcmp(g->term, "GPGGA") == 0)
                    g->sentence_type = GPS_SENTENCE_GPGGA;
            else
                    g->sentence_type = GPS_SENTENCE_OTHER;
            return false;
    }

    if (g->sentence_type != GPS_SENTENCE_OTHER && g->term[0])
            switch(COMBINE(g->sentence_type, g->term_number)) {
                    case COMBINE(GPS_SENTENCE_GPRMC, 2): /* GPRMC validity */ 
                            g->gps_data_good = g->term[0] == 'A';
                            break;
                    case COMBINE(GPS_SENTENCE_GPRMC, 3): /* Latitude */ 
                    case COMBINE(GPS_SENTENCE_GPGGA, 2):
                            g->new_latitude = parse_degrees(g);
                            break;
                    case COMBINE(GPS_SENTENCE_GPRMC, 4): /* N/S */ 
                    case COMBINE(GPS_SENTENCE_GPGGA, 3):
                            if (g->term[0] == 'S')
                                    g->new_latitude = -g->new_latitude;
                            break;
                    case COMBINE(GPS_SENTENCE_GPRMC, 5): /* Longitude */
                    case COMBINE(GPS_SENTENCE_GPGGA, 4):
                            g->new_longitude = parse_degrees(g);
                            break;
                    case COMBINE(GPS_SENTENCE_GPRMC, 6): /* E/W */ 
                    case COMBINE(GPS_SENTENCE_GPGGA, 5):
                            if (g->term[0] == 'W')
                                    g->new_longitude = -g->new_longitude;
                            break;
                    case COMBINE(GPS_SENTENCE_GPGGA, 6): /* Fix data (GPGGA) */ 
                            g->gps_data_good = g->term[0] > '0';
                            break;
            }

    return false;
}

/* Function is used to encode character */
bool encode(gps *const g, const char c) {
    bool valid_sentence = false;

    switch(c) {
            case ',': /* term terminators */ 
                    g->parity ^= c;
            case '\r':
            case '\n':
            case '*':
                    if (g->term_offset < sizeof(g->term)) {
                            g->term[g->term_offset] = 0;
                            valid_sentence = term_complete(g);
                    }
                    ++g->term_number;
                    g->term_offset = 0;
                    g->is_checksum_term = c == '*';
                    return valid_sentence;

            case '$': /* sentence begin */
                    g->term_number = g->term_offset = 0;
                    g->parity = 0;
                    g->sentence_type = GPS_SENTENCE_OTHER;
                    g->is_checksum_term = false;
                    return valid_sentence;
    }

    /* ordinary characters */ 
    if (g->term_offset < sizeof(g->term) - 1)
            g->term[g->term_offset++] = c;
    if (!g->is_checksum_term)
            g->parity ^= c;

    return valid_sentence;
}


void get_position(const gps *const g, double *const latitude, double *const longitude) {
    *latitude = g->latitude / 1000000.0;
    *longitude = g->longitude / 1000000.0;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/




