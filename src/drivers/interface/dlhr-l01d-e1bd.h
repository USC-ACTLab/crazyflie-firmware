/**
 * @file dlhr-l01d-e1bd.h
 * @brief DLHR-L01D-E1BD differential pressure sensor driver
 * @author Will Wu, Anoop Kiran, ACT Lab @ Brown University
 * @date 2025-04-29
 */

#ifndef _DLHR_L01D_E1BD_H_
#define _DLHR_L01D_E1BD_H_

#include "i2cdev.h"

typedef uint8_t dlhr_sensor_status_t;

typedef struct {
    dlhr_sensor_status_t status;
    uint32_t pout;
    uint32_t tout;
} dlhr_sensor_data_t;

#define DLHR_L01D_E1BD_DATA_LENGTH 7 // bytes
#define SINGLE_MEASUREMENT_PERIOD_MS 10 // see datasheet, 18-bit resolutio requires 4.1ms max, 5 to be safe
#define INH2O_TO_PA 249.089 // 1 inH2O = 249.089 Pa

enum STATUS_BYTE {
    ALU_ERROR_BIT = 0,
    SENSOR_CONFIGURATION_BIT = 1,
    MEMORY_ERROR_BIT,
    MODE_0_BIT,
    MODE_1_BIT,
    BUSY_BIT,
    POWER_BIT,
    MSB_BIT,
    STATUS_BYTE_LEGNTH
};

enum POWER_STATUS {
    POWER_OFF = 0,
    POWER_ON
};

enum BUSY_STATUS {
    READY = 0,
    PROCESSING_COMMAND
};

#define MODE_0_NORMAL 0
#define MODE_1_NORMAL 0

enum MEMORY_ERROR_STATUS {
    NO_MEMORY_ERROR = 0,
    EEPROM_CHECKSUM_FAIL = 1
};

#define SENSOR_CONFIGURATION 0

enum ALU_ERROR_STATUS {
    NO_ERROR = 0,
    ALU_ERROR
};

enum SENSOR_MESUREMENT_COMMAND {
    START_SINGLE = 0xAA,
    START_AVERAGE_2 = 0xAC,
    START_AVERAGE_4 = 0xAD,
    START_AVERAGE_8 = 0xAE,
    START_AVERAGE_16 = 0xAF
};

/**
 * @brief convert sensor 24-bit digital output to pressure in inH2O
 * @param[in] pout 24-bit digital output from sensor
 * @return pressure in inH2O
 */
double poutToPressureinH2O(const uint32_t pout);

/**
 * @brief convert sensor 24-bit digital output to temperature in C
 * @param[in] tout 24-bit digital output from sensor
 * @return temperature in C
 */
double toutToTemperatureC(const uint32_t tout);

/**
 * @brief Initializes the DLHR-L01D-E1BD pressure sensor
 * @param I2Cx I2C device to use
 * @return true if initialization was successful, false otherwise
 */
bool dlhrL01dE1bdInit(I2C_Dev *i2cPort);

/**
 * @brief Reads the status byte from the sensor as test
 * @return true if the sensor is ready, false otherwise
 */
bool dlhrL01dE1bdTestConnection(void);

/**
 * @brief read sensor status
 * @return sensor status byte
 */
dlhr_sensor_status_t dlhrL01dE1bdGetStatus(void);

/**
 * @brief check sensor status, whether it is ready to read data
 * @param status sensor status byte
 * @return true if the sensor is ready, false if busy
 *
 */
bool dlhrL01dE1bdIsSensorReady(void);

/**
 * @brief check sensor status, whether it has erred
 * @param status sensor status byte
 * @return true if the sensor has erred or it's not on, false otherwise
 */
bool dlhrL01dE1bdCheckError(const dlhr_sensor_status_t status);

/**
 * @brief start single read measurement
 * @return true on success, false otherwise
 */
bool dlhrL01dE1bdStartSingleMeasurement(void);

/**
 * @brief read sensor data
 * @param[in] buffer buffer to store the sensor data
*/
bool dlhrL01dE1bdReadSensorData(dlhr_sensor_data_t *data);

#endif // _DLHR_L01D_E1BD_H_