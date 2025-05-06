/**
 * @file dlhr-l01d-e1bd.c
 * @brief DLHR-L01D-E1BD differential pressure sensor driver implementation
 * @author Will Wu, Anoop Kiran, ACT Lab @ Brown University
 * @date 2025-04-30
 */

#include "FreeRTOS.h"
#include "cf_math.h"
#include "debug.h"
#include "dlhr-l01d-e1bd.h"
#include "task.h"

static const uint32_t OS_DIG_DIFFERENTIAL = 0x800000; // 0.5 * 2^24
static const double FSS_DIFFERENTIAL = 4.0; // 2 * FSS(INH2O) = 4.0 inH2O, FSS = 1 - (-1) INH2O

static bool isInit = false;

static const uint8_t devAddr = DOWN_WASH_DECK_I2C_ADDR;
static I2C_Dev *I2Cx;

double poutToPressureinH2O(const uint32_t pout) {
    return 1.25 * ((double) pout - OS_DIG_DIFFERENTIAL) / (double)(1 << 24) * FSS_DIFFERENTIAL;
}

double toutToTemperatureC(const uint32_t tout) {
    return ((double) tout * 125.0) / (double)(1 << 24) - 40.0;
}

bool dlhrL01dE1bdInit(I2C_Dev *i2cPort) {
    if (i2cPort == NULL) {
        return false;
    }
    if (isInit) {
        return true;
    }

    I2Cx = i2cPort;
    isInit = true;
    return true;
}

bool dlhrL01dE1bdTestConnection(void) {
    if (!isInit) {
        DEBUG_PRINT("Sensor not initialized.\n");
        return false;
    }

    const dlhr_sensor_status_t status = dlhrL01dE1bdGetStatus();
    return !dlhrL01dE1bdCheckError(status);
}

dlhr_sensor_status_t dlhrL01dE1bdGetStatus(void) {
    dlhr_sensor_status_t status; // Initialize status to 0, note this is power off
    const bool read_result = i2cdevRead(I2Cx, devAddr, 1, &status);
    if (!read_result) {
        DEBUG_PRINT("Failed to read sensor status.\n");
        return 0x00; // Return 0 if read fails
    }
    return status;
}

bool dlhrL01dE1bdIsSensorReady(void) {
    const dlhr_sensor_status_t status = dlhrL01dE1bdGetStatus();
    const bool status_bit = (status >> BUSY_BIT) & 1;
    return (status_bit == READY);
}

bool dlhrL01dE1bdCheckError(const dlhr_sensor_status_t status) {
    const uint8_t error_mask = (1 << ALU_ERROR_BIT) | (1 << MEMORY_ERROR_BIT) | (1 << MODE_0_BIT) | (1 << MODE_1_BIT);
    return ((status & error_mask) != 0) || ((status >> POWER_BIT & 1) != POWER_ON);
}

bool dlhrL01dE1bdStartSingleMeasurement(void) {
    if (!isInit) {
        DEBUG_PRINT("Sensor not initialized.\n");
        return false;
    }

    const uint8_t meas_command = START_SINGLE;
    const bool write_result = i2cdevWrite(I2Cx, devAddr, 1, &meas_command);
    return write_result;
}

bool dlhrL01dE1bdReadSensorData(dlhr_sensor_data_t *data) {
    if (!isInit) {
        DEBUG_PRINT("Sensor not initialized.\n");
        return false;
    }
    if (data == NULL) {
        DEBUG_PRINT("Data pointer is NULL.\n");
        return false;
    }

    uint8_t data_buffer[DLHR_L01D_E1BD_DATA_LENGTH];
    const bool read_result = i2cdevRead(I2Cx, devAddr, DLHR_L01D_E1BD_DATA_LENGTH, data_buffer);
    if (!read_result) {
        DEBUG_PRINT("Failed to read sensor data.\n");
        return false;
    }

    data->status = data_buffer[0];
    data->pout = ((uint32_t)data_buffer[1] << 16) | ((uint32_t)data_buffer[2] << 8) | (uint32_t)data_buffer[3];
    data->tout = ((uint32_t)data_buffer[4] << 16) | ((uint32_t)data_buffer[5] << 8) | (uint32_t)data_buffer[6];

    return true;
}