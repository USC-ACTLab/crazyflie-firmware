/**
 * @file downwash.c
 * @brief Downwash deck driver implementation
 * @author Will Wu, ACT Lab @ Brown University
 * @date 2025-04-29
 */

#define DEBUG_MODULE "ZR2"

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "static_mem.h"

#include "downwash.h"
#include "dlhr-l01d-e1bd.h"
#include "i2cdev.h"
#include "cf_math.h"

static bool isInit = false;
static dlhr_sensor_status_t sensor_status = 0;
// static dlhr_sensor_data_t sensor_data;
static float pressure_log;
static float temperature_log;
static uint32_t pout_log;
static uint32_t tout_log;
static bool busy_status = false;

void downwashDeckInit(DeckInfo* info) {
    if (isInit) {
        return;
    }

    const bool sensor_init_res = dlhrL01dE1bdInit(I2C1_DEV);
    if (!sensor_init_res) {
        DEBUG_PRINT("Failed to initialize the DLHR-L01D-E1BD sensor.\n");
        return;
    }

    xTaskCreate(downwashDeckTask, DOWNWASH_DECK_TASK_NAME, DOWNWASH_DECK_TASK_STACKSIZE, NULL, DOWNWASH_DECK_TASK_PRI, NULL);

    // Set the deck as initialized
    isInit = true;

    DEBUG_PRINT("Downwash deck initialized.\n");
}

bool downwashDeckTest(void) {
    if (!isInit) {
        return false;
    }
    // sensor_status = dlhrL01dE1bdGetStatus();
    // DEBUG_PRINT("Sensor status during check: %d\n", sensor_status);
    // return !dlhrL01dE1bdCheckError(sensor_status);
    return true;
}

void downwashDeckTask(void* arg) {
    TickType_t lastWakeTime;

    systemWaitStart();
    lastWakeTime = xTaskGetTickCount();

    while(1) {
        vTaskDelayUntil(&lastWakeTime, M2T(10));
        // DEBUG_PRINT("Sensor status: %d\n", dlhrL01dE1bdGetStatus());
        const bool start_result = dlhrL01dE1bdStartSingleMeasurement();
        if(!start_result) {
            DEBUG_PRINT("Failed to start single measurement.\n");
            continue;
        }
        bool valid_data_available = dlhrL01dE1bdIsSensorReady();
        while (!valid_data_available) {
            valid_data_available = dlhrL01dE1bdIsSensorReady();
        };
        // DEBUG_PRINT("Sensor is ready.\n");
        dlhr_sensor_data_t sensor_data;
        const bool received_data = dlhrL01dE1bdReadSensorData(&sensor_data);
        if (!received_data) {
            DEBUG_PRINT("Failed to read sensor data.\n");
            continue;
        }
        // DEBUG_PRINT("Immediate read: %d\n", sensor_data.status);
        tout_log = sensor_data.tout;
        pout_log = sensor_data.pout;
        pressure_log = poutToPressureinH2O(sensor_data.pout) * INH2O_TO_PA;
        temperature_log = toutToTemperatureC(sensor_data.tout);
    }
}

static const DeckDriver downwash_deck = {
    .vid = 0xBC,
    .pid = 0x14,
    .name = "bcDownwash",
    .usedGpio = DECK_USING_IO_1,
    .usedPeriph = DECK_USING_I2C,
    // .requiredEstimator = StateEstimatorTypeKalman,
    .init = downwashDeckInit,
    .test = downwashDeckTest,
};

DECK_DRIVER(downwash_deck);

LOG_GROUP_START(downwash)
LOG_ADD(LOG_FLOAT, pressure, &pressure_log)
LOG_ADD(LOG_FLOAT, temperature, &temperature_log)
LOG_ADD(LOG_UINT32, pout, &pout_log)
LOG_ADD(LOG_UINT32, tout, &tout_log)
LOG_ADD(LOG_UINT8, status, &sensor_status)
LOG_ADD(LOG_UINT8, busy, &busy_status)
LOG_GROUP_STOP(downwash)