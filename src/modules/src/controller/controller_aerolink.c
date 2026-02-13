/**
 * @file controller_aerolink.c
 * @author Will Wu, Noah Silverman
 * ACT Lab @ Brown University, 2026
 * @brief Aerolink controller implementation
 * Distributed attitude controller with structure-aware force-torque mixing
 */

#include "controller_aerolink.h"
#include "attitude_controller.h"

#include <stdbool.h>

#include "stabilizer_types.h"

#include "pid.h"
#include "param.h"
#include "log.h"
#include "commander.h"
#include "platform_defaults.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

void controllerAerolinkInit(void) {
    attitudeControllerInit(ATTITUDE_UPDATE_DT);
}


bool controllerAerolinkTest(void) {
    // more test here in the future
    bool pass = true;
    pass &= attitudeControllerTest();
    return pass;
}

void controllerAerolink(control_t *control,
                        const setpoint_t *setpoint,
                        const sensorData_t *sensors,
                        const state_t *state,
                        const stabilizerStep_t stabilizerStep) {
    // to be implemented
}


// LOG Groups