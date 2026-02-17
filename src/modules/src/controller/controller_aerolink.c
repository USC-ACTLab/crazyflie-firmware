/**
 * @file controller_aerolink.c
 * @author Will Wu, Noah Silverman
 * ACT Lab @ Brown University, 2026
 * @brief Aerolink controller implementation
 * Distributed attitude controller with structure-aware force-torque mixing
 */

#include "controller_aerolink.h"
#include "aerolink_attitude_controller.h"

#include <stdbool.h>

#include "stabilizer_types.h"

#include "pid.h"
#include "param.h"
#include "log.h"
#include "commander.h"
#include "platform_defaults.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static float Ixx, Iyy, Izz;
static unsigned int num_module_in_structure;
static bool isinit = false;
static bool receivedStructureInfo = false;

void controllerAerolinkInit(void) {
    aerolinkAttitudeControllerInit(ATTITUDE_UPDATE_DT);
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
    control->controlMode = controlModeStructure;
    if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
        float rollOutput, pitchOutput, yawOutput;
        aerolinkAttitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                                    setpoint->attitude.roll, setpoint->attitude.pitch, setpoint->attitude.yaw,
                                                    sensors->gyro.x, sensors->gyro.y, sensors->gyro.z,
                                                    &rollOutput, &pitchOutput, &yawOutput);
    }
}

// Param Groups
PARAM_GROUP_START(aerolink)
/**
 * @brief Moment of inertia around X-axis [kg*m^2]
 */
PARAM_ADD(PARAM_FLOAT, Ixx, &Ixx)
/**
 * @brief Moment of inertia around Y-axis [kg*m^2]
 */
PARAM_ADD(PARAM_FLOAT, Iyy, &Iyy)
/**
 * @brief Moment of inertia around Z-axis [kg*m^2]
 */
PARAM_ADD(PARAM_FLOAT, Izz, &Izz)
/**
 * @brief Number of modules in the structure
 * It's pretty unlikely we will fly more than 255 modules atm, but I do hope one day we will!
 */
PARAM_ADD(PARAM_UINT8, num_modules, &num_module_in_structure)

PARAM_GROUP_STOP(aerolink)

// LOG Groups