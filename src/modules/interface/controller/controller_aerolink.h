/**
 * @file controller_aerolink.h
 * @author Will Wu, Noah Silverman
 * ACT Lab @ Brown University, 2026
 * @brief Aerolink controller interface
 */

#ifndef __CONTROLLER_AEROLINK_H__
#define __CONTROLLER_AEROLINK_H__

#include "stabilizer_types.h"

void controllerAerolinkInit(void);
bool controllerAerolinkTest(void);
void controllerAerolink(control_t *control,
                        const setpoint_t *setpoint,
                        const sensorData_t *sensors,
                        const state_t *state,
                        const stabilizerStep_t stabilizerStep);

#endif // __CONTROLLER_AEROLINK_H__