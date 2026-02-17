/**
 * @file aerolink_attitude_controller.h
 * @author Will Wu, Noah Silverman
 * ACT Lab @ Brown University, 2026
 * @brief Aerolink attitude controller interface
 */

#ifndef __AEROLINK_ATTITUDE_CONTROLLER_H__
#define __AEROLINK_ATTITUDE_CONTROLLER_H__

#include "stabilizer_types.h"

#include <stdbool.h>
#include <stdint.h>

void aerolinkAttitudeControllerInit(const float updateDt);
bool aerolinkAttitudeControllerTest(void);

void aerolinkAttitudeControllerCorrectAttitudePID(
      float eulerRollActual, float eulerPitchActual, float eulerYawActual,
      float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
      float rollRateActual, float pitchRateActual, float yawRateActual,
      float *rollOutput, float *pitchOutput, float *yawOutput);

void aerolinkAttitudeResetRollAttitudePID(float rollActual);
void aerolinkAttitudeResetPitchAttitudePID(float pitchActual);
void aerolinkAttitudeResetAllPID(float rollActual, float pitchActual, float yawActual);

float attitudeControllerGetYawMaxDelta(void);


#endif // __AEROLINK_ATTITUDE_CONTROLLER_H__