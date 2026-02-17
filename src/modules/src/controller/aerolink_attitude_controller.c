/**
 * @file aerolink_attitude_controller.c
 * @author Will Wu, Noah Silverman
 * ACT Lab @ Brown University, 2026
 */
#include <stdbool.h>

#include "aerolink_attitude_controller.h"
#include "stabilizer_types.h"
#include "pid.h"
#include "param.h"
#include "log.h"
#include "commander.h"
#include "platform_defaults.h"

static bool isinit = false;
static float updateComboPID(PidObject* pid, const float measured_attitude,
                            const float measured_rate, const bool isYawAngle) {
  float output = 0.0f;

  pid->error = pid->desired - measured_attitude; // attitude
  if (isYawAngle){
    if (pid->error > 180.0f){
      pid->error -= 360.0f;
    } else if (pid->error < -180.0f){
      pid->error += 360.0f;
    }
  }
  float rateError = -measured_rate; // desired rate is 0
  if (isYawAngle){
    // treat angle treating
    if (rateError > 180.0f){
      rateError -= 360.0f;
    } else if (rateError < -180.0f){
      rateError += 360.0f;
    }
  }
  if (pid->enableDFilter) {
    rateError = lpf2pApply(&pid->dFilter, rateError);
  }
  if (isnan(rateError)) {
    rateError = 0;
  }

  pid->outP = pid->kp * pid->error;
  pid->outI = 0.0f;
  pid->outD = pid->kd * rateError;
  output = pid->outP + pid->outI + pid->outD;

  pid->prevMeasured = measured_attitude;
  return output;
}

PidObject pidRoll = {
  .kp = PID_ROLL_KP,
  .ki = 0,
  .kd = PID_ROLL_RATE_KP,
  .kff = PID_ROLL_KFF,
};

PidObject pidPitch = {
  .kp = PID_PITCH_KP,
  .ki = 0,
  .kd = PID_PITCH_RATE_KP,
  .kff = PID_PITCH_KFF,
};

PidObject pidYaw = {
  .kp = PID_YAW_KP,
  .ki = PID_YAW_KI,
  .kd = PID_YAW_KD,
  .kff = PID_YAW_KFF,
};

void aerolinkAttitudeControllerInit(const float updateDt) {
  if(isinit) {
      return;
  }
  pidInit(&pidRoll,  0, pidRoll.kp,  pidRoll.ki,  pidRoll.kd,  pidRoll.kff,  updateDt,
      ATTITUDE_RATE, ATTITUDE_ROLL_RATE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);
  pidInit(&pidPitch, 0, pidPitch.kp, pidPitch.ki, pidPitch.kd, pidPitch.kff, updateDt,
      ATTITUDE_RATE, ATTITUDE_PITCH_RATE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);
  pidInit(&pidYaw,   0, pidYaw.kp,   pidYaw.ki,   pidYaw.kd,   pidYaw.kff,   updateDt,
      ATTITUDE_RATE, ATTITUDE_YAW_RATE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);
  // set integral limits even though no integral term exist
  pidSetIntegralLimit(&pidRoll,  PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYaw,   PID_YAW_INTEGRATION_LIMIT);
  isinit = true;
}

bool aerolinkAttitudeControllerTest(void) {
  return isinit;
}

void aerolinkAttitudeControllerCorrectAttitudePID(
      float eulerRollActual, float eulerPitchActual, float eulerYawActual,
      float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
      float rollRateActual, float pitchRateActual, float yawRateActual,
      float *rollOutput, float *pitchOutput, float *yawOutput) {
  // manually update the PID, since we bundle position with rate
  // Kp(\Theta_desired - Theta_actual) + Kd(0 - Omega_actual)
  pidSetDesired(&pidRoll, eulerRollDesired);
  pidSetDesired(&pidPitch, eulerPitchDesired);
  pidSetDesired(&pidYaw, eulerYawDesired);

  *rollOutput = updateComboPID(&pidRoll, eulerRollActual, rollRateActual, false);
  *pitchOutput = updateComboPID(&pidPitch, eulerPitchActual, pitchRateActual, false);
  *yawOutput = updateComboPID(&pidYaw, eulerYawActual, yawRateActual, true);
}

void aerolinkAttitudeResetRollAttitudePID(float rollActual)
{
    pidReset(&pidRoll, rollActual);
}

void aerolinkAttitudeResetPitchAttitudePID(float pitchActual)
{
    pidReset(&pidPitch, pitchActual);
}

void aerolinkAttitudeResetAllPID(float rollActual, float pitchActual, float yawActual)
{
  pidReset(&pidRoll, rollActual);
  pidReset(&pidPitch, pitchActual);
  pidReset(&pidYaw, yawActual);
}
