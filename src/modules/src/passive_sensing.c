/**
 * passive_sensing.c
 *
 * Passive sensing / interaction-force proxy estimation for Crazyflie
 *
 * - Uses IMU (accelerometer + gyroscope) and current attitude + thrust
 *   commands to estimate a proxy for external interaction forces.
 * - Provides simple features (acc magnitude, gyro energy) for logging or CRTP.
 * - Designed so the estimator can later be replaced with a full UKF.
 *
 * Current implementation:
 * - Residual-like computation (rigid body model).
 * - Estimates external force in the body frame.
 * - Low-pass filters the estimate for robustness.
 */

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "log.h"
#include "param.h"
#include "stabilizer.h"     // state_t, control_t, Axis3f
#include "sensors.h"        // sensorData_t
#include "passive_sensing.h" // declare passiveSensingOutput_t + APIs

// -----------------------------------------------------------------------------
// Parameters (defaults)
// -----------------------------------------------------------------------------

/** Mass of the Crazyflie [kg]. ~0.032 kg for CF2 */
static float ps_mass_kg = 0.032f;

/** Thrust coefficient [N / thrust_unit].
 *   F_thrust_body.z = k_thrust * control->thrust
 */
static float ps_k_thrust = 1.0e-6f;

/** Low-pass filter gain for external force estimate. */
static float ps_alpha_force = 0.1f;

// -----------------------------------------------------------------------------
// Internal state
// -----------------------------------------------------------------------------

typedef struct {
    Axis3f forceEstimateBody;   // estimated external force [N], body frame
} PassiveEstimatorState;

static PassiveEstimatorState psState;
static passiveSensingOutput_t psOut;

// -----------------------------------------------------------------------------
// Helper math
// -----------------------------------------------------------------------------

/** Convert quaternion q=[x,y,z,w] to rotation matrix R (body -> world). */
static void quatToRotationMatrix(const float qx, const float qy, const float qz,
                                 const float qw, float R[3][3]) {
    const float xx = qx * qx;
    const float yy = qy * qy;
    const float zz = qz * qz;
    const float xy = qx * qy;
    const float xz = qx * qz;
    const float yz = qy * qz;
    const float wx = qw * qx;
    const float wy = qw * qy;
    const float wz = qw * qz;

    R[0][0] = 1.0f - 2.0f * (yy + zz);
    R[0][1] = 2.0f * (xy - wz);
    R[0][2] = 2.0f * (xz + wy);

    R[1][0] = 2.0f * (xy + wz);
    R[1][1] = 1.0f - 2.0f * (xx + zz);
    R[1][2] = 2.0f * (yz - wx);

    R[2][0] = 2.0f * (xz - wy);
    R[2][1] = 2.0f * (yz + wx);
    R[2][2] = 1.0f - 2.0f * (xx + yy);
}

/** Rotate a vector from world to body frame: v_body = R^T * v_world */
static void rotateWorldToBody(const float R[3][3],
                              const Axis3f* vWorld,
                              Axis3f* vBody) {
    vBody->x = R[0][0] * vWorld->x + R[1][0] * vWorld->y + R[2][0] * vWorld->z;
    vBody->y = R[0][1] * vWorld->x + R[1][1] * vWorld->y + R[2][1] * vWorld->z;
    vBody->z = R[0][2] * vWorld->x + R[1][2] * vWorld->y + R[2][2] * vWorld->z;
}

/** Euclidean norm of a 3D vector. */
static float norm3(const Axis3f* v) {
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

// -----------------------------------------------------------------------------
// Core estimator logic
// -----------------------------------------------------------------------------

/**
 * Rigid-body residual for external force in body frame:
 *
 *   m * a_body = F_thrust_body + F_ext_body - m * g_body
 *   => F_ext_body = m*a_body + m*g_body - F_thrust_body
 */
static void estimateExternalForceBody(const sensorData_t* sensorData,
                                      const state_t* state,
                                      const control_t* control,
                                      PassiveEstimatorState* estState) {
    // Convert accelerometer from G to m/s^2
    Axis3f accBodyMs2 = {
        sensorData->acc.x * 9.81f,
        sensorData->acc.y * 9.81f,
        sensorData->acc.z * 9.81f
    };

    // Rotation matrix from quaternion
    float R[3][3];
    quatToRotationMatrix(
        state->attitudeQuaternion.x,
        state->attitudeQuaternion.y,
        state->attitudeQuaternion.z,
        state->attitudeQuaternion.w,
        R
    );

    // Gravity in world frame -> body frame
    Axis3f gWorld = {0.0f, 0.0f, -9.81f};
    Axis3f gBody;
    rotateWorldToBody(R, &gWorld, &gBody);

    // Thrust in body frame
    Axis3f thrustBody = {0.0f, 0.0f, ps_k_thrust * control->thrust};

    // Compute external force residual
    Axis3f FextBody;
    FextBody.x = ps_mass_kg * accBodyMs2.x + ps_mass_kg * gBody.x - thrustBody.x;
    FextBody.y = ps_mass_kg * accBodyMs2.y + ps_mass_kg * gBody.y - thrustBody.y;
    FextBody.z = ps_mass_kg * accBodyMs2.z + ps_mass_kg * gBody.z - thrustBody.z;

    // Low-pass filter
    const float alpha = ps_alpha_force;
    estState->forceEstimateBody.x =
        (1.0f - alpha) * estState->forceEstimateBody.x + alpha * FextBody.x;
    estState->forceEstimateBody.y =
        (1.0f - alpha) * estState->forceEstimateBody.y + alpha * FextBody.y;
    estState->forceEstimateBody.z =
        (1.0f - alpha) * estState->forceEstimateBody.z + alpha * FextBody.z;
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

void passiveSensingInit(void) {
    memset(&psState, 0, sizeof(psState));
    memset(&psOut, 0, sizeof(psOut));
}

void passiveSensingUpdate(const sensorData_t* sensorData,
                          const state_t* state,
                          const control_t* control,
                          passiveSensingOutput_t* out,
                          const float dt) {
    (void)dt; // unused for now

    // Update estimator
    estimateExternalForceBody(sensorData, state, control, &psState);

    // Features
    psOut.accMag = norm3(&sensorData->acc);
    psOut.gyroEnergy = norm3(&sensorData->gyro);

    // Force estimate
    psOut.force = psState.forceEstimateBody;

    // Torque not implemented
    psOut.torque.x = 0.0f;
    psOut.torque.y = 0.0f;
    psOut.torque.z = 0.0f;

    if (out) {
        *out = psOut;
    }
}

// -----------------------------------------------------------------------------
// Parameters and Logging
// -----------------------------------------------------------------------------

PARAM_GROUP_START(passive)
PARAM_ADD_CORE(PARAM_FLOAT, mass,    &ps_mass_kg)
PARAM_ADD_CORE(PARAM_FLOAT, kthrust, &ps_k_thrust)
PARAM_ADD_CORE(PARAM_FLOAT, alphaF,  &ps_alpha_force)
PARAM_GROUP_STOP(passive)

LOG_GROUP_START(passive)
LOG_ADD(LOG_FLOAT, fx,     &psOut.force.x)
LOG_ADD(LOG_FLOAT, fy,     &psOut.force.y)
LOG_ADD(LOG_FLOAT, fz,     &psOut.force.z)
LOG_ADD(LOG_FLOAT, accMag, &psOut.accMag)
LOG_ADD(LOG_FLOAT, gyroE,  &psOut.gyroEnergy)
LOG_GROUP_STOP(passive)
