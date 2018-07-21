/*
The MIT License (MIT)

Copyright (c) 2018 James Alan Preiss

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
This controller is based on the following publication:

Daniel Mellinger, Vijay Kumar:
Minimum snap trajectory generation and control for quadrotors.
IEEE International Conference on Robotics and Automation (ICRA), 2011.

We added the following:
 * Integral terms (compensates for: battery voltage drop over time, unbalanced center of mass due to asymmmetries, and uneven wear on propellers and motors)
 * D-term for angular velocity
 * Support to use this controller as an attitude-only controller for manual flight

This controller is identical to controller_mellinger but uses real physical units everywhere.
*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "position_controller.h"
#include "controller_libquadrotor.h"
#include "quad_control.h"

struct quad_physical_params physical = {
  // from "System Identification of the Crazyflie 2.0 Nano Quadrocopter".
  // J. Foerster, M. Hamer, R. D'Andrea. Bachelor Thesis, ETH Zurich, 2015.
  .mass = 0.027f,
  .arm_length = 0.046f,
  .inertia = { 1.66e-5f, 1.66e-5f, 2.92e-5f },
  .thrust_to_torque = 0.006f,
  .max_thrust = 0.15f,
  .drag = 0.0f, // TODO
  .motor_layout = 'x',
  .motor_0_ccw = true, // TODO
};

struct quad_ctrl_SE3_params params = {
  .linear = {
    .kp = {.xy = 12.5f, .z = 39.0f},
    .ki = {.xy = 1.55f, .z = 1.55f},
    .kd = {.xy = 6.25f, .z = 12.5f},
  },
  .omega = {
    .kp = {.xy = 18.6f, .z = 11.2f},
    .ki = {.xy = 0.00f, .z = 0.00f},
    //.kd = {.xy = 0.0f, .z = 0.0f}, // TODO
  },
  .attitude = {
    .kp = {.xy = 130.3f, .z = 111.7f},
  },

  .int_linear_bound = {.x = 2.0f, .y = 2.0f, .z = 0.4f},
  .int_omega_bound = {.x = 1.0f, .y = 1.0f, .z = 0.0f}, // TODO
};

struct quad_ctrl_SE3_state state;


//
// public functions
//

void controllerLibQuadrotorInit(void)
{
  quad_ctrl_SE3_init(&state);
}

bool controllerLibQuadrotorTest(void)
{
  return true;
}

static inline struct vec v2math(struct vec3_s v)
{
  return mkvec(v.x, v.y, v.z);
}

static inline struct quat q2math(struct quaternion_s q)
{
  return mkquat(q.x, q.y, q.z, q.w);
}

static inline struct vec att2math(struct attitude_s a)
{
  return mkvec(radians(a.roll), radians(a.pitch), radians(a.yaw));
}

static inline struct vec3_s math2v(struct vec v, uint32_t timestamp)
{
  struct vec3_s vv = { .x = v.x, .y = v.y, .z = v.z, .timestamp = timestamp };
  return vv;
}

void controllerLibQuadrotor(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *in_state,
                                         const uint32_t tick)
{
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

  float const dt = (float)(1.0f / ATTITUDE_RATE);

  // construct libquadrotor setpoint
  struct quad_state set = {
    .pos = v2math(setpoint->position),
    .vel = v2math(setpoint->velocity),
    .acc = v2math(setpoint->acceleration),

    .quat = q2math(setpoint->attitudeQuaternion),
    .omega = att2math(setpoint->attitudeRate),
  };

  // construct libquadrotor state
  struct quad_state s = {
    .pos = v2math(in_state->position),
    .vel = v2math(in_state->velocity),
    .acc = vscl(GRAV, v2math(in_state->acc)),

    .quat = q2math(in_state->attitudeQuaternion),
    .omega = vscl(radians(1.0f), mkvec(sensors->gyro.x, sensors->gyro.y, sensors->gyro.z)),
    //.omega = att2math(sensors->gyro),
    //.omega = vscl(radians(1.0f), v2math(sensors->gyro)),
  };


  if (setpoint->mode.x == modeAbs) {
    quad_ctrl_SE3(&state, &params, &s, &set, dt);
  } else {
    // TODO
    /*
    target_thrust.x = -sinf(radians(setpoint->attitude.pitch));
    target_thrust.y = -sinf(radians(setpoint->attitude.roll));
    // In case of a timeout, the commander tries to level, ie. x/y are disabled, but z will use the previous setting
    // In that case we ignore the last feedforward term for acceleration
    if (setpoint->mode.z == modeAbs) {
      target_thrust.z = g_vehicleMass * GRAVITY_MAGNITUDE + kp_z  * r_error.z + kd_z  * v_error.z + ki_z  * i_error_z;
    } else {
      target_thrust.z = 1;
    }
    */
  }

  if (setpoint->mode.z == modeDisable) {
    //control->thrust = setpoint->thrust;
  } else {
    //control->thrust = massThrust * current_thrust;
  }

  // TODO convert to integer units
  // TODO flip pitch axis?
}

PARAM_GROUP_START(ctrlLibQ)
PARAM_ADD(PARAM_FLOAT, kp_xy, &params.linear.kp.xy)
PARAM_ADD(PARAM_FLOAT, kd_xy, &params.linear.kd.xy)
PARAM_ADD(PARAM_FLOAT, ki_xy, &params.linear.ki.xy)
//PARAM_ADD(PARAM_FLOAT, i_range_xy, &i_range_xy)
PARAM_ADD(PARAM_FLOAT, kp_z, &params.linear.kp.z)
PARAM_ADD(PARAM_FLOAT, kd_z, &params.linear.kd.z)
PARAM_ADD(PARAM_FLOAT, ki_z, &params.linear.ki.z)
//PARAM_ADD(PARAM_FLOAT, i_range_z, &i_range_z)
PARAM_ADD(PARAM_FLOAT, mass, &physical.mass)
PARAM_ADD(PARAM_FLOAT, kR_xy, &params.attitude.kp.xy)
PARAM_ADD(PARAM_FLOAT, kR_z, &params.attitude.kp.z)
PARAM_ADD(PARAM_FLOAT, kw_xy, &params.omega.kp.xy)
PARAM_ADD(PARAM_FLOAT, kw_z, &params.omega.kp.z)
//PARAM_ADD(PARAM_FLOAT, ki_m_xy, &ki_m_xy)
//PARAM_ADD(PARAM_FLOAT, ki_m_z, &ki_m_z)
//PARAM_ADD(PARAM_FLOAT, kd_omega_rp, &kd_omega_rp)
//PARAM_ADD(PARAM_FLOAT, i_range_m_xy, &i_range_m_xy)
//PARAM_ADD(PARAM_FLOAT, i_range_m_z, &i_range_m_z)
PARAM_GROUP_STOP(ctrlLibQ)

LOG_GROUP_START(ctrlLibQ)
LOG_GROUP_STOP(ctrlLibQ)
