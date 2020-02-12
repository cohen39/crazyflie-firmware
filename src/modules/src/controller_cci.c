
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "attitude_controller_ca.h"
#include "sensfusion6.h"
#include "position_controller.h"
#include "controller_cci.h"

#include "casadi/mem.h"
#include "casadi/gen/f_control.h"


#include "log.h"
#include "param.h"
#include "math3d.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static bool tiltCompensationEnabled = false;

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

casadi_mem *att_ctrl_mem = 0;
float i_climbr_err = 0;

void controllerCCIInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
}

bool controllerCCITest(void)
{
  bool pass = true;

  // pass &= attitudeControllerTest();

  return pass;
}

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}

void controllerCCI(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Rate-controled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) {
       attitudeDesired.yaw += setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT;
    } else {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    }

    attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
    attitudeDesired.roll = setpoint->attitude.roll;
    attitudeDesired.pitch = setpoint->attitude.pitch;
    actuatorThrust = setpoint->thrust;

    att_ctrl_mem = casadi_alloc(f_control_functions);

    float u_cmd[4] = {
      attitudeDesired.roll,
      attitudeDesired.pitch,
      attitudeDesired.yaw,
      actuatorThrust
    };

    float quat[4] = {
      state->attitudeQuaternion.q0,
      state->attitudeQuaternion.q1,
      state->attitudeQuaternion.q2,
      state->attitudeQuaternion.q3
    };

    float omega_b[3] = {
      sensors->gyro.x,
      sensors->gyro.y,
      sensors->gyro.z
    };

    float v_n[3] = {
      state->velocity.x,
      state->velocity.y,
      state->velocity.z
    };

    float gains[6] = {
		  4,     	//kp_roll, kp_pitch
      0.2,   	//kp_rollr, kp_pitchr
      1,     	//kp_yaw
      0.2,   	//kp_yawr
      0.5,   	//kp_climbr
     	10     	//ki_climbr
	  };

    float u_control[4];
    float dt = ATTITUDE_UPDATE_DT;

    att_ctrl_mem->arg[0] = quat;
    att_ctrl_mem->arg[1] = omega_b;
    att_ctrl_mem->arg[2] = v_n;
    att_ctrl_mem->arg[3] = u_cmd;
    att_ctrl_mem->arg[4] = &i_climbr_err;
    att_ctrl_mem->arg[5] = gains;
    att_ctrl_mem->arg[6] = &dt;
    att_ctrl_mem->res[0] = u_control;
    att_ctrl_mem->res[1] = &i_climbr_err;
    casadi_eval(att_ctrl_mem);

    control->yaw = u_control[0];
    control->pitch = u_control[1];
    control->roll = u_control[2];
    control->thrust = u_control[3];

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    casadi_free(att_ctrl_mem);
  }

  if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
    positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
  }

  // if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
  //   // Switch between manual and automatic position control
  //   if (setpoint->mode.z == modeDisable) {
  //     actuatorThrust = setpoint->thrust;
  //   }
  //   if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
  //     attitudeDesired.roll = setpoint->attitude.roll;
  //     attitudeDesired.pitch = setpoint->attitude.pitch;
  //   }

  //   attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
  //                               attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
  //                               &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

  //   // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
  //   // value. Also reset the PID to avoid error buildup, which can lead to unstable
  //   // behavior if level mode is engaged later
  //   if (setpoint->mode.roll == modeVelocity) {
  //     rateDesired.roll = setpoint->attitudeRate.roll;
  //     attitudeControllerResetRollAttitudePID();
  //   }
  //   if (setpoint->mode.pitch == modeVelocity) {
  //     rateDesired.pitch = setpoint->attitudeRate.pitch;
  //     attitudeControllerResetPitchAttitudePID();
  //   }

  //   // TODO: Investigate possibility to subtract gyro drift.
  //   attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
  //                            rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

  //   attitudeControllerGetActuatorOutput(&control->roll,
  //                                       &control->pitch,
  //                                       &control->yaw);

  //   control->yaw = -control->yaw;

  //   cmd_thrust = control->thrust;
  //   cmd_roll = control->roll;
  //   cmd_pitch = control->pitch;
  //   cmd_yaw = control->yaw;
  //   r_roll = radians(sensors->gyro.x);
  //   r_pitch = -radians(sensors->gyro.y);
  //   r_yaw = radians(sensors->gyro.z);
  //   accelz = sensors->acc.z;
  // }

  // if (tiltCompensationEnabled)
  // {
  //   control->thrust = actuatorThrust / sensfusion6GetInvThrustCompensationForTilt();
  // }
  // else
  // {
  //   control->thrust = actuatorThrust;
  // }

  // if (control->thrust == 0)
  // {
  //   control->thrust = 0;
  //   control->roll = 0;
  //   control->pitch = 0;
  //   control->yaw = 0;

  //   cmd_thrust = control->thrust;
  //   cmd_roll = control->roll;
  //   cmd_pitch = control->pitch;
  //   cmd_yaw = control->yaw;

  //   attitudeControllerResetAllPID();
  //   positionControllerResetAllPID();

  //   // Reset the calculated YAW angle for rate control
  //   attitudeDesired.yaw = state->attitude.yaw;
  // }
}


LOG_GROUP_START(controller)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
LOG_ADD(LOG_FLOAT, accelz, &accelz)
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)

PARAM_GROUP_START(controller)
PARAM_ADD(PARAM_UINT8, tiltComp, &tiltCompensationEnabled)
PARAM_GROUP_STOP(controller)
