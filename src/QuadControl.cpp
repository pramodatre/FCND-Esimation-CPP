#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#include <iostream>

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  // We need to solve for F1, F2, F3, and F4 using four equations.
  // F = F1 + F2 + F3 + F4
  // tou_x = F1 - F2 - F3 + F4
  // tou_y = F1 + F2 - F3 - F4
  // tou_z = tou_1 + tou_2 + tou_3 + tou_4 where
  // tou_1 = -k_m * w1^2, tou_2 = k_m * w2^2, 
  // tou_3 = -k_m * w3^2, tou_4 = k_m * w4^2
  // tou = k_m * w^2 (moment)
  // F = k_f * w^2 (thrust)
  // kappa is tourque per N of thrust = k_m / k_f
  float l = L / sqrt(2);
  // front left
  cmd.desiredThrustsN[0] = (1.f/4.f) * (collThrustCmd + momentCmd.x/l + momentCmd.y/l - momentCmd.z/kappa);
  // front right
  cmd.desiredThrustsN[1] = (1.f/4.f) * (collThrustCmd - momentCmd.x/l + momentCmd.y/l + momentCmd.z/kappa);
  // rear left
  cmd.desiredThrustsN[2] = (1.f/4.f) * (collThrustCmd + momentCmd.x/l - momentCmd.y/l + momentCmd.z/kappa);
  // rear right
  cmd.desiredThrustsN[3] = (1.f/4.f) * (collThrustCmd - momentCmd.x/l - momentCmd.y/l - momentCmd.z/kappa);
  // constrain the thrust to max thrusts that each motor can provide
  cmd.desiredThrustsN[0] = CONSTRAIN(cmd.desiredThrustsN[0], minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[1] = CONSTRAIN(cmd.desiredThrustsN[1], minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[2] = CONSTRAIN(cmd.desiredThrustsN[2], minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[3] = CONSTRAIN(cmd.desiredThrustsN[3], minMotorThrust, maxMotorThrust);

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  // To compute moments, we need rotational acceleration and 
  // moment of inertia around x, y, and z axis. Let's compute
  // angular accelerations around x, y, and z axis.
  float u_bar_p = kpPQR.x * (pqrCmd.x - pqr.x);
  float u_bar_q = kpPQR.y * (pqrCmd.y - pqr.y);
  float u_bar_r = kpPQR.z * (pqrCmd.z - pqr.z);
  // Using moment of inertia to compute moments
  momentCmd.x = Ixx * u_bar_p;
  momentCmd.y = Iyy * u_bar_q;
  momentCmd.z = Izz * u_bar_r;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  float b_x_a = R(0, 2);
  float b_y_a = R(1, 2);
  // From accelCmd we need to find commanded b's b_x_c, b_y_c
  // accelCmd.x = c * b_x_c
  // accelCmd.y = c * b_y_c
  // collThrustCmd is in N. Converting it to acceleration.
  // collThrustCmd = m * a
  if(collThrustCmd > 0)
  {
    float c = collThrustCmd / mass;
    float b_x_c = -CONSTRAIN(accelCmd.x / c, -maxTiltAngle, maxTiltAngle);
    float b_y_c = -CONSTRAIN(accelCmd.y / c, -maxTiltAngle, maxTiltAngle);
    float b_dot_x_c = kpBank * (b_x_a - b_x_c);
    float b_dot_y_c = kpBank * (b_y_a - b_y_c);
    // Computing p_c and q_c using
    // p_c = (1/R33) * (R21 * b_dot_x_c - R11 * b_dot_y_c)
    // q_c = (1/R33) * (R22 * b_dot_x_c - R12 * b_dot_y_c)
    pqrCmd.x = (1.f / R(2, 2)) * (-R(1, 0) * b_dot_x_c + R(0, 0) * b_dot_y_c);
    pqrCmd.y = (1.f / R(2, 2)) * (-R(1, 1) * b_dot_x_c + R(0, 1) * b_dot_y_c);
    pqrCmd.z = 0.f;
  }
  else
  {
    pqrCmd.x = 0.f;
    pqrCmd.y = 0.f;
    pqrCmd.z = 0.f;
  }

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  // check if commanded velocity is within bounds
  velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);
  float z_err = posZCmd - posZ;
  float z_dot_err = velZCmd - velZ;
  // Accumulated error
  integratedAltitudeError += z_err * dt;
  float u1_bar = kpPosZ * z_err + kpVelZ * z_dot_err + accelZCmd + KiPosZ * integratedAltitudeError;
  float b_z = R(2, 2);
  thrust = CONSTRAIN(mass * (CONST_GRAVITY - u1_bar) / b_z, 4 * minMotorThrust, 4 * maxMotorThrust);
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //check if commanded speed is within the limits, otherwise
  // contrain the speed values to desired range for the quad
  velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
  velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);
  // x direction
  float x_diff = posCmd.x - pos.x;
  float x_dot_diff = velCmd.x - vel.x;
  float x_a = accelCmd.x + kpPosXY * x_diff + kpVelXY * x_dot_diff;
  accelCmd.x = CONSTRAIN(x_a, -maxAccelXY, maxAccelXY);
  // y direction
  float y_diff = posCmd.y - pos.y;
  float y_dot_diff = velCmd.y - vel.y;
  float y_a = accelCmd.y + kpPosXY * y_diff + kpVelXY * y_dot_diff;
  accelCmd.y = CONSTRAIN(y_a, -maxAccelXY, maxAccelXY);
  accelCmd.z = 0.f;
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  yawCmd = fmodf(yawCmd, 2 * F_PI);
  float yaw_err = yawCmd - yaw;
  if(yaw_err > F_PI)
  {
    yaw_err = yaw_err - 2 * F_PI;
  }
  else if(yaw_err < -F_PI)
  {
    yaw_err = yaw_err + 2 * F_PI;
  }

  yawRateCmd = kpYaw * yaw_err;

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
