#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
  //printed = false;
  //quadNumber = 1;
  //maxX = 0;

 #ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0); // Kp_pos
  kpPosZ = config->Get(_config + ".kpPosZ", 0); // Kp_alt
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0); // Kp_vel
  kpVelZ = config->Get(_config + ".kpVelZ", 0); // Kp_hdot ?

  kpBank = config->Get(_config + ".kpBank", 0); // Kp_roll, Kp_pitch
  kpYaw = config->Get(_config + ".kpYaw", 0); // Kp_yaw

  kpPQR = config->Get(_config + ".kpPQR", V3F()); // Kp_p, Kp_q, Kp_r

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

  float l = L / (float)sqrt(2);

  float tau_x = momentCmd.x / l;
  float tau_y = momentCmd.y / l;
  float tau_z = momentCmd.z / -kappa;

  float thrust_1 = (collThrustCmd + tau_x + tau_y + tau_z) / 4; // front left
  float thrust_2 = (collThrustCmd - tau_x + tau_y - tau_z) / 4; // front right
  float thrust_3 = (collThrustCmd + tau_x - tau_y - tau_z) / 4; // rear left These last two may be backwards
  float thrust_4 = (collThrustCmd - tau_x - tau_y + tau_z) / 4; // rear right

  cmd.desiredThrustsN[0] = thrust_1; // front left
  cmd.desiredThrustsN[1] = thrust_2; // front right
  cmd.desiredThrustsN[2] = thrust_3; // rear left
  cmd.desiredThrustsN[3] = thrust_4; // rear right

  //printf("CollectiveThrustCommands: %f   %f", collThrustCmd, thrust_1 + thrust_2 + thrust_3 + thrust_4);

  //cmd.desiredThrustsN[0] = mass * 9.81f / 4.f; // front left
  //cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right
  //cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left
  //cmd.desiredThrustsN[3] = mass * 9.81f / 4.f; // rear right

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
  V3F MOI = (Ixx, Iyy, Izz);
 
  V3F rate_error = pqrCmd - pqr;

  momentCmd = MOI * (kpPQR * rate_error);  

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

	float c_d = collThrustCmd / mass;

	//if (collThrustCmd > 0.0)
	//{
		float target_R13 = -CONSTRAIN(accelCmd[0] / c_d, -maxTiltAngle, maxTiltAngle);
		float target_R23 = -CONSTRAIN(accelCmd[1] / c_d, -maxTiltAngle, maxTiltAngle);
		
		pqrCmd[0] = (1 / R(2, 2)) * (-R(1, 0) * kpBank * (R(0, 2) - target_R13) + R(0, 0) * kpBank * (R(1, 2) - target_R23));
		pqrCmd[1] = (1 / R(2, 2)) * (-R(1, 1) * kpBank * (R(0, 2) - target_R13) + R(0, 1) * kpBank * (R(1, 2) - target_R23));
		
		//}
	//else  // Otherwise command no rate
	//{
	//	//print("negative thrust command")
	//	pqrCmd[0] = 0.0;
	//	pqrCmd[1] = 0.0;
	//	collThrustCmd = 0.0;
	//}

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
  velZCmd = kpPosZ * (posZCmd - posZ) + velZCmd;

  // Limit the ascent / descent rate
  velZCmd = CONSTRAIN(velZCmd, -maxDescentRate, maxAscentRate);

  integratedAltitudeError += (posZCmd - posZ) * dt;
  accelZCmd = kpVelZ * (velZCmd - velZ) + KiPosZ * integratedAltitudeError + accelZCmd;

  thrust = mass * (9.81f -accelZCmd) / R(2, 2);  

  if (thrust > maxMotorThrust * 4)
	  thrust = maxMotorThrust * 4;
  else if (thrust < minMotorThrust * 4)
	  thrust = minMotorThrust * 4;

  //printf("Z position %f \n", posZ);

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

  velCmd.x = kpPosXY * (posCmd.x - pos.x) + velCmd.x;
  velCmd.y = kpPosXY * (posCmd.y - pos.y) + velCmd.y;

  //Limit speed
  float velocity_norm = sqrt(velCmd.x* velCmd.x + velCmd.y * velCmd.y);
  if (velocity_norm > maxSpeedXY)
  {
	  velCmd.x = velCmd.x * maxSpeedXY / velocity_norm;
	  velCmd.y = velCmd.y * maxSpeedXY / velocity_norm;
	  //printf("MVXY");
  }

  accelCmd = accelCmdFF + kpPosXY * (posCmd - pos) + kpVelXY * (velCmd - vel);

  //Limit acceleration
  float accel_norm = sqrt(accelCmd.x* accelCmd.x + accelCmd.y * accelCmd.y);
  if (accel_norm > maxAccelXY)
  {
	  accelCmd.x = accelCmd.x * maxAccelXY / accel_norm;
	  accelCmd.y = accelCmd.y * maxAccelXY / accel_norm;
	  //printf("MAXY");
  }
  
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
  float yawRateCmd = 0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

	float PI = (float)M_PI;
	yawCmd = fmodf(yawCmd, 2.f * PI);
	float yawRateError = yawCmd - yaw;

	if (yawRateError > M_PI)
		yawRateError = yawRateError - 2.f*PI;
	else if(yawRateError < -M_PI)
		yawRateError = yawRateError + 2.f*PI;

	yawRateCmd= kpYaw * yawRateError;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
	curTrajPoint = GetNextTrajectoryPoint(simTime);

	float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

	// reserve some thrust margin for angle control
	float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
	collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust + thrustMargin)*4.f, (maxMotorThrust - thrustMargin)*4.f);

	V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);

	V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
	desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

	V3F desMoment = BodyRateControl(desOmega, estOmega);

	// Test code to help tune parameters
	//// remember which quad it is 
	//if (quadNumber == 2) quadNumber = 1; else quadNumber = 2
	//if (printed & simTime < 1.f) printed = false;  # Reset 'printed' flag

	//if (simTime> 4.f & quadNumber == 1) # Wait for quads to settle down
	//{
	//	if (maxX < abs(estPos.x - curTrajPoint.position.x))
	//	{
	//		maxX = abs(estPos.x - curTrajPoint.position.x);		# Remember highest value
	//	}
	//}
	// 
	//if (!printed & simTime > 8.f) # Wait till near end of simulation
	//{
	//	printf("Quad %i     Max Pos X= %f      Y= %f \n", quadNumber, maxX, estPos.y);
	//	printed = true;
	//}

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
