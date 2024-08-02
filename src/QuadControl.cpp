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

#ifndef __PX4_NUTTX
	// Load params from simulator parameter system
	ParamsHandle config = SimpleConfig::GetInstance();

	// Load parameters (default to 0)
	kpPosXY = config->Get(_config + ".kpPosXY", 0);
	kpPosZ = config->Get(_config + ".kpPosZ", 0);
	KiPosZ = config->Get(_config + ".KiPosZ", 0);

	kpVelXY = config->Get(_config + ".kpVelXY", 0);
	kpVelZ = config->Get(_config + ".kpVelZ", 0);

	kpBank = config->Get(_config + ".kpBank", 0);
	kdBank = config->Get(_config + ".kdBank", 0);
	kpYaw = config->Get(_config + ".kpYaw", 0);
	kdYaw = config->Get(_config + ".kdYaw", 0);
	KiYaw = config->Get(_config + ".KiYaw", 0);

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

	float l = L / sqrtf(2.f);

	// Desired moments
	float c = collThrustCmd;
	float p = momentCmd.x / l;  // Roll moment command
	float q = momentCmd.y / l;  // Pitch moment command
	float r = momentCmd.z / this->kappa;  // Yaw moment command

	// Calculating individual thrusts for the motors
	// Honestly I don't know what motor is which right now, it seems that they are mixed up
	float T1 = (c + p + q + r) / 4.0f;  // Front right
	float T2 = (c - p + q - r) / 4.0f;  // Front left
	float T3 = (c + p - q - r) / 4.0f;  // Rear right
	float T4 = (c - p - q + r) / 4.0f;  // Rear left

	T1 = CONSTRAIN(T1, 0, maxMotorThrust);
	T2 = CONSTRAIN(T2, 0, maxMotorThrust);
	T3 = CONSTRAIN(T3, 0, maxMotorThrust);
	T4 = CONSTRAIN(T4, 0, maxMotorThrust);

	// Setting the thrust commands to the cmd array
	cmd.desiredThrustsN[0] = T1;  // Front right
	cmd.desiredThrustsN[1] = T2;  // Front left
	cmd.desiredThrustsN[2] = T3;  // Rear right
	cmd.desiredThrustsN[3] = T4;  // Rear left

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

	// Calculate the body rate error
	V3F rateError = pqrCmd - pqr;

	// Proportional control to compute the control torques
	V3F controlTorque = kpPQR * rateError;

	// Compute the desired moments by multiplying by the moments of inertia
	momentCmd.x = Ixx * controlTorque.x;
	momentCmd.y = Iyy * controlTorque.y;
	momentCmd.z = Izz * controlTorque.z;

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


	// If collThrustCmd is close to zero, we can't control roll/pitch effectively
	// Thats because we calculate acceleration
	if (collThrustCmd > 0.0f) {
		// Convert collective thrust command to acceleration (m/s^2)
		float c = -collThrustCmd / mass;

		// Extract current rotation matrix elements
		float R11 = R(0, 0);
		float R12 = R(0, 1);
		float R13 = R(0, 2);
		float R21 = R(1, 0);
		float R22 = R(1, 1);
		float R23 = R(1, 2);
		float R33 = R(2, 2);

		// Desired roll and pitch angles
		float b_x_c_target = accelCmd.x / c;
		float b_y_c_target = accelCmd.y / c;

		// Limit targets to avoid excessive tilting
		b_x_c_target = CONSTRAIN(b_x_c_target, -maxTiltAngle, maxTiltAngle);
		b_y_c_target = CONSTRAIN(b_y_c_target, -maxTiltAngle, maxTiltAngle);

		// Current roll and pitch angles
		float b_x = R13;
		float b_y = R23;

		// Errors
		float b_x_err = b_x_c_target - b_x;
		float b_y_err = b_y_c_target - b_y;

		// Derivative commands
		float b_x_p_term = kpBank * b_x_err;
		float b_y_p_term = kpBank * b_y_err;

		// Calculate desired p and q rates
		float p_c = (R21 * b_x_p_term - R11 * b_y_p_term) / R33;
		float q_c = (R22 * b_x_p_term - R12 * b_y_p_term) / R33;

		// Set desired pqr rates
		pqrCmd.x = p_c;
		pqrCmd.y = q_c;
	}
	else {
		pqrCmd.x = 0.0f;
		pqrCmd.y = 0.0f;
	}

	// Z rate remains zero
	pqrCmd.z = 0.0f;


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
	velZCmd = CONSTRAIN(velZCmd, -maxDescentRate, maxAscentRate);

	// Proportional, derivative and integration control for altitude
	float zError = posZCmd - posZ;
	float zDotError = velZCmd - velZ;
	float pTerm = kpPosZ * zError;
	float dTerm = kpVelZ * zDotError;
	float u1Bar = pTerm + dTerm + accelZCmd;
	integratedAltitudeError += zError * dt;
	u1Bar += KiPosZ * integratedAltitudeError;

	// Calculate thrust in the body frame
	float R33 = R(2, 2);
	float accelZ = u1Bar - CONST_GRAVITY;  // accelZ should account for gravity
	thrust = -mass * accelZ / R33;  // Negative because NED coordinates


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


	// Limit the desired velocity to maxSpeedXY
	if (velCmd.magXY() > maxSpeedXY) {
		velCmd = velCmd.norm() * maxSpeedXY;
	}

	// Calculate position error
	V3F posError = posCmd - pos;

	// Calculate velocity error
	V3F velError = velCmd - vel;

	// Calculate errors
	V3F pTerm = kpPosXY * posError;
	V3F dTerm = kpVelXY * velError;

	// Desired acceleration
	accelCmd += pTerm + dTerm;

	// Limit the desired acceleration to maxAccelXY
	if (accelCmd.magXY() > maxAccelXY) {
		accelCmd = accelCmd.norm() * maxAccelXY;
	}

	// Ensure the z-component of acceleration is zero
	accelCmd.z = 0;

	/////////////////////////////// END STUDENT CODE ////////////////////////////

	return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw, float dt)
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

	// Normalize the commanded yaw to be within the range [-pi, pi]
	yawCmd = fmodf(yawCmd, 2.0f * F_PI);
	if (yawCmd > F_PI) {
		yawCmd -= 2.0f * F_PI;
	}
	else if (yawCmd < -F_PI) {
		yawCmd += 2.0f * F_PI;
	}

	// Calculate the yaw error and normalize to handle wrap-around
	float yawError = yawCmd - yaw;
	if (yawError > F_PI) {
		yawError -= 2.0f * F_PI;
	}
	else if (yawError < -F_PI) {
		yawError += 2.0f * F_PI;
	}

	// Calculate the yaw velocity
	float yawVel = (yaw - prevYaw) / dt;
	float yawVelError = 0 - yawVel;
	if (yawVelError > F_PI) {
		yawVelError -= 2.0f * F_PI;
	}
	else if (yawVelError < -F_PI) {
		yawVelError += 2.0f * F_PI;
	}

	// Proportional control for yaw (assuming that dt<1 to avoid excessing F_PI)
	integratedYawError += yawError * dt;

	yawRateCmd = kpYaw * yawError + kdYaw * yawVelError + integratedYawError * KiYaw;
	prevYaw = yaw;

	// IMPORTANT: I don't know why but this one needs to be negative. I can't wrap my head around it :)
	yawRateCmd = -yawRateCmd;

	/////////////////////////////// END STUDENT CODE ////////////////////////////

	return yawRateCmd;
}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
	curTrajPoint = GetNextTrajectoryPoint(simTime);

	float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

	// reserve some thrust margin for angle control
	float thrustMargin = .1f * (maxMotorThrust - minMotorThrust);
	collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust + thrustMargin) * 4.f, (maxMotorThrust - thrustMargin) * 4.f);

	V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);

	V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
	desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw(), dt); // I added dt here to calc I and D parts of PID

	V3F desMoment = BodyRateControl(desOmega, estOmega);
	
	return GenerateMotorCommands(collThrustCmd, desMoment);
}
