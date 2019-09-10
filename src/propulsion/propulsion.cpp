#include "propulsion.hpp"

using namespace std;

using Eigen::Vector3d;
using Eigen::Quaterniond;

//////////////////////////////
// Propulsion interfrace class
//////////////////////////////

// Constructor
Propulsion::Propulsion(YAML::Node propConfig, YAML::Node worldConfig)
{
	getParameter(worldConfig, "deltaT", dt);

	vector<double> doubleVect;
	getParameterList(propConfig, "CGOffset", doubleVect);
	CGOffset = Vector3d(doubleVect.data());

	doubleVect.clear();
	getParameterList(propConfig, "mountOrientation", doubleVect);
	mountOrientation = Vector3d(doubleVect.data());

	theta = 0; // Initialize propeller angle

	if (!getParameter(propConfig, "chanMotor", chanMotor, false)) {chanMotor = -1;}
	if (!getParameter(propConfig, "chanGimbal", chanGimbal, false)) {chanGimbal = -1;}
	if (!getParameter(propConfig, "gimbalAngle_max", gimbalAngle_max, false)) {gimbalAngle_max = 0.0;}

	inputMotor = 0.0;
	inputGimbal = 0.0;

	if (!getParameter(propConfig, "rotationDir", rotationDir, false)) {rotationDir=1.0;}
}

// Destructor
Propulsion::~Propulsion()
{
}

void Propulsion::setInput(Input_t input, YAML::Node config)
{
	// TODO: I shouldn't check this all the time, since it is an optional parameter
	getParameter(config, "gimbalAngle_max", gimbalAngle_max);
	if (chanMotor>-1) {inputMotor = input.value[chanMotor];}
	if (chanGimbal>-1) {inputGimbal = gimbalAngle_max * input.value[chanGimbal]; }
}

void Propulsion::setInputPwm(InputPwm_t p_input, YAML::Node config)
{
	Input_t input;
	if (chanMotor>-1) {input.value[chanMotor] = PwmToHalfRange(p_input.value[chanMotor]);}
	if (chanGimbal>-1) {input.value[chanGimbal] = PwmToFullRange(p_input.value[chanGimbal]);}

	setInput(input, config);
}

// Engine physics step, container for the generic class
void Propulsion::stepEngine(SimState_t states, Inertial_t inertial, Environment_t environment)
{
	rotateWind(states, inertial, environment);
	updateRadPS(states, inertial, environment);
	rotateProp();
	getForce(states, inertial, environment);
	getTorque(states, inertial, environment);
	rotateForce();
	rotateTorque(inertial);
}


// Convert the relateive wind from body axes to propeller axes
void Propulsion::rotateWind(SimState_t states, Inertial_t inertial, Environment_t environment)
{
	body_to_mount = Eigen::Translation<double, 3>(CGOffset)*euler2quat(mountOrientation);
	body_to_mount_rot = Eigen::Translation<double, 3>(Vector3d::Zero())*euler2quat(mountOrientation);

	// Construct transformation to apply gimbal movement. Gimbal rotation MUST be aligned with the resulting z-axis!
	mount_to_gimbal = Eigen::Translation<double, 3>(Vector3d::Zero())*euler2quat(Vector3d(0, 0, inputGimbal));
	mount_to_gimbal_rot = Eigen::Translation<double, 3>(Vector3d::Zero())*euler2quat(Vector3d(0, 0, inputGimbal));

	// Transform the relative wind from body axes to propeller axes
	Vector3d relWind = body_to_mount_rot*mount_to_gimbal_rot*(states.velocity.linear-environment.wind);
	Vector3d airdata = getAirData(relWind);
	normalWind = airdata.x();
	// Airdata airdata;
	// airdata.calcAirData(states.velocity.linear, environment.wind);
	// normalWind = airdata.relWind.x();

	if (!std::isfinite(normalWind)) {throw runtime_error("propulsion.cpp: NaN value in normalWind"); }
	if (std::fabs(normalWind)>1e+160) {throw runtime_error("propulsion.cpp/rotateWind: normalWind over 1e+160");}
}

void Propulsion::rotateProp() // Update propeller angle
{
	if (!std::isfinite(omega)) {throw runtime_error("propulsion.cpp: non-finite omega value");}
	theta += omega*dt;
	if (theta > 2.0*M_PI) theta -= 2*M_PI;
	if (theta < 0.0) theta += 2*M_PI;

	gimbal_to_prop = Eigen::Translation<double, 3>(Vector3d::Zero()) * euler2quat(Vector3d(0.0, theta, 0.0));
	gimbal_to_prop_rot = Eigen::Translation<double, 3>(Vector3d::Zero()) * euler2quat(Vector3d(0.0, theta, 0.0));

	body_to_prop = body_to_mount * (mount_to_gimbal * gimbal_to_prop);
	body_to_prop_rot = body_to_mount_rot * (mount_to_gimbal_rot * gimbal_to_prop_rot);
}

 // Convert the resulting force to the body axes
void Propulsion::rotateForce()
{
	wrenchProp.force = body_to_prop_rot.inverse() * wrenchProp.force;
}

// Convert the resulting torque to the body axes
void Propulsion::rotateTorque(Inertial_t inertial)
{
	wrenchProp.torque = body_to_prop_rot.inverse() * wrenchProp.torque;

	// Convert the torque from the motor frame to the body frame
	// TODO: I don't understand why this is true. I may remove it in the future
	double x, y, z;
	double ratio = inertial.J(0,0) / (inertial.J(0,0) + inertial.mass * CGOffset.x()*CGOffset.x());
	x =  ratio * wrenchProp.torque.x();
	ratio = inertial.J(1,1) / (inertial.J(1,1) + inertial.mass * CGOffset.y()*CGOffset.y());
	y =  ratio * wrenchProp.torque.y();
	ratio = inertial.J(2,2) / (inertial.J(2,2) + inertial.mass * CGOffset.z()*CGOffset.z());
	z =  ratio * wrenchProp.torque.z();
	wrenchProp.torque = Vector3d(x, y, z);

	// Add torque to to force misalignment with CG
	// r x F, where r is the distance from CoG to CoL
	// Will potentially add the following code in the future, to support shift of CoG mid-flight
	// XmlRpc::XmlRpcValue list;
	// int i;
	// if(!ros::param::getCached("motor/CGOffset", list)) {ROS_FATAL("Invalid parameters for -/motor/CGOffset- in param server!"); ros::shutdown();}
	// for (i = 0; i < list.size(); ++i) {
	// 	ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	// 	CGOffset[i]=list[i];
	// }

	wrenchProp.torque = wrenchProp.torque + CGOffset.cross(wrenchProp.force);
}

#include "no_engine.cpp"

#include "beard_engine.cpp"

#include "piston_engine.cpp"

#include "electric_engine.cpp"


// Build engine model
Propulsion * buildPropulsion(YAML::Node propConfig, YAML::Node worldConfig)
{
	int motorType;
	getParameter(propConfig, "motorType", motorType);
	std::cout<< "building engine model: ";
	switch (motorType)
	{
		case 0:
			std::cout << "selecting no engine" << std::endl;
			return new NoEngine(propConfig, worldConfig);
		case 1:
			std::cout << "selecting Beard engine" << std::endl;
			return new EngBeard(propConfig, worldConfig);
		case 2:
			std::cout << "selecting piston engine" << std::endl;
			return new PistonEng(propConfig, worldConfig);
		case 3:
			std::cout << "selecting electric engine" << std::endl;
			return new ElectricEng(propConfig, worldConfig);
		default:
			throw runtime_error("Error while constructing motor");
			break;
	}
}
