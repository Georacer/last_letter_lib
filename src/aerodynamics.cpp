
#include "aerodynamics.hpp"
#include "prog_utils.hpp"

using namespace std;

using Eigen::Quaterniond;

//////////////////////////
// Define Aerodynamics class
//////////////////////////

// Aerodynamics::Aerodynamics(ModelPlane * parent, int ID)
// Pass the ID-filtered aerodynamics config YAML::Node
Aerodynamics::Aerodynamics(YAML::Node config)
{
    readParametersAerodynamics(config);
}

Aerodynamics::~Aerodynamics()
{
}

void Aerodynamics::readParametersAerodynamics(YAML::Node config)
{
	vector<double> doubleVect;
	getParameterList(config, "CGOffset", doubleVect);
	CGOffset = Vector3d(doubleVect.data());

	doubleVect.clear();
	getParameterList(config, "mountOrientation", doubleVect);
	mountOrientation = Vector3d(doubleVect.data());

	if (!getParameter(config, "chanAileron", chanAileron, false)) {chanAileron = -1;}
	if (!getParameter(config, "chanElevator", chanElevator, false)) {chanElevator = -1;}
	if (!getParameter(config, "chanRudder", chanRudder, false)) {chanRudder = -1;}
	if (!getParameter(config, "chanGimbal", chanGimbal, false)) {chanGimbal = -1;}

	if (!getParameter(config, "gimbalAngle_max", gimbalAngle_max, false)) {gimbalAngle_max = -1;}
	getParameter(config, "deltaa_max", deltaa_max);
	getParameter(config, "deltae_max", deltae_max);
	getParameter(config, "deltar_max", deltar_max);

	inputAileron = 0.0;
	inputElevator = 0.0;
	inputRudder = 0.0;
	inputGimbal = 0.0;
}

void Aerodynamics::setInput(Input_t input)
{
	//Convert -1 - 1 input range to radians
	if (chanAileron>-1) {inputAileron = deltaa_max * input.value[chanAileron];}
	if (chanElevator>-1) {inputElevator = deltae_max * input.value[chanElevator];}
	if (chanRudder>-1) {inputRudder = deltar_max * input.value[chanRudder];}
	if (chanGimbal>-1) {inputGimbal = gimbalAngle_max * input.value[chanGimbal];}
}

void Aerodynamics::setInputPwm(InputPwm_t p_input)
{
	// Convert PPM to -1 - 1 range
	Input_t input;
	if (chanAileron>-1) {input.value[chanAileron] = PwmToFullRange(p_input.value[chanAileron]);}
	if (chanElevator>-1) {input.value[chanElevator] = PwmToFullRange(p_input.value[chanElevator]);}
	if (chanRudder>-1) {input.value[chanRudder] = PwmToFullRange(p_input.value[chanRudder]);}
	if (chanGimbal>-1) {input.value[chanGimbal] = PwmToFullRange(p_input.value[chanGimbal]);}

	// Call normalized input setter
	setInput(input);
}

// One step in the physics engine
void Aerodynamics::stepDynamics(const SimState_t states, const Inertial_t inertial, const Environment_t environment)
{
	rotateFrame(states, environment);
	getForce(environment);
	getTorque(environment);
	rotateForce();
	rotateTorque(inertial);
}

// Convert the relative wind from body axes to airfoil axes
void Aerodynamics::rotateFrame(SimState_t states, Environment_t environment)
{
	// Construct transformation from body axes to mount frame
	q_bm = euler2quat(mountOrientation);
	body_to_mount = Eigen::Translation<double, 3>(CGOffset)*q_bm.conjugate();
	body_to_mount_rot = Eigen::Translation<double, 3>(Vector3d::Zero())*q_bm.conjugate();

	// Construct transformation to apply gimbal movement. Gimbal rotation MUST be aligned with (be applied on) the resulting z-axis!
	q_mg = euler2quat(Vector3d(0, 0, inputGimbal));
	mount_to_gimbal = Eigen::Translation<double, 3>(Vector3d::Zero())*q_mg.conjugate();
	mount_to_gimbal_rot = Eigen::Translation<double, 3>(Vector3d::Zero())*q_mg.conjugate();

	q_bg = q_mg*q_bm;
	body_to_gimbal = body_to_mount * mount_to_gimbal;
	body_to_gimbal_rot = body_to_mount_rot * mount_to_gimbal_rot;

	// Transform the relative wind from body axes to airfoil axes
	Vector3d relWind = q_mg*q_bm*(states.velocity.linear-environment.wind);
	Vector3d airdata = getAirData(relWind);
	airspeed = airdata.x();
	alpha = airdata.y();
	beta = airdata.z();
	// Airdata airdata;
	// airdata.calcAirData(states.velocity.linear, environment.wind);
	// Calculate the new, relative air data
	// airspeed = airdata.airspeed;
	// alpha = airdata.alpha;
	// beta = airdata.beta;

	if (!std::isfinite(airspeed)) {throw runtime_error("aerodynamicsLib.cpp/rotateWind: NaN value in airspeed");}
	if (std::fabs(airspeed)>1e+160) {throw runtime_error("aerodynamicsLib.cpp/rotateWind: normalWind over 1e+160");}

	// Rotate angular rates from the body frame to the airfoil frame
	relativeRates = q_mg * q_bm * states.velocity.angular;
	p = relativeRates.x();
	q = relativeRates.y();
	r = relativeRates.z();
}

 // Convert the resulting force from the gimbal axes to the body axes
void Aerodynamics::rotateForce()
{
	wrenchAero.force = q_bg.conjugate() * wrenchAero.force;
}

// Convert the resulting torque from the gimbal axes to the body axes
void Aerodynamics::rotateTorque(Inertial_t inertial)
{
	// Convert the torque from the airfoil frame to the body frame
	wrenchAero.torque = q_bg.conjugate() * wrenchAero.torque;

	// Calculate the increased moment of inertia due to off-center torque application
	double x, y, z;
	double ratio = inertial.J(0,0) / (inertial.J(0,0) + inertial.mass * CGOffset.x()*CGOffset.x());
	x =  ratio * wrenchAero.torque.x();
	ratio = inertial.J(1,1) / (inertial.J(1,1) + inertial.mass * CGOffset.y()*CGOffset.y());
	y =  ratio * wrenchAero.torque.y();
	ratio = inertial.J(2,2) / (inertial.J(2,2) + inertial.mass * CGOffset.z()*CGOffset.z());
	z =  ratio * wrenchAero.torque.z();
	wrenchAero.torque = Vector3d(x, y, z);

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

	wrenchAero.torque = wrenchAero.torque + CGOffset.cross(wrenchAero.force);
}

/////////////////////////////
// Define NoAerodynamics class
/////////////////////////////

// Class constructor
NoAerodynamics::NoAerodynamics(YAML::Node config) : Aerodynamics(config)
{
}

// Class destructor
NoAerodynamics::~NoAerodynamics()
{
}

// Force calculation function
void NoAerodynamics::getForce(Environment_t environment)
{
}

// Torque calculation function
void NoAerodynamics::getTorque(Environment_t environment)
{
}

/////////////////////////////
// Define StdLinearAero class
/////////////////////////////

// Class constructor
StdLinearAero::StdLinearAero(YAML::Node config) : Aerodynamics(config)
{
	readParametersAerodynamics(config);
}

// Class destructor
StdLinearAero::~StdLinearAero()
{
}

void StdLinearAero::readParametersAerodynamics(YAML::Node config)
{
	Aerodynamics::readParametersAerodynamics(config);
	// Read aerodynamic coefficients from parameter server
	getParameter(config, "c_lift_q", c_lift_q);
	getParameter(config, "c_lift_deltae", c_lift_deltae);
	getParameter(config, "c_drag_q", c_drag_q);
	getParameter(config, "c_drag_deltae", c_drag_deltae);
	getParameter(config, "c", c);
	getParameter(config, "b", b);
	getParameter(config, "s", s);
	getParameter(config, "c_y_0", c_y_0);
	getParameter(config, "c_y_b", c_y_b);
	getParameter(config, "c_y_p", c_y_p);
	getParameter(config, "c_y_r", c_y_r);
	getParameter(config, "c_y_deltaa", c_y_deltaa);	
	getParameter(config, "c_y_deltar", c_y_deltar);	
	getParameter(config, "c_l_0", c_l_0);
	getParameter(config, "c_l_b", c_l_b);
	getParameter(config, "c_l_p", c_l_p);
	getParameter(config, "c_l_r", c_l_r);
	getParameter(config, "c_l_deltaa", c_l_deltaa);
	getParameter(config, "c_l_deltar", c_l_deltar);
	getParameter(config, "c_m_0", c_m_0);
	getParameter(config, "c_m_a", c_m_a);
	getParameter(config, "c_m_q", c_m_q);
	getParameter(config, "c_m_deltae", c_m_deltae);
	getParameter(config, "c_n_0", c_n_0);
	getParameter(config, "c_n_b", c_n_b);
	getParameter(config, "c_n_p", c_n_p);
	getParameter(config, "c_n_r", c_n_r);
	getParameter(config, "c_n_deltaa", c_n_deltaa);
	getParameter(config, "c_n_deltar", c_n_deltar);
	getParameter(config, "c_drag_p", c_drag_p);
	getParameter(config, "c_lift_0", c_lift_0);
	getParameter(config, "c_lift_a", c_lift_a0);
	getParameter(config, "oswald", oswald);
	getParameter(config, "mcoeff", M);
	getParameter(config, "alpha_stall", alpha0);
}

// Force calculation function
void StdLinearAero::getForce(Environment_t environment)
{
	// Read air density
	rho = environment.density;

	//request lift and drag alpha-coefficients from the corresponding functions
	double c_lift_a = liftCoeff(alpha);
	double c_drag_a = dragCoeff(alpha);

	//convert coefficients to the body frame
	double c_x_a = -c_drag_a*cos(alpha)+c_lift_a*sin(alpha);
	double c_x_q = -c_drag_q*cos(alpha)+c_lift_q*sin(alpha);
	double c_z_a = -c_drag_a*sin(alpha)-c_lift_a*cos(alpha);
	double c_z_q = -c_drag_q*sin(alpha)-c_lift_q*cos(alpha);

	//calculate aerodynamic force
	double qbar = 1.0/2.0*rho*pow(airspeed,2)*s; //Calculate dynamic pressure
	double ax, ay, az;
	if (airspeed==0)
	{
		ax = 0;
		ay = 0;
		az = 0;
	}
	else
	{
		ax = qbar*(c_x_a + c_x_q*c*q/(2*airspeed) - c_drag_deltae*cos(alpha)*fabs(inputElevator) + c_lift_deltae*sin(alpha)*inputElevator);
		// split c_x_deltae to include "abs" term
		ay = qbar*(c_y_0 + c_y_b*beta + c_y_p*b*p/(2*airspeed) + c_y_r*b*r/(2*airspeed) + c_y_deltaa*inputAileron + c_y_deltar*inputRudder);
		az = qbar*(c_z_a + c_z_q*c*q/(2*airspeed) - c_drag_deltae*sin(alpha)*fabs(inputElevator) - c_lift_deltae*cos(alpha)*inputElevator);
		// split c_z_deltae to include "abs" term
	}

	wrenchAero.force = Vector3d(ax, ay, az);
}

// Torque calculation function
void StdLinearAero::getTorque(Environment_t environment)
{
	// Read air density
	rho = environment.density;

	//calculate aerodynamic torque
	double qbar = 1.0/2.0*rho*pow(airspeed,2)*s; //Calculate dynamic pressure
	double la, na, ma;
	if (airspeed==0)
	{
		la = 0;
		ma = 0;
		na = 0;
	}
	else
	{
		la = qbar*b*(c_l_0 + c_l_b*beta + c_l_p*b*p/(2*airspeed) + c_l_r*b*r/(2*airspeed) + c_l_deltaa*inputAileron + c_l_deltar*inputRudder);
		ma = qbar*c*(c_m_0 + c_m_a*alpha + c_m_q*c*q/(2*airspeed) + c_m_deltae*inputElevator);
		na = qbar*b*(c_n_0 + c_n_b*beta + c_n_p*b*p/(2*airspeed) + c_n_r*b*r/(2*airspeed) + c_n_deltaa*inputAileron + c_n_deltar*inputRudder);
	}

	wrenchAero.torque = Vector3d(la, ma, na);

	// Removing torque calculation, because the CG effect is now being taken into account in the 'rotateTorque' function
}

//////////////////////////
//C_lift_alpha calculation
double StdLinearAero::liftCoeff (double alpha)
{
	double sigmoid = ( 1+exp(-M*(alpha-alpha0))+exp(M*(alpha+alpha0)) ) / (1+exp(-M*(alpha-alpha0))) / (1+exp(M*(alpha+alpha0)));
	double linear = (1.0-sigmoid) * (c_lift_0 + c_lift_a0*alpha); //Lift at small AoA
	double flatPlate = sigmoid*(2*copysign(1,alpha)*pow(sin(alpha),2)*cos(alpha)); //Lift beyond stall

	double result  = linear+flatPlate;
	return result;
}

//////////////////////////
//C_drag_alpha calculation
double StdLinearAero::dragCoeff (double alpha)
{
	AR = pow(b,2)/s;
	double c_drag_a = c_drag_p + pow(c_lift_0+c_lift_a0*alpha,2)/(M_PI*oswald*AR);

	return c_drag_a;
}

/////////////////////////
// Define HCUAVAero class
/////////////////////////

// Class constructor
HCUAVAero::HCUAVAero (YAML::Node config) : StdLinearAero(config)
{
	readParametersAerodynamics(config);
}

// Class destructor
HCUAVAero::~HCUAVAero()
{
}

void HCUAVAero::readParametersAerodynamics(YAML::Node config)
{
	StdLinearAero::readParametersAerodynamics(config);

	// Create CLift polynomial
	YAML::Node liftPolyConfig = filterConfig(config, "cLiftPoly");
	liftCoeffPoly =  buildPolynomial(liftPolyConfig);
	// Create CDrag polynomial
	YAML::Node dragPolyConfig = filterConfig(config, "cDragPoly");
	dragCoeffPoly =  buildPolynomial(dragPolyConfig);
}

//////////////////////////
//C_lift_alpha calculation
double HCUAVAero::liftCoeff (double alpha)
{
	return liftCoeffPoly->evaluate(alpha);
}

//////////////////////////
//C_drag_alpha calculation
double HCUAVAero::dragCoeff (double alpha)
{
	return dragCoeffPoly->evaluate(alpha);
}


// Build aerodynamics model
Aerodynamics * buildAerodynamics(YAML::Node config)
{
	int aerodynamicsType;
	getParameter(config, "aerodynamicsType", aerodynamicsType);
	std::cout<< "building aerodynamics model: ";
	switch (aerodynamicsType)
	{
	case 0:
		std::cout << "selecting no aerodynamics model" << std::endl;
		return new NoAerodynamics(config);
	case 1:
		std::cout << "selecting StdLinearAero aerodynamics" << std::endl;
		return new StdLinearAero(config);
	case 2:
		std::cout << "selecting HCUAVAero aerodynamics" << std::endl;
		return new HCUAVAero(config);
	default:
		throw runtime_error("Error while constructing aerodynamics");
		break;
	}
}