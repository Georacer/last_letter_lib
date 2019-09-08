#ifndef UAV_UTILS_
#define UAV_UTILS_


#include <cstdio>
#include <cmath>
#include <Eigen/Eigen>
#include <random>

using Eigen::Quaterniond;
using Eigen::Vector3d;


//////////////////
// Data Structures
//////////////////

struct Geoid {
	constexpr static double WGS84_Ra = 6378137.0; // Earth ellipsoid semi-major axis (alpha);
	constexpr static double EARTH_flattening = 0.003352811;
	constexpr static double WGS84_e2 = 0.006694380022901;
	constexpr static double EARTH_Omega = 7.292115e-5;
	constexpr static double EARTH_grav = 9.7803267714;
	double latitude;
	double longitude;
	double altitude;
	Vector3d velocity; // in m/s North, East, Up
};

struct Pose {
	Pose(): position(Vector3d::Zero()), orientation(Quaterniond::Identity()) {}
	Vector3d position;
	Quaterniond orientation;
};

struct Twist {
	Twist(): linear(Vector3d::Zero()), angular(Vector3d::Zero()) {}
	Vector3d linear;
	Vector3d angular;
};

struct Wrench_t {
	Wrench_t(): force(Vector3d::Zero()), torque(Vector3d::Zero()) {}
	Vector3d force;
	Vector3d torque;
};

struct WrenchSum_t {
	Wrench_t wrenchGrav;
	Wrench_t wrenchAero;
	Wrench_t wrenchProp;
	Wrench_t wrenchGround;
	Wrench_t wrenchExternal; // For any unmodelled or independed external disturbances
};

struct Accelerations {
	Accelerations(): linear(Vector3d::Zero()), angular(Vector3d::Zero()) {}
	Vector3d linear;
	Vector3d angular;
};

struct SimState_t {
	Geoid geoid;
	Pose pose; // NED
	Twist velocity; // body
	Accelerations acceleration; // body
	// double rotorspeed[4]; // in rad/s
	std::array<double, 4> rotorspeed; // in rad/s
	// TODO: a fixed number of 4 rotors is restrictive
};

struct Input_t {
	std::array<double, 12> value; // -1 - 1 (or 0 - 1) ranged
};

struct InputPwm_t {
	std::array<uint16_t, 12> value; // PWM microseconds, 1000-2000 typical range
};

struct Inertial_t {
	double mass;
	Eigen::Matrix<double, 3, 3, Eigen::RowMajor> J, Jinv;
};

// Air data class declaration
class Airdata
{
	public:
	Airdata();
	~Airdata();
	Vector3d relWind; // relative wind vector elements
	// double u_r, v_r, w_r; // relative wind vector elements
	double airspeed; // relative airspeed
	double alpha; // angle of attach
	double beta; // angle of sideslip
	void calcAirData(Vector3d velBody, Vector3d velWind);
};

//////////////
// Controllers
//////////////

class PID
{
	private:
	public:
	///////////
	//Variables
	double P, I, D, satU, satL, Ts, Tau, trim;
	double Iterm, Iprev, Eprev, Uprev, Dprev;
	double output;
	///////////
	//Functions

	//Constructor
	PID (double Pi, double Ii, double Di, double satUi, double satLi, double trim, double Tsi, double Ni);

	//Destructor
	~PID ();

	//Main step
	double step(double error);
	double step(double error, double dt);
	double step(double error, double dt, double derivative);
	void init(void);
};

class APID
{
public:
	///////////
	//Variables
	double P, I, D, satU, satL, Ts, Tau, trim;
	double Pinit, Iinit, Dinit;
	double Iterm, Iprev, Eprev, Uprev, Dprev;
	double Ierror, bumplessI1, bumplessI2, trErr;
	double output;
	///////////
	//Functions

	//Constructor
	APID (double Pi, double Ii, double Di, double satUi, double satLi, double trim, double Tsi, double Ni);

	//Destructor
	~APID ();

	//Main step
	double step(double error, bool track, double trInput);
	void init(void);
};

/////////////////////////////
// WGS84 utility functions //
/////////////////////////////

/**
 * [WGS84_RN Calculate the curvature of the Earth in the prime vertical
 * @param  lat The latitude in degrees
 * @return     RN - radius corresponding to curvature in meters
 */
double WGS84_RN(double lat);

/**
 * [WGS84_RN Calculate the curvature of the Earth in the meridian
 * @param  lat The latitude in degrees
 * @return     RM - radius corresponding to curvature in meters
 */
double WGS84_RM(double lat);


//////////////////////////////
// PPM and PWM functionalities
//////////////////////////////

double PwmToHalfRange(uint16_t pwmValue);
// Convert a 1000-2000 us value to 0-1 range

double PwmToFullRange(uint16_t pwmValue);
// Convert a 1000-2000 us value to -1-1 range

uint16_t HalfRangeToPwm(double signal);
// Convert a 0-1 range to 1000-2000 us range

uint16_t FullRangeToPwm(double signal);
// Convert a -1-1 range to 1000-2000 us range


//////////////////////////
// Miscellaneous Utilities
//////////////////////////

Vector3d getAirData (Vector3d speeds);


#endif