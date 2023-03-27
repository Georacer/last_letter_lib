#ifndef UAV_UTILS_
#define UAV_UTILS_

#include <cstdio>
#include <cmath>
#include <unordered_map>
#include <Eigen/Eigen>
#include <random>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

namespace last_letter_lib
{
	namespace uav_utils
	{

		//////////////////
		// Data Structures
		//////////////////

		// Quaternion from ENU inertial frame to NED inertial frame
		const Quaterniond q_enu_ned = Eigen::AngleAxis<double>(M_PI / 2, Vector3d::UnitZ()) *
									  Eigen::AngleAxis<double>(M_PI, Vector3d::UnitX());

		const Quaterniond q_ned_enu = q_enu_ned.conjugate();
		// Quaternion from Gazebo Body frame to Aerospace Body frame
		const Quaterniond q_bg_ba{Eigen::AngleAxis<double>(M_PI, Vector3d::UnitX())};
		const Quaterniond q_ba_bg = q_bg_ba.conjugate();

		struct Geoid
		{
			Geoid() : latitude(0), longitude(0), altitude(0), velocity(Vector3d::Zero()) {}
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

		struct Pose
		{
			Pose() : position(Vector3d::Zero()), orientation(Quaterniond::Identity()) {}
			Vector3d position;
			Quaterniond orientation;
		};

		struct Inertial
		{
			double mass{0};
			Matrix3d tensor{Matrix3d::Zero()};
		};

		struct Twist
		{
			Twist() : linear(Vector3d::Zero()), angular(Vector3d::Zero()) {}
			Vector3d linear;
			Vector3d angular;
		};

		struct Wrench_t
		{
			Wrench_t() : force(Vector3d::Zero()), torque(Vector3d::Zero()) {}
			Vector3d force;
			Vector3d torque;
		};

		typedef std::unordered_map<std::string, Wrench_t> LinkWrenchMap_t;

		struct WrenchSum_t
		{
			Wrench_t wrenchGrav;
			Wrench_t wrenchAero;
			Wrench_t wrenchProp;
			Wrench_t wrenchGround;
			Wrench_t wrenchExternal; // For any unmodelled or independed external disturbances
		};

		struct Accelerations
		{
			Accelerations() : linear(Vector3d::Zero()), angular(Vector3d::Zero()) {}
			Vector3d linear;
			Vector3d angular;
		};

		struct SimState_t
		{
			SimState_t() : rotorspeed(4, 0.01) {}
			Geoid geoid;
			Pose pose;						// NED
			Twist velocity;					// NED
			Accelerations acceleration;		// NED
			std::vector<double> rotorspeed; // in rad/s
		};

		typedef std::unordered_map<std::string, SimState_t> LinkStateMap_t;

		struct Input_t
		{
			Input_t() : value(12, 0) {}
			std::vector<double> value; // -1 - 1 (or 0 - 1) ranged
		};

		struct InputPwm_t
		{
			InputPwm_t() : value(12, 1000) {}
			std::vector<uint16_t> value; // PWM microseconds, 1000-2000 typical range
		};

		struct Inertial_t
		{
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
			double alpha;	 // angle of attach
			double beta;	 // angle of sideslip
			void calcAirData(Vector3d velBody, Vector3d velWind);
		};

		Vector3d getAirData(Vector3d speeds);
		Vector3d getVelocityFromAirdata(Vector3d airdata);

		////////////////////////////
		// Kinematic Transformations
		////////////////////////////

		Vector3d getEulerDerivatives(Vector3d euler, Vector3d rates);
		Vector3d getAngularRatesFromEulerDerivatives(Vector3d euler, Vector3d eulerDot);

		//////////////
		// Controllers
		//////////////

		class PID
		{
		private:
		public:
			///////////
			// Variables
			double P, I, D, satU, satL, Ts, Tau, trim;
			double Iterm, Iprev, Eprev, Uprev, Dprev;
			double output;
			///////////
			// Functions

			// Constructor
			PID(double Pi, double Ii, double Di, double satUi, double satLi, double trim, double Tsi, double Ni);

			// Destructor
			~PID();

			// Main step
			double step(double error);
			double step(double error, double dt);
			double step(double error, double dt, double derivative);
			void init(void);
		};

		class APID
		{
		public:
			///////////
			// Variables
			double P, I, D, satU, satL, Ts, Tau, trim;
			double Pinit, Iinit, Dinit;
			double Iterm, Iprev, Eprev, Uprev, Dprev;
			double Ierror, bumplessI1, bumplessI2, trErr;
			double output;
			///////////
			// Functions

			// Constructor
			APID(double Pi, double Ii, double Di, double satUi, double satLi, double trim, double Tsi, double Ni);

			// Destructor
			~APID();

			// Main step
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
		 * [WGS84_RM Calculate the curvature of the Earth in the meridian
		 * @param  lat The latitude in degrees
		 * @return     RM - radius corresponding to curvature in meters
		 */
		double WGS84_RM(double lat);

		/////////////////////////////
		// Other navigation functions
		/////////////////////////////

		/**
		 * @brief Get latitude and longitude coordinates fromlocal position.
		 * @details Taken from https://github.com/PX4/PX4-SITL_gazebo/blob/97106007eb5c934b902a5329afb55b45e94d5063/include/common.h
		 * Makes the small-angle assumption and disregards altitude.
		 *
		 * @param pos_north Local position North
		 * @param pos_east Local position East
		 * @param lat_home Home (initialization) latitude, in degrees
		 * @param lon_home Home (initialization) longitude, in degrees
		 * @return std::pair<double, double> The new (lat, lon) coordinates, in degrees
		 */
		std::pair<double, double> reproject(const double pos_north,
											const double pos_east,
											const double lat_home,
											const double lon_home);

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
	} // namespace uav_utils
} // namespace last_letter_lib

#endif
