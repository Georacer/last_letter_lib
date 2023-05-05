#ifndef ENVIRONMENT_
#define ENVIRONMENT_

#include <cstdlib>
#include <math.h>
#include <iostream>

#include <Eigen/Eigen>
#include "yaml-cpp/yaml.h"

#include <last_letter_lib/prog_utils.hpp>
#include <last_letter_lib/uav_utils.hpp>

#define E2_EARTH Geoid::WGS84_e2
#define RP_EARTH Geoid::WGS84_Ra * (1.0 - E2_EARTH)

#define grav_const 3.986004418e14
#define grav_temp (2.0 / Geoid::WGS84_Ra) * (1.0 + Geoid::EARTH_flattening + (Geoid::EARTH_Omega * Geoid::EARTH_Omega) * (Geoid::WGS84_Ra * Geoid::WGS84_Ra) * RP_EARTH / grav_const)

#define Rd 287.05307 // Gas constant for dry air, J/kg K
#define L0 -6.5		 // Temperature lapse rate, at sea level deg K/km

using namespace std;
using namespace last_letter_lib::programming_utils;
using namespace last_letter_lib::uav_utils;
using Eigen::Vector3d;

namespace last_letter_lib
{
	////////////////////
	// Type Declarations
	////////////////////

	struct Environment_t
	{
		Vector3d wind;		// Wind velocity vector in in m/s, typically world frame
		double density;		// in kg/m^3
		double pressure;	// in mBar
		double temperature; // in Kelvin
		double gravity;		// in m/s^2
	};

	/////////
	// Classes
	/////////

	class EnvironmentModel
	{
	public:
		Environment_t environment;
		SimState_t states;
		double dt, simRate;
		double allowTurbulence;

		// conditions starting at sea level, in a region with temperature gradient
		double T0;	  // Temperature at sea level, degrees K
		double P0;	  // Pressure at sea level, in HG
		double Rho0;  // Density at sea level, kg/m**3
		double grav0; // Surface earth gravity
		double windRef, windRefAlt, windDir, surfSmooth, kwind;
		Vector3d wind;
		double Lu, Lw, sigmau, sigmaw;
		double windDistU;
		double windDistV[2], windDistW[2];

		/////////////
		// Constructor
		EnvironmentModel(ParameterManager envConfig, ParameterManager worldConfig);

		void calcEnvironment(const SimState_t InpStates);

		void calcWind();

		void calcDens();

		void calcPres();

		void calcTemp();

		void calcGrav();

		void readParametersWorld(ParameterManager worldConfig);

		void readParametersEnvironment(ParameterManager envConfig);
	};
} // namespace last_letter_lib

#endif
