#include <time.h>

#include <last_letter_lib/math_utils.hpp>
#include <last_letter_lib/prog_utils.hpp>

#include <last_letter_lib/environment.hpp>

///////////////////
// Environment Class
// Based on Aerocalc: http://www.kilohotel.com/python/aerocalc/
// Valid up to 11km
//////////////////

using namespace std;
using namespace last_letter_lib::programming_utils;

namespace last_letter_lib
{

	ostringstream oss;

	/////////////
	// Constructor
	EnvironmentModel::EnvironmentModel(ParameterManager envConfig, ParameterManager worldConfig)
	{
		grav0 = uav_utils::Geoid::EARTH_grav;
		readParametersWorld(worldConfig);
		readParametersEnvironment(envConfig);
	}

	//////////////////////////////////////
	// Read world configuration parameters
	void EnvironmentModel::readParametersWorld(ParameterManager worldConfig)
	{
		dt = worldConfig.get<double>("deltaT");
	}

	/////////////////////////////////////////////
	// Read environment configuration parameters
	void EnvironmentModel::readParametersEnvironment(ParameterManager envConfig)
	{
		allowTurbulence = envConfig.get<bool>("Dryden/use");

		// initialize atmosphere stuff
		T0 = envConfig.get<double>("groundTemp");
		T0 += 274.15;
		P0 = envConfig.get<double>("groundPres");
		Rho0 = envConfig.get<double>("rho");

		// Initialize bias wind engine
		windRef = envConfig.get<double>("windRef");
		windRefAlt = envConfig.get<double>("windRefAlt");
		windDir = envConfig.get<double>("windDir");
		surfSmooth = envConfig.get<double>("surfSmooth");
		windDir = windDir * M_PI / 180; // convert to rad
		kwind = windRef / pow(windRefAlt, surfSmooth);
		// Initialize turbulence engine
		Lu = envConfig.get<double>("Dryden/Lu");
		Lw = envConfig.get<double>("Dryden/Lw");
		sigmau = envConfig.get<double>("Dryden/sigmau");
		sigmaw = envConfig.get<double>("Dryden/sigmaw");
		windDistU = 0;
		for (int i = 0; i < 2; i++)
		{
			windDistV[i] = 0;
			windDistW[i] = 0;
		}
		bool randomize_seed;
		randomize_seed = envConfig.get<bool>("Dryden/randomizeSeed");
		if (randomize_seed)
		{
			srand(time(NULL)); // Initialize the random number generator, if needed
		}
	}

	////////////////////////////////////////////////
	// Read input states and publish environment data
	void EnvironmentModel::calcEnvironment(const uav_utils::SimState_t InpStates)
	{
		states = InpStates;
		calcTemp(); // Run 1st
		calcGrav();
		calcWind();
		calcDens();
		calcPres();
	}

	/////////////////////////////////////
	// Calculate wind element in NED axes
	void EnvironmentModel::calcWind()
	{
		Vector3d relAirVelocity;
		double Va, input, temp[2];

		// calculate bias wind in Earth frame (0-direction is North)
		//  Wind comes FROM its designated direction
		double x, y, z;
		x = -cos(windDir) * kwind * pow(abs(states.geoid.altitude) + 0.001, surfSmooth);
		y = -sin(windDir) * kwind * pow(abs(states.geoid.altitude) + 0.001, surfSmooth); // abs is used to avoid exp(x,0) which may return nan
		z = 0;
		wind = Vector3d(x, y, z);
		if (wind.hasNaN())
		{
			oss << wind;
			throw runtime_error("earth wind NAN in environment object: " + oss.str());
		}
		environment.wind = wind;

		//////////////////////////
		// Calculate turbulent wind

		// Calculate airspeed

		// Read vehicle orientation quaternion.
		// Vehicle quaternion refers to the Body-to-Earth rotation
		Quaterniond q_eb = states.pose.orientation.conjugate();
		relAirVelocity = q_eb * (states.velocity.linear - wind); // Velocity vector in body frame
		Va = relAirVelocity.norm();

		if (allowTurbulence)
		{
			input = (((double)rand()) / (RAND_MAX)-0.5); // turbulence u-component

			windDistU = windDistU * (1 - Va / Lu * dt) + sigmau * sqrt(2 * Va / (M_PI * Lu)) * dt * input;

			input = (((double)rand()) / (RAND_MAX)-0.5); // turbulence v-component
			temp[0] = windDistV[0];
			temp[1] = windDistV[1];
			windDistV[1] = -pow(Va / Lu, 2) * dt * temp[0] + temp[1] + sigmau * sqrt(3 * Va / (M_PI * Lu)) * Va / (sqrt(3) * Lu) * dt * input;
			windDistV[0] = (1.0 - 2.0 * Va / Lu * dt) * temp[0] + dt * temp[1] + sigmau * sqrt(3 * Va / (M_PI * Lu)) * dt * input;

			input = ((double)rand() / (RAND_MAX)-0.5); // turbulence w-component
			temp[0] = windDistW[0];
			temp[1] = windDistW[1];
			windDistW[1] = -pow(Va / Lw, 2) * dt * temp[0] + temp[1] + sigmaw * sqrt(3 * Va / (M_PI * Lw)) * Va / (sqrt(3) * Lw) * dt * input;
			windDistW[0] = (1.0 - 2.0 * Va / Lw * dt) * temp[0] + dt * temp[1] + sigmaw * sqrt(3 * Va / (M_PI * Lw)) * dt * input;
		}
		if (isnan(windDistU) || isnan(windDistV[0]) || isnan(windDistW[0]))
		{
			oss << windDistU << " " << windDistV[0] << " " << windDistW[0];
			throw runtime_error("turbulence NAN in environmentNode " + oss.str());
		}

		Vector3d disturbance(windDistU, windDistV[0], windDistW[0]);

		environment.wind += q_eb.conjugate() * disturbance; // Add turbulence

		if (environment.wind.hasNaN())
		{
			oss << environment.wind;
			throw runtime_error("body wind NAN in environmentNode!" + oss.str());
		}
	}

	///////////////////////
	// Calculate air density
	void EnvironmentModel::calcDens()
	{
		double Hb = 0, Tb = T0, Pb = P0, L = L0;
		double alt2pressRatio = (Pb / P0) * pow(1 - (L / Tb) * (states.geoid.altitude / 1000.0 - Hb), ((1000.0 * grav0) / (Rd * L))); // Corrected to 1 - (L/...)
		double alt2tempRatio = environment.temperature / T0;
		double density = Rho0 * alt2pressRatio / alt2tempRatio;
		environment.density = density;
	}

	///////////////////////////////
	// Calculate barometric pressure
	void EnvironmentModel::calcPres()
	{
		double pressure;
		double Hb = 0, Tb = T0, Pb = P0, L = L0;
		pressure = Pb * pow(1 - (L / Tb) * (states.geoid.altitude / 1000.0 - Hb), ((1000.0 * grav0) / (Rd * L))); // Corrected to 1 - (L/...)
		environment.pressure = pressure;
	}

	///////////////////////
	// Calculate temperature
	void EnvironmentModel::calcTemp()
	{
		environment.temperature = T0 + states.geoid.altitude / 1000.0 * L0;
	}

	///////////////////
	// Calculate gravity
	void EnvironmentModel::calcGrav()
	{
		double slat2 = pow(sin(M_PI / 180 * states.geoid.latitude), 2);
		double Re2 = pow(uav_utils::Geoid::WGS84_Ra, 2);

		grav0 = uav_utils::Geoid::EARTH_grav * (1.0 + 0.00193185138639 * slat2) / sqrt(1.0 - 0.00669437999013 * slat2);
		double gravity = grav0 * (1.0 - grav_temp * states.geoid.altitude + 3.0 * (pow(states.geoid.altitude, 2) / Re2));
		if (isnan(gravity))
		{
			throw runtime_error("environment.cpp: gravity is NaN");
		}
		environment.gravity = gravity;
	}
} // namespace last_letter_lib
