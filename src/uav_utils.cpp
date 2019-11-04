#include "uav_utils.hpp"

using Eigen::Quaterniond;
using Eigen::Vector3d;


//////////////////////////
// Define Airdata class
//////////////////////////

//Class Constructor
Airdata::Airdata()
{
	relWind = Vector3d::Zero();
	airspeed = 0;
	alpha = 0;
	beta = 0;
}

//Class Destructor
Airdata::~Airdata()
{
}

//Caclulate airspeed and aerodynamics angles
void Airdata::calcAirData(Vector3d velBody, Vector3d velWind)
{
	// Calculate relative airspeed
	// geometry_msgs::Vector3 rel_velocity = parentObj->states.velocity.linear - parentObj->environment.wind;
	// u_r = rel_velocity.x;
	// v_r = rel_velocity.y;
	// w_r = rel_velocity.z;
	relWind = velBody - velWind;
	Vector3d airdata = getAirData(relWind);
	airspeed = airdata.x();
	alpha = airdata.y();
	beta = airdata.z();
}

/////////////////////////////////////////
// Aerodynamc angles/ airspeed calculation
Vector3d getAirData (Vector3d speeds)
{
	double u = speeds.x();
	double v = speeds.y();
	double w = speeds.z();

	double airspeed = sqrt(pow(u,2)+pow(v,2)+pow(w,2));
	double alpha = atan2(w,u);
	double beta = 0;
	if (u==0) {
		if (v==0) {
			beta=0;
		}
		else {
			beta=asin(v/abs(v));
		}

	}
	else {
		beta = atan2(v,u);
	}

	Vector3d result = Vector3d(airspeed, alpha, beta);

	return result;
}

// Convert airdata to body-frame velocities
Vector3d getVelocityFromAirdata(Vector3d airdata)
{
	double u, v, w;
	double Va = airdata.x();
	double alpha = airdata.y();
	double beta = airdata.z();
	u = Va*cos(alpha)*cos(beta);
	v = Va*sin(beta);
	w = Va*sin(alpha)*cos(beta);
	return Vector3d(u, v, w);
}

////////////////////////////
// Kinematic Transformations
////////////////////////////

Vector3d getEulerDerivatives(Vector3d euler, Vector3d rates)
{
	double phi = euler(0);
	double theta = euler(1);
	double psi = euler(2);
	double p = rates(0);
	double q = rates(1);
	double r = rates(2);

	double phiDot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
	double thetaDot = cos(phi)*q - sin(phi)*r;
	double psiDot = sin(phi)/cos(theta)*q + cos(phi)/cos(theta)*r;

	return Vector3d(phiDot, thetaDot, psiDot);
}


///////////////////
// Define PID class
///////////////////

	//Constructor
	PID::PID (double Pi, double Ii, double Di, double satUi = std::numeric_limits<double>::max(), double satLi = std::numeric_limits<double>::min(), double trimi = 0.0,
		double Tsi = 1.0/100, double Taui = 0.1)
	{
		init();
		P = Pi;
		I = Ii;
		D = Di;
		satU = satUi;
		satL = satLi;
		trim = trimi;
		Ts = Tsi;
		Tau = Taui;
	}

	void PID::init(void) {
		Iterm = 0;
		Iprev = 0;
		Eprev = 0;
		Uprev = 0;
		Dprev = 0;
	}

	double PID::step(double error)
	{

		Iterm += I*Ts*error;
		double Dterm = 1.0 / (Tau + Ts) * ( Tau * Dprev + error - Eprev);
		output = P*error + Iterm + D*Dterm + trim;

//		double Pterm = P*error;
//		Iterm += Ts/2 * (error + Eprev);
//		double Dterm = (2*Tau - Ts)/(2*Tau + Ts) * Dprev + 2/(2*Tau + Ts)*(error - Eprev); //Bilinear transform, not working
//		output = Pterm + I*Iterm + D*Dterm;

		if (output>satU) {
			output = satU;
			Iterm = Iprev;
		}
		if (output<satL) {
			output = satL;
			Iterm = Iprev;
		}
		Iprev = Iterm;
		Eprev = error;
		Uprev = output;
		Dprev = Dterm;
		return output;
	}

	double PID::step(double error, double dt)
	{
		double Pterm = P*error;
		Iterm += I*error*dt;
		double Dterm = D / (D + dt/Tau) * (Uprev + (error - Eprev)/Tau);
		output = Pterm + Iterm + Dterm + trim;
		if (output>satU) {
			output = satU;
			Iterm = Iprev;
		}
		if (output<satL) {
			output = satL;
			Iterm = Iprev;
		}
		Iprev = Iterm;
		Eprev = error;
		Uprev = output;
		return output;
	}

	double PID::step(double error, double dt, double derivative)
	{
		double Pterm = P*error;
		Iterm += I*error*dt;
		double Dterm = D / (D + dt/Tau) * (Uprev + (error - Eprev)/Tau);
		output = Pterm + Iterm + Dterm + trim;
		if (output>satU) {
			output = satU;
			Iterm = Iprev;
		}
		if (output<satL) {
			output = satL;
			Iterm = Iprev;
		}
		Iprev = Iterm;
		Uprev = output;
		return output;
	}

	//Destructor
	PID::~PID ()
	{
	}

///////////////////
// Define APID class
///////////////////

	//Constructor
	APID::APID (double Pi, double Ii, double Di, double satUi = std::numeric_limits<double>::max(), double satLi = std::numeric_limits<double>::min(), double trimi = 0.0,
		double Tsi = 1.0/100, double Taui = 0.1)
	{
		init();
		Pinit = Pi;
		P = Pinit;
		Iinit = Ii;
		I = Iinit;
		Dinit = Di;
		D = Dinit;
		satU = satUi;
		satL = satLi;
		trim = trimi;
		Ts = Tsi;
		Tau = Taui;
	}

	void APID::init(void) {
		Iterm = 0;
		Iprev = 0;
		Eprev = 0;
		Uprev = 0;
		Dprev = 0;
		Ierror = 0;
		bumplessI1 = 0;
		bumplessI2 = 0;
	}

	double APID::step(double error, bool track, double trInput)
	{
		double PGainPrev = P;
		double IGainPrev = I;
		double DGainPrev = I;
		if (!track) {
			Ierror += error*Ts;
			// Ierror = std::max(-100.0, std::min(100.0, Ierror));

			P += (0.000001*error*error - 0.01*P)*Ts;
			I += (1e-7*Ierror*Ierror - 0.5*I)*Ts;
			// I += (0.00001*error*Ierror)*Ts;
			D += (1e-8*pow((error - Eprev)/Ts,2) - 0.05*D)*Ts;
			if (I<Iinit) {
				I = Iinit;
			}
			bumplessI1 = 0;
			bumplessI2 = 0;
		}

		Iterm += I*Ts*error;
		output = P*error + Iterm + D*(error-Eprev)/Ts + trim;
		// output = P*error + Iterm + trim;


		if (!track) {
			if (output>satU) {
				output = satU;
				Iterm = Iprev;
				P = PGainPrev;
				I = IGainPrev;
				D = DGainPrev;
			}
			if (output<satL) {
				output = satL;
				Iterm = Iprev;
				P = PGainPrev;
				I = IGainPrev;
				D = DGainPrev;
			}
		}

		if (track) {
			trErr = 100*(trInput - output);
			bumplessI1 += 25*trErr*Ts;
			bumplessI2 += 2.0*bumplessI1*Ts;
			Iterm += (trErr + bumplessI1 + bumplessI2)*Ts;
			Ierror = 0;
			P = Pinit;
			I = Iinit;
			D = Dinit;
		}

		Iprev = Iterm;
		Eprev = error;
		return output;
	}

	//Destructor
	APID::~APID ()
	{
	}


/////////////////////////////
// WGS84 utility functions //
/////////////////////////////

double WGS84_RN(double lat)
{
	double sfi =sin(lat*M_PI/180);
	return Geoid::WGS84_Ra/sqrt(1-Geoid::WGS84_e2*sfi*sfi);
}

double WGS84_RM(double lat)
{
	double sfi =sin(lat*M_PI/180);
	return Geoid::WGS84_Ra*(1-Geoid::WGS84_e2)/pow(1-Geoid::WGS84_e2*sfi*sfi,1.5);
}


//////////////////////////
// Miscellaneous Utilities
//////////////////////////

//////////////////////////////
// PPM and PWM functionalities

double PwmToHalfRange(uint16_t pwmValue)
// Convert a 1000-2000 us value to 0-1 range
{
    return ((double)pwmValue - 1000) / 1000;
}

double PwmToFullRange(uint16_t pwmValue)
// Convert a 1000-2000 us value to -1-1 range
{
    return ((double)pwmValue - 1500) / 500;
}

uint16_t HalfRangeToPwm(double signal)
// Convert a 0-1 range to 1000-2000 us range
{
    return (uint16_t)(signal * 1000 + 1000);
}

uint16_t FullRangeToPwm(double signal)
// Convert a -1-1 range to 1000-2000 us range
{
    return (uint16_t)(signal * 500 + 1500);
}
