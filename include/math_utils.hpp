#ifndef MATH_UTILS_
#define MATH_UTILS_

#include <cstdio>
#include <Eigen/Eigen>
#include <cmath>

#define R_earth 6378137.0
#define f_earth 1.0/298.257223563
#define e_earth sqrt(2.0*f_earth - f_earth*f_earth)

using Eigen::Quaterniond;
using Eigen::Vector3d;

extern "C" {
    // LU decomoposition of a general matrix
    void dgetrf_(int* M, int *N, double* A, int* lda, int* IPIV, int* INFO);

    // generate inverse of a matrix given its LU decomposition
    void dgetri_(int* N, double* A, int* lda, int* IPIV, double* WORK, int* lwork, int* INFO);
}

////////////
// Functions

void quat_normalize (Quaterniond *q);
void quat_normal_wcomplete (Quaterniond *q);
void quat_equiv_wpos_get (Quaterniond *q);
void quat_equiv_wneg_get (Quaterniond *q);
void quat_inverse (const Quaterniond q, Quaterniond *q_inv);
void quat_product (const Quaterniond a, const Quaterniond b, Quaterniond *c);

Quaterniond euler2quat (const Vector3d euler);
void quat2rotmtx (const Quaterniond q, double *rotmtx);
Vector3d quat2euler (const Quaterniond q);
void euler2rotmtx (const Vector3d euler, double *rotmtx);

void quat_vector3_rotate (Quaterniond q, Vector3d v, Vector3d *res);

void multi_mtx_mtx_3X3 (double *a, double *b, double *res);
void multi_mtx_mtx_3Xn(double *a, double *b, double *res,int n);
void multi_mtxT_mtx_3X3 (double *a, double *b, double *res);
void multi_mtxT_mtx_3Xn(double *a, double *b, double *res,int n);
int is_pos_def(double * R); // Check if 3x3 matrix is positive definite

template<typename T>
const T& constrain(const T& x, const T& a, const T& b);
template<typename T>
T map(const T&x, const T& in_min, const T& in_max, const T& out_min, const T& out_max);
template<typename T>
T map_centered(const T&x, const T& in_min, const T& in_max, const T& out_min, const T& out_max);


//////////////////////////
// 3D Vector operations //
//////////////////////////
double vector3_norm(Vector3d a);
void vector3_cross(Vector3d a, Vector3d b, Vector3d *c);
Vector3d vector3_normalize(Vector3d a);

// Vector3d operator+(const Vector3d& a, Vector3d& b);
// Vector3d operator-(Vector3d& vec);
// Vector3d operator-(Vector3d& a,Vector3d& b);
Vector3d operator*(const double *mtx, Vector3d& vec);
// Vector3d operator*(const double a, Vector3d& vec);
Vector3d operator/(const double *mtx, Vector3d& vec);


int inverse(double* A, double* Ainv, int N);


//////////
// Classes

class Polynomial
{
public:
	Polynomial();
	virtual ~Polynomial();
	virtual double evaluate() {return 0;}
	virtual double evaluate(double x) {return 0;}
	virtual double evaluate(double x, double y) {return 0;}
};

class Polynomial1D : public Polynomial
{
public:
	double coeffNo;
	double * coeffs;

	Polynomial1D(int maxOrder, double * coeffArray);
	~Polynomial1D();
	double evaluate(double x);
};

class Polynomial2D : public Polynomial
{
public:
	double coeffNo1, coeffNo2;
	double * coeffs;

	Polynomial2D(int maxOrder1, int maxOrder2, double * coeffArray);
	~Polynomial2D();
	double evaluate(double x, double y);
	// Important notes: maxOrder of variable y must be greater or equal to maxOrder of variable x
	// Pass the coefficient array with the following ordering (eg for maxOrder1=1, maxOrder2=3):
	// [00 01 02 03 10 11 12]
};

class Spline3 : public Polynomial
{
public:
	int breaksNo;
	double * breaks, * coeffs;

	Spline3(int breaksNoIn, double * breaksIn, double * coeffsIn);
	~Spline3();
	double evaluate(double x);
};


// Discrete transfer function implementation
// for strictly proper TFs
class discrTF
{
public:
	///////////
	//Variables
	double * alpha, * beta;
	double * outputHist, * inputHist;
	int alphaOrder, betaOrder;

	///////////
	//Functions

	// Constructor
	discrTF (double * alphaIn, int alphaOrderIn, double * betaIn, int betaOrderIn);

	// Destructor
	~discrTF ();

	// matrix reset
	void init(double restInp, double restOut);

	// main step
	double step(double input);
};

Vector3d getAirData (Vector3d speeds);

void WGS84_NM(double lat,double *NE, double *ME);


/////////////////////////////////////////
// Check for NaN in various structures //
/////////////////////////////////////////
bool isnan(Vector3d vec);
bool isnan(Quaterniond q);
bool isnan_mtx(double * R, int n);

bool myisfinite(const Vector3d vec);
bool myisfinite(const Quaterniond q);
bool myisfinite_mtx(double * R, int n);

///////////////////////////////////////
// Numerical derivative functionalities
///////////////////////////////////////

namespace Eigen
{
template <typename F>
VectorXd derivative_partial_numerical(F funcPtr, VectorXd values)
// Version without parameters
{

double de = 1e-4; // Numerical differentiation step

uint num_inputs = values.size();

VectorXd result = VectorXd(num_inputs);

for (int i=0; i<num_inputs; ++i)
{
	// Setup the argument shift
	VectorXd eps = VectorXd::Zero(num_inputs);
	eps(i) = de;
	VectorXd x_plus = values + eps;
	VectorXd x_minus = values - eps;

	double f_plus, f_minus;

	f_plus = funcPtr(x_plus);
	f_minus = funcPtr(x_minus);

	result(i) = (f_plus - f_minus) / (2 * de);
}

return result;

}

template <typename F>
VectorXd derivative_partial_numerical(F funcPtr, VectorXd values, VectorXd parameters)
// Version with parameters
{

double de = 1e-4; // Numerical differentiation step

uint num_inputs = values.size();

VectorXd result = VectorXd(num_inputs);

for (int i=0; i<num_inputs; ++i)
{
	// Setup the argument shift
	VectorXd eps = VectorXd::Zero(num_inputs);
	eps(i) = de;
	VectorXd x_plus = values + eps;
	VectorXd x_minus = values - eps;

	double f_plus, f_minus;

	f_plus = funcPtr(x_plus, parameters);
	f_minus = funcPtr(x_minus, parameters);

	result(i) = (f_plus - f_minus) / (2 * de);
}

return result;

}

} // End of Eigen namespace



#endif