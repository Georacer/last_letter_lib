#include "math_utils.hpp"

using Eigen::Quaterniond;
using Eigen::Vector3d;

void quat_normalize (Quaterniond *q)
{
  q->normalize();
}

void quat_normal_wcomplete (Quaterniond *q)
{
  double x = q->x(), y = q->y(), z = q->z();
  double w = sqrt (1.0 - (x*x + y*y + z*z));

  q = new Quaterniond(w, x, y, z);
}

void quat_equiv_wpos_get (Quaterniond *q)
{
  double x, y, z, w;
  if (q -> w() < 0.0)
    {
      w = -(q->w());
      x = -(q->x()); y = -(q->y()); z = -(q->z());
    }
  q = new Quaterniond(w, x, y, z);
}

void quat_equiv_wneg_get (Quaterniond *q)
{
  double x, y, z, w;
  if (q -> w() > 0.0)
    {
      w = -(q->w());
      x = -(q->x()); y = -(q->y()); z = -(q->z());
    }
  q = new Quaterniond(w, x, y, z);
}

void quat_inverse (const Quaterniond q, Quaterniond *q_inv)
{
  *q_inv = q.inverse();
}

void quat_product (const Quaterniond a, const Quaterniond b, Quaterniond *c)
{
  double x, y, z, w;
  w = a.w()*b.w() - (a.x()*b.x() + a.y()*b.y() + a.z()*b.z());
  x = (a.w()*b.x() + a.x()*b.w() + a.y()*b.z() - a.z()*b.y());
  y = (a.w()*b.y() - a.x()*b.z() + a.y()*b.w() + a.z()*b.x());
  z = (a.w()*b.z() + a.x()*b.y() - a.y()*b.x() + a.z()*b.w());
  *c = Quaterniond(w, x, y, z);
}

// Follow the Tait-Brian convention (yaw->pitch-roll)
// Returns the Earth->Body orientation quaternion
// i.e. v_b = q * v_e
Quaterniond euler2quat (const Vector3d euler)
{
  // // Alternate implementation 
  // double x, y, z, w;
  // double cpsi, spsi, ctheta, stheta, cphi, sphi;
  // cpsi = cos (0.5 * euler.z()); spsi = sin (0.5 * euler.z());
  // ctheta = cos (0.5 * euler.y()); stheta = sin (0.5 * euler.y());
  // cphi = cos (0.5 * euler.x()); sphi = sin (0.5 * euler.x());
  // w = cphi*ctheta*cpsi + sphi*stheta*spsi;
  // x = sphi*ctheta*cpsi - cphi*stheta*spsi;
  // y = cphi*stheta*cpsi + sphi*ctheta*spsi;
  // z = cphi*ctheta*spsi - sphi*stheta*cpsi;
  // Quaterniond q = Quaterniond(w, x, y, z);

  Quaterniond q;
  q = Eigen::AngleAxis<double>(-euler.x(), Vector3d::UnitX())
      * Eigen::AngleAxis<double>(-euler.y(), Vector3d::UnitY())
      * Eigen::AngleAxis<double>(-euler.z(), Vector3d::UnitZ());

  return q;
}

void quat2rotmtx (const Quaterniond q, double *rotmtx)
{
  double q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();
  double q00 = q0*q0, q11 = q1*q1, q22 = q2*q2, q33 = q3*q3;
  double q01 = q0*q1, q02 = q0*q2, q03 = q0*q3;
  double q12 = q1*q2, q13 = q1*q3;
  double q23 = q2*q3;
  rotmtx [0] = q00 + q11 - q22 - q33;
  rotmtx [1] = 2.0*(q12 - q03);
  rotmtx [2] = 2.0*(q02 + q13);
  rotmtx [3] = 2.0*(q12 + q03);
  rotmtx [4] = q00 - q11 + q22 - q33;
  rotmtx [5] = 2.0*(q23 - q01);
  rotmtx [6] = 2.0*(q13 - q02);
  rotmtx [7] = 2.0*(q01 + q23);
  rotmtx [8] = q00 - q11 - q22 + q33;
}

Vector3d quat2euler (const Quaterniond q)
// Based on Stevens & Lewis p. 42
// Takes the body-to-Earth quaternion and produces the Euler anlges
// CAUTION! This quaternion is the conjugate of the one returned from quat2euler
{
//	double rotmtx[9];
	Vector3d euler;
//	quat2rotmtx(q, rotmtx);
//	euler.x() = atan2(rotmtx[5],rotmtx[8]);
//	euler.y() = -asin(rotmtx[2]);
//	euler.z() = atan2(rotmtx[1],rotmtx[0]);
//
//	return euler;
	const double tol = 0.499;
	const double q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();
	const double q00 = q0*q0, q11 = q1*q1, q22 = q2*q2, q33 = q3*q3;
	const double q01 = q0*q1, q23 = q2*q3;
	const double q12 = q1*q2, q03 = q0*q3;
	double test = q1*q3 - q0*q2;
	if (test < -tol)
	  {
	    euler(0) = 0.0;
	    euler(1) = 0.5 * M_PI;
	    euler(2) = 2.0 * atan2 (q0, q3);
	    return euler;
	  }
	else if (test > +tol)
	  {
	    euler(0) = 0.0;
	    euler(1) = -0.5 * M_PI;
	    euler(2) = 2.0 * atan2 (q0, q3);
	    return euler;
	  }
	else
	  {
	    euler(0) = atan2 (2.0*(q01 + q23), q00 - q11 - q22 + q33);
	    euler(1) = asin (-2.0*test);
	    euler(2) = atan2 (2.0*(q12 + q03), q00 + q11 - q22 - q33);
	    return euler;
	  }
}

void euler2rotmtx (const Vector3d euler, double *rotmtx)
{
  double cpsi, spsi, ctheta, stheta, cphi, sphi;
  /**** Calculate trigonometric values. ****/
  cpsi = cos (euler.z()); spsi = sin (euler.z());
  ctheta = cos (euler.y()); stheta = sin (euler.y());
  cphi = cos (euler.x()); sphi = sin (euler.x());
  /**** Calculate rotation matrix. ****/
  rotmtx [0] = cpsi * ctheta;
  rotmtx [1] = sphi * cpsi * stheta - cphi * spsi;
  rotmtx [2] = cphi * cpsi * stheta + sphi * spsi;
  rotmtx [3] = spsi * ctheta;
  rotmtx [4] = sphi * spsi * stheta + cphi * cpsi;
  rotmtx [5] = cphi * spsi * stheta - sphi * cpsi;
  rotmtx [6] = -stheta;
  rotmtx [7] = sphi * ctheta;
  rotmtx [8] = cphi * ctheta;
}

void quat_vector3_rotate (Quaterniond q, Vector3d v, Vector3d *res)
{
  Quaterniond vq, q_inv, qa, qb;
  vq = Quaterniond(0.0, v.x(), v.y(), v.z());
  quat_inverse (q, & q_inv);
  quat_product (q, vq, & qa);
  quat_product (qa, q_inv, & qb);
  *res = Vector3d( qb.x(), qb.y(), qb.z());
}

void multi_mtx_mtx_3X3(double *a, double *b, double *res)
{
  int iii, jjj, kkk;
  for (iii = 0; iii < 3; ++ iii) {
    for (jjj = 0; jjj < 3; ++ jjj) {
      res [iii * 3 + jjj] = 0.0;
      for (kkk = 0; kkk < 3; ++ kkk)
	res [iii * 3 + jjj] += a [iii * 3 + kkk] * b [kkk * 3 + jjj];
    }
  }
}

void multi_mtx_mtx_3Xn(double *a, double *b, double *res, int n)
{
  int iii, jjj, kkk;
  for (iii = 0; iii < 3; ++ iii) {
    for (jjj = 0; jjj < n; ++ jjj) {
      res [iii * n + jjj] = 0.0;
      for (kkk = 0; kkk < 3; ++ kkk)
	res [iii * n + jjj] += a [iii * 3 + kkk] * b [kkk*n + jjj];
    }
  }
}


void multi_mtxT_mtx_3X3 (double *a, double *b, double *res)
{
  int iii, jjj, kkk;
  for (iii = 0; iii < 3; ++ iii) {
    for (jjj = 0; jjj < 3; ++ jjj) {
      res [iii * 3 + jjj] = 0.0;
      for (kkk = 0; kkk < 3; ++ kkk)
	res [iii * 3 + jjj] += a [kkk * 3 + iii] * b [kkk* 3 + jjj];
    }
  }
}

void multi_mtxT_mtx_3Xn(double *a, double *b, double *res,int n)
{
  int iii, jjj, kkk;
  for (iii = 0; iii < 3; ++ iii) {
    for (jjj = 0; jjj < n; ++ jjj) {
      res [iii * n + jjj] = 0.0;
      for (kkk = 0; kkk < 3; ++ kkk)
	res [iii * n + jjj] += a [kkk * 3 + iii] * b [kkk*n + jjj];
    }
  }
}

template<typename T>
const T& constrain(const T& x, const T& a, const T& b)
{
  if (x < a)
  {
    return a;
  }
  else if (x > b)
  {
    return b;
  }
  else
  {
    return x;
  }
}
// Explicit instantiations to allow using the library without need of its headers
template const int& constrain<int>(const int& x, const int& a, const int& b);
template const float& constrain<float>(const float& x, const float& a, const float& b);
template const double& constrain<double>(const double& x, const double& a, const double& b);

/**
 * @brief Rescale a number x from the range (in_min, in_max) to the range (out_min, out_max)
 * 
 * @tparam T 
 */
template<typename T>
T map(const T& x, const T& in_min, const T& in_max, const T& out_min, const T& out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
// Explicit instantiations to allow using the library without need of its headers
template int map<int>(const int& x, const int& in_min, const int& in_max, const int& out_min, const int& out_max);
template float map<float>(const float& x, const float& in_min, const float& in_max, const float& out_min, const float& out_max);
template double map<double>(const double& x, const double& in_min, const double& in_max, const double& out_min, const double& out_max);

/**
 * @brief Rescale a number x from the range (in_min, in_max) to the range (out_min, out_max), but always preserve zero
 * This function differs form the simple map by splitting the input/output range around zero and applying the simple
 * map to each interval independently
 * 
 * @tparam T 
 */
template<typename T>
T map_centered(const T& x, const T& in_min, const T& in_max, const T& out_min, const T& out_max)
{
  T zero{0};
  if (x<0)
  {
    return map(x, in_min, zero, out_min, zero);
  }
  else
  {
    return map(x, zero, in_max, zero, out_max);
  }
}
// Explicit instantiations to allow using the library without need of its headers
template float map_centered<float>(const float& x, const float& in_min, const float& in_max, const float& out_min, const float& out_max);
template double map_centered<double>(const double& x, const double& in_min, const double& in_max, const double& out_min, const double& out_max);

double vector3_norm(Vector3d a)
{
  return a.norm();
}

void vector3_cross(Vector3d a, Vector3d b, Vector3d *c)
{
  double x, y, z;
	x = a.y()*b.z() - a.z()*b.y();
	y = a.z()*b.x() - a.x()*b.z();
	z = a.x()*b.y() - a.y()*b.x();
  *c = Vector3d(x, y, z);
}

Vector3d vector3_normalize(Vector3d a)
{
  return a.normalized();
}

// Vector3d operator+(const Vector3d& a, Vector3d& b)
// {
//   return a+b;
// }

// Vector3d operator-(Vector3d& vec)
// {
//   return -vec;
// }

// Vector3d operator-(Vector3d& a,Vector3d& b)
// {
//   return a-b;
// }

Vector3d operator*(const double *mtx, Vector3d& vec)
{
  double x, y, z;
  x = mtx[0]*vec.x()+mtx[1]*vec.y()+mtx[2]*vec.z();
  y = mtx[3]*vec.x()+mtx[4]*vec.y()+mtx[5]*vec.z();
  z = mtx[6]*vec.x()+mtx[7]*vec.y()+mtx[8]*vec.z();
  Vector3d c = Vector3d(x, y, z);

  return c;
}

// Vector3d operator*(const double a, Vector3d& vec)
// {
//   return a*vec;
// }

Vector3d operator/(const double *mtx, Vector3d& vec)
{
  double x, y, z;
  x = mtx[0]*vec.x()+mtx[3]*vec.y()+mtx[6]*vec.z();
  y = mtx[1]*vec.x()+mtx[4]*vec.y()+mtx[7]*vec.z();
  z = mtx[2]*vec.x()+mtx[5]*vec.y()+mtx[8]*vec.z();
  Vector3d c = Vector3d(x, y, z);

  return c;
}

int inverse(double* A, double* Ainv, int N)
{
  int *IPIV = new int[N+1];
  int LWORK = N*N;
  double *WORK = new double[LWORK];
  int INFO;

  memcpy(Ainv,A,LWORK*sizeof(double));


  dgetrf_(&N,&N,Ainv,&N,IPIV,&INFO);
  dgetri_(&N,Ainv,&N,IPIV,WORK,&LWORK,&INFO);

  delete IPIV;
  delete WORK;

  return INFO;
}

////////////////////
// Class Definitions
////////////////////

////////////////////
// Define Polynomial

Polynomial::Polynomial ()
{
};

Polynomial::~Polynomial ()
{
};


//////////////////////
// Define Polynomial1D

// class constructor
Polynomial1D::Polynomial1D (int maxOrder, double * coeffArray) : Polynomial()
{
    int i;
    coeffNo = maxOrder;
    // Create and initialize polynomial coefficients container
    coeffs = (double*)malloc(sizeof(double) * (coeffNo+1));
    for (i=0; i<=coeffNo; i++) {
        coeffs[i] = coeffArray[i];
    }
}

// class destructor
Polynomial1D::~Polynomial1D ()
{
    free(coeffs);
}

// polynomial evaluation
double Polynomial1D::evaluate (double x) {
    int i;
    double sum=0;
    for (i=0; i<=coeffNo; i++) {
        sum += coeffs[i]*pow(x,i);
    }
    return sum;
}


//////////////////////
// Define Polynomial2D

// class constructor
Polynomial2D::Polynomial2D (int maxOrder1, int maxOrder2, double * coeffArray) : Polynomial()
{
    // Attention! maxOrder2 > maxOrder1. If not, swap the variables!
    int i;
    coeffNo1 = maxOrder1;
    coeffNo2 = maxOrder2;
    // Create and initialize polynomial coefficients container
    int arrayLen = (2*maxOrder2 + 2*maxOrder1*maxOrder2 + maxOrder1 - maxOrder1*maxOrder1 + 2)/2;
    coeffs = (double*)malloc(sizeof(double) * arrayLen);
    for (i=0; i<arrayLen; i++) {
        coeffs[i] = coeffArray[i];
    }
}

// class destructor
Polynomial2D::~Polynomial2D ()
{
    free(coeffs);
}

// polynomial evaluation
double Polynomial2D::evaluate (double x, double y) {
    int i, j, k=0;
    double sum=0;
    for (i=0; i<=coeffNo1; i++) {
        for (j=0; j<=coeffNo2; j++) {
            if (i+j<=coeffNo2) {
                sum += coeffs[k]*pow(x,i)*pow(y,j);
                k++;
            }
        }
    }
    // std::cout << "2DPoly: " << x << " " << y << " " << sum << std::endl; // Sanity check output
    return sum;
}


/////////////////
// Define Spline3
// Cubic spline, 4 parameters per variable interval

// class constructor
Spline3::Spline3(int breaksNoIn, double * breaksIn, double * coeffsIn) : Polynomial()
{
    int i;
    breaksNo = breaksNoIn;
    // Create and initialize breaks container
    breaks = (double*)malloc(sizeof(double) * (breaksNo+1));
    for (i=0; i<=breaksNo; i++) {
        breaks[i] = breaksIn[i];
    }
    // Create and initialize polynomial coefficients container
    coeffs = (double*)malloc(sizeof(double) * (breaksNo*4));
    for (i=0; i<(breaksNo*4); i++) {
        coeffs[i] = coeffsIn[i];
    }
}

// class destructor
Spline3::~Spline3()
{
    free(breaks);
    free(coeffs);
}

// polynomial evaluation
double Spline3::evaluate(double x)
{
    int i;
    for (i=0;i<breaksNo;i++) {
    if (x<=breaks[i+1])
      break;
    }
    if (i==breaksNo) i--;
    double delta = x-breaks[i];
    double value = coeffs[4*i]*pow(delta,3) + coeffs[4*i+1]*pow(delta,2) + coeffs[4*i+2]*delta + coeffs[4*i+3];
    return value;
}


///////////////
// 3D Ellipsoid
///////////////


Ellipsoid3D::Ellipsoid3D(const Ellipsoid3DCoefficients_t params)
{
    update_coeffs(params);
}

void Ellipsoid3D::update_coeffs(const Ellipsoid3DCoefficients_t params)
{
    coeffs = params; // Copy over parameters
}

double Ellipsoid3D::evaluate(double x, double y, double z)
{
    double result{0};
    result += coeffs.A*x*x;
    result += coeffs.B*y*y;
    result += coeffs.C*z*z;
    result += 2*coeffs.D*x*y;
    result += 2*coeffs.E*x*z;
    result += 2*coeffs.F*y*z;
    result += 2*coeffs.G*x;
    result += 2*coeffs.H*y;
    result += 2*coeffs.I*z;
    result += coeffs.J;
    return result;
}


///////////////////////
// Define discrTF class
///////////////////////

// Constructor
discrTF::discrTF (double * alphaIn, int alphaOrderIn, double * betaIn, int betaOrderIn)
{
  int i;
  alphaOrder = alphaOrderIn;
  betaOrder = betaOrderIn;

  outputHist = (double*)malloc(sizeof(double) * alphaOrder);
  inputHist = (double*)malloc(sizeof(double) * alphaOrder);

  alpha = (double*)malloc(sizeof(double) * alphaOrder);
  beta = (double*)malloc(sizeof(double) * (betaOrder+1));
  for (i=0; i<alphaOrder; i++){
    alpha[i] = alphaIn[i];
  }
  for (i=0; i<=betaOrder; i++){
    beta[i] = betaIn[i];
  }

  init(0, 0);
}

// Destructor
discrTF::~discrTF ()
{
  free(alpha);
  free(beta);
  free(outputHist);
  free(inputHist);
}

// matrix reset
void discrTF::init(double restInp, double restOut)
{
  int i;
  for (i=0; i<alphaOrder; i++){
    outputHist[i]=restOut;
    inputHist[i]=restInp;
  }
}

// main step
double discrTF::step(double input)
{
  int i;
  double sum = 0;
  for (i=0; i<alphaOrder; i++){ // evaluate past output contribution
    sum -= alpha[i]*outputHist[i];
  }
  for (i=0; i<=betaOrder; i++){ // evaluate input contribution
    sum += beta[i]*inputHist[i];
  }
  for (i=0; i<(alphaOrder-1); i++){ // Slide history vectors back
    outputHist[i] = outputHist[i+1];
    inputHist[i] = inputHist[i+1];
  }
  outputHist[alphaOrder-1] = sum;
  inputHist[alphaOrder-1]  = input;

  return sum;
}

/////////////////////////////////////////
// Check for NaN in various structures //
/////////////////////////////////////////

bool isnan(Vector3d vec)
{
  if (vec.hasNaN()) {
    return true;
  }
  return false;
}

bool isnan(Quaterniond q)
{
  if (q.vec().hasNaN()) {
    return true;
  }
  if (std::isnan(q.w())) {
    return true;
  }
  return false;
}

bool isnan_mtx(double * R, int n)
{
  for (int i=0; i<n; i++){
    if (std::isnan(R[i])){
      return true;
    }
  }
  return false;
}

bool myisfinite(const Vector3d vec)
{
  if (!vec.allFinite()) {
    return false;
  }
  return true;
}

bool myisfinite(const Quaterniond q)
{
  if (!q.vec().allFinite()) {
    return false;
  }
  if (!std::isfinite(q.w())) {
    return false;
  }
  return true;
}

bool myisfnite_mtx(double * R, int n)
{
  for (int i=0; i<n; i++){
    if (!std::isfinite(R[i])){
      return false;
    }
  }
  return true;
}

//////////////////////////////////////////////
// Check if 3x3 matrix is positive definite //
//////////////////////////////////////////////
// Uses Sylvester's Criterion: https://en.wikipedia.org/wiki/Sylvester%27s_criterion
int is_pos_def(double * R)
{
  // Calculate determinant
  double det = R[0]*R[4]*R[8] - R[0]*R[5]*R[7] - R[1]*R[3]*R[8] + R[1]*R[5]*R[6] + R[2]*R[3]*R[7] - R[2]*R[4]*R[6];
  if (det==0) {
    return -1; // Matrix is singular
  }

  // Matrix is not positive definite
  if (R[0]<0) {return -2;}
  if ((R[0]*R[4] - R[1]*R[3])<0) {return -2;}
  if (det<0) {return -2;}

  // Matrix is positive definite
  return 0;
}