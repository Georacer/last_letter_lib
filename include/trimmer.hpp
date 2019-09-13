#ifndef TRIMMER_
#define TRIMMER_

#include <Eigen/Eigen>
#include <nlopt.hpp>

#include "uav_model.hpp"

using namespace std;
using Eigen::Vector3d;

struct State_t {
    Vector3d position;
    Vector3d euler;
    Vector3d linearVel;
    Vector3d angularVel;
};

// Input parameters which serve as independent variables defining the trim state
struct TrimParameters_t {
    double phi;
    double theta;
    double Va ;
    double alpha;
    double beta;
    double r;
};

struct TrimState_t {
    State_t trimState;
    State_t trimDerivatives;
}; 

struct OptimResult_t {
    vector<double> trimInput;
    double cost;
    int returnCode;
    bool success;
};

class Trimmer
{
    public:
    UavModel * uav;
    uint funCallCount = 0;
    nlopt::opt opt; // The nlopt optimizer object
    OptimResult_t result;
    vector<double> initInput;

    TrimState_t trimState;

    Trimmer(const string uavName);
    ~Trimmer();
    void setInitInput(vector<double>);
    void resetFunCallCount();
    TrimState_t calcTrimState(const TrimParameters_t);
    SimState_t convertState4ll(const State_t p_state);
    Input_t convertInput4ll(const vector<double> &u);
    double calcCost(const Derivatives_t stateDer, Eigen::Vector4d input);
    double costWrapper(const vector<double> &u, vector<double> &grad, TrimState_t trimState);
    static double objFunWrapper(const vector<double> &u, vector<double> &grad, void *trimmerObjPtr);
    OptimResult_t findTrimInput(const TrimParameters_t);
    void pyFindTrimInput(double * trimParamArray, double * result);
    string printOptimalResult();
};

#endif