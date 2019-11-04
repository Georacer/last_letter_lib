#ifndef TRIMMER_
#define TRIMMER_

#include <Eigen/Eigen>
#include <nlopt.hpp>

#include "uav_model.hpp"

using namespace std;
using Eigen::Vector3d;
using Eigen::Quaterniond;

struct State_t {
    Vector3d position;
    Vector3d euler;
    Vector3d linearVel;
    Vector3d angularVel;
};

struct TrimTrajectoryParameters_t {
    double Va;
    double Gamma;
    double R;
};

// Input parameters which serve as independent variables defining the trim state
struct TrimStateParameters_t {
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

class TrimmerState
{
    public:

    UavModel * uav;
    uint funCallCount = 0;
    nlopt::opt opt; // The nlopt optimizer object
    OptimResult_t result;
    vector<double> initState;

    TrimState_t trimState;
    vector<double> trimInput;

    TrimTrajectoryParameters_t targetTrajectory;

    TrimmerState(const string uavName);
    ~TrimmerState();
    void setInitState(vector<double>);
    void resetFunCallCount();
    TrimState_t calcTrimState(const TrimTrajectoryParameters_t);
    double calcCost(const SimState_t state, const Derivatives_t stateDer, Eigen::Vector4d input);
    double costWrapper(const vector<double> &u, vector<double> &grad);
    static double objFunWrapper(const vector<double> &u, vector<double> &grad, void *trimmerObjPtr);
    OptimResult_t findTrimState(const TrimTrajectoryParameters_t);
    void pyFindTrimState(double * trimParamArray, double * result);
    string printOptimalResult(bool verbose=false);

};

class TrimmerInput
{
    public:
    UavModel * uav;
    uint funCallCount = 0;
    nlopt::opt opt; // The nlopt optimizer object
    OptimResult_t result;
    vector<double> initInput;

    TrimState_t trimState;

    TrimmerInput(const string uavName);
    ~TrimmerInput();
    void setInitInput(vector<double>);
    void resetFunCallCount();
    TrimState_t calcTrimState(const TrimStateParameters_t);
    double calcCost(const Derivatives_t stateDer, Eigen::Vector4d input);
    double costWrapper(const vector<double> &u, vector<double> &grad, TrimState_t trimState);
    static double objFunWrapper(const vector<double> &u, vector<double> &grad, void *trimmerObjPtr);
    OptimResult_t findTrimInput(const TrimStateParameters_t);
    void pyFindTrimInput(double * trimParamArray, double * result);
    string printOptimalResult(bool verbose=false);
};

SimState_t convertState4ll(const State_t p_state);
Input_t convertInput4ll(const vector<double> &u);

#endif