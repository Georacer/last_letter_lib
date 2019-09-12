#include <math.h>
#include <iomanip>
#include <time.h>
#include <chrono>

#include <Eigen/Eigen>
#include <nlopt.hpp>

#include "uav_model.hpp" // Inlude the last_letter lib main library

using namespace std;
using Eigen::Vector3d;

UavModel * uav;
int optimSteps = 0;

struct State_t {
    Vector3d position;
    Vector3d euler;
    Vector3d linearVel;
    Vector3d angularVel;
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

template<typename T>
string printVector(vector<T> vec, string delimiter=", ")
{
    ostringstream oss;
    for (const auto value: vec)
        oss << value << delimiter;

    return oss.str();
}

string printVector(Vector3d vec3d, string delimiter=", ")
{
    vector<double> vec(3);
    vec[0] = vec3d[0];
    vec[1] = vec3d[1];
    vec[2] = vec3d[2];
    return printVector(vec, delimiter);
}

string printVector(Eigen::Quaterniond quat, string delimiter=", ")
{
    vector<double> vec(4);
    vec[0] = quat.w();
    vec[1] = quat.x();
    vec[2] = quat.y();
    vec[2] = quat.z();
    return printVector(vec, delimiter);
}


// Calculate the full trim state and its derivative given the 6 independent trim variables
TrimState_t calcTrimState(const vector<double> trim_data)
{
    double trim_phi = trim_data[0];
    double trim_theta = trim_data[1];
    double trim_Va = trim_data[2];
    double trim_alpha = trim_data[3];
    double trim_beta = trim_data[4];
    double trim_r = trim_data[5];

    // Calculate dependent state elements
    double k = trim_r / (cos(trim_phi) * cos(trim_theta));
    double trim_p = -k * sin(trim_theta);
    double trim_q = k * sin(trim_phi) * cos(trim_theta);

    TrimState_t trimState;
    trimState.trimState.position = Vector3d(0, 0, -100);
    trimState.trimState.euler = Vector3d(trim_phi, trim_theta, 0);
    Vector3d velocity = getVelocityFromAirdata(Vector3d(trim_Va, trim_alpha, trim_beta));
    trimState.trimState.linearVel = velocity;
    trimState.trimState.angularVel = Vector3d(trim_p, trim_q, trim_r);

    // Calculate derived quantities
    double gamma = trim_theta - trim_alpha;

    trimState.trimDerivatives.position = Vector3d(0, 0, -trim_Va*sin(gamma));
    trimState.trimDerivatives.euler = Vector3d(0, 0, k); // k=Va/R*cos(gamm)
    trimState.trimDerivatives.linearVel = Vector3d::Zero();
    trimState.trimDerivatives.angularVel = Vector3d::Zero();

    return trimState;
}

// Convert State_t to SimState_t for passing to UavModel
SimState_t convertState4ll(const State_t p_state)
{
    SimState_t state;
    state.pose.position = p_state.position;
    state.pose.orientation = euler2quat(p_state.euler).conjugate();
    // state.pose.orientation = euler2quat(p_state.euler);
    state.velocity.linear = p_state.linearVel;
    state.velocity.angular = p_state.angularVel;
    return state;
}

Input_t convertInput4ll(const vector<double> &u)
{
    Input_t input;
    input.value[0] = u[0];
    input.value[1] = u[1];
    input.value[2] = u[2];
    input.value[3] = u[3];
    return input;
}

// Calculate the optimization error cost, based on the optimized input and the corresponding derivative
double calcCost(const Derivatives_t stateDer, Eigen::Vector4d input)
{
    // The cost function should ideally check the u and v components of the linear velocity.
    // But since this model deals with body-frame forces, instead of wind-frame forces (as the Python model/trimmer does)
    // the moments cannot affect alpha and beta. As a result, it is numerically very difficult for thrust input to fix all
    // of the u and w velocities.
    // Thus, for now we leave only airspeed in the juristiction of thrust and hope that alpha and beta can stay close to their
    // trimmed values, since control inputs will stabilize angular velocities
    double derErrorWeight = 1000;
    double inputWeight = 10;

    // double speedDerTerm = stateDer.speedDot.transpose()*derErrorWeight*stateDer.speedDot;
    double speedDerTerm = stateDer.speedDot[0]*derErrorWeight*stateDer.speedDot[0];
    double rateDerTerm = stateDer.rateDot.transpose()*10*derErrorWeight*stateDer.rateDot;
    // The position and orientation derivatives are not taken into account, because they are designed to be optimal
    // and the optimized input vector does not affect them anyway.

    double inputTerm = input.transpose()*inputWeight*input;

    return speedDerTerm + rateDerTerm + inputTerm;
}

// Wrapper to the cost function. Accepts input u under optimization and also passes the trimState parameter to the cost function
double costWrapper(const vector<double> &u, vector<double> &grad, void *trimStatePtr)
{
    ++optimSteps;

    TrimState_t * trimState = reinterpret_cast<TrimState_t *>(trimStatePtr);

    Input_t input = convertInput4ll(u);
    SimState_t state = convertState4ll(trimState->trimState);

    // If your motors have state as well (RPM) you need to set it here again
    uav->setState(state);
    uav->setInput(input);
    uav->step();

    Derivatives_t stateDot = uav->kinematics.stateDot;

    Eigen::Vector4d inputVect;
    inputVect[0] = input.value[0];
    inputVect[1] = input.value[1];
    inputVect[2] = input.value[2];
    inputVect[3] = input.value[3];
    return calcCost(stateDot, inputVect);
}

OptimResult_t sample(nlopt::opt *opt, const TrimState_t &trimState, const vector<double> initInput)
{
    OptimResult_t result;
    result.trimInput = initInput;

    try
    {
        nlopt::result returnCode = opt->optimize(result.trimInput, result.cost);

        result.returnCode = returnCode;
        result.success = true;
    }
    catch (exception &e)
    {
        cout << "nlopt failed: " << e.what() << endl;
        result.success=false;
    }

    return result;
}

int main(int argc, char * argv[])
{
    string uavName = "skywalker_2013";
    ConfigsStruct_t configs = loadModelConfig(uavName);
    uav = new UavModel(configs);

    // nlopt::opt opt(nlopt::LN_COBYLA, 4);
    nlopt::opt opt(nlopt::LN_BOBYQA, 4);
    // nlopt::opt opt(nlopt::LN_PRAXIS, 4); // Known working

    std::vector<double> lb(4, -1);
    lb[2] = 0;
    opt.set_lower_bounds(lb);

    std::vector<double> ub(4, 1);
    opt.set_upper_bounds(ub);

    // Set initial input value
    vector<double> initInput(4);
    initInput[0] = 0;
    initInput[1] = 0;
    initInput[2] = 0.5;
    initInput[3] = 0;

    // Set a trim point
    double trim_phi = 0;
    double trim_theta = 1*M_PI/180;
    double trim_Va = 10;
    double trim_alpha = 1*M_PI/180;
    double trim_beta = 0;
    double trim_r = 0*M_PI/180;
    vector<double> trim_data(6);
    trim_data[0] = trim_phi;
    trim_data[1] = trim_theta;
    trim_data[2] = trim_Va;
    trim_data[3] = trim_alpha;
    trim_data[4] = trim_beta;
    trim_data[5] = trim_r;

    TrimState_t trimState = calcTrimState(trim_data);

    // Calculate trim derivatives

    opt.set_min_objective(costWrapper, &trimState);

    opt.set_xtol_abs(1e-2);
    // opt.set_ftol_abs(1);
    // opt.set_xtol_rel(1e-4);

    cout << "Generated trim state:\n";
    cout << "position:\n" << printVector(trimState.trimState.position, "\n");
    cout << "euler:\n" << printVector(trimState.trimState.euler, "\n");
    cout << "linearVel:\n" << printVector(trimState.trimState.linearVel, "\n");
    cout << "angularVel:\n" << printVector(trimState.trimState.angularVel, "\n");
    cout << endl;

    cout << "Desired state derivatives:\n";
    cout << "posDot:\n" << printVector(trimState.trimDerivatives.position, "\n");
    cout << "eulDot:\n" << printVector(trimState.trimDerivatives.euler, "\n");
    cout << "speedDot:\n" << printVector(trimState.trimDerivatives.linearVel, "\n");
    cout << "rateDot:\n" << printVector(trimState.trimDerivatives.angularVel, "\n");
    cout << endl;

    OptimResult_t result;

    uint32_t loopNum=1000;
    double seconds;

    cout << "Testing optimization time requirement." << endl;

    auto t_start = chrono::steady_clock::now();
    for (uint32_t i=0; i<loopNum; i++)
    {
        result = sample(&opt, trimState, initInput);
    }
    auto t_end = chrono::steady_clock::now();
    seconds = (double)chrono::duration_cast<chrono::milliseconds>(t_end-t_start).count()/1000;


    cout << "Found minimum at\n" << printVector(result.trimInput) << "\n with value = " << setprecision(10) << result.cost << endl;
    cout << "after " << optimSteps << " steps.\n";
    cout << "Optimization return code: " << result.returnCode << endl;
    cout << endl;

    // Re-run the optimal state and input
    uav->setState(convertState4ll(trimState.trimState));
    uav->setInput(convertInput4ll(result.trimInput));
    uav->step();

    cout << "Optimal calculated propagated state:\n";
    cout << "position:\n" << printVector(uav->state.pose.position, "\n");
    cout << "orientation:\n" << printVector(uav->state.pose.orientation, "\n");
    cout << "linearVel:\n" << printVector(uav->state.velocity.linear, "\n");
    cout << "angularVel:\n" << printVector(uav->state.velocity.angular, "\n");
    cout << endl;
    cout << "Optimal state derivatives:\n";
    cout << "posDot:\n" << printVector(uav->kinematics.stateDot.posDot, "\n");
    cout << "speedDot:\n" << printVector(uav->kinematics.stateDot.speedDot, "\n");
    cout << "rateDot:\n" << printVector(uav->kinematics.stateDot.rateDot, "\n");
    cout << endl;

    cout << seconds << " second(s) elapsed for " << loopNum << " steps" << endl;
}