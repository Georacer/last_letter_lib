#include <math.h>

#include <trimmer.hpp>

Trimmer::Trimmer(const string uavName):
    opt(nlopt::LN_BOBYQA, 4),
    // opt(nlopt::LN_COBYLA, 4);
    // opt(nlopt::LN_PRAXIS, 4); // Known working
    initInput(4, 0) 
{
    ConfigsStruct_t configs = loadModelConfig(uavName);
    uav = new UavModel(configs);

    std::vector<double> lb(4, -1);
    lb[2] = 0;
    opt.set_lower_bounds(lb);

    std::vector<double> ub(4, 1);
    opt.set_upper_bounds(ub);

    // Set initial input value
    vector<double> inputVect {0, 0, 0.5, 0};
    setInitInput(inputVect);

    // Calculate trim derivatives

    // opt.set_min_objective(costWrapper, &trimState);
    opt.set_min_objective(objFunWrapper, this);

    opt.set_xtol_abs(1e-2);
    // opt.set_ftol_abs(1);
    // opt.set_xtol_rel(1e-4);
}

Trimmer::~Trimmer()
{}

void Trimmer::setInitInput(const vector<double> inputVect)
{
    initInput[0] = inputVect[0];
    initInput[1] = inputVect[1];
    initInput[2] = inputVect[2];
    initInput[3] = inputVect[3];
}

void Trimmer::resetFunCallCount()
{
    funCallCount = 0;
}

// Calculate the full trim state and its derivative given the 6 independent trim variables
TrimState_t Trimmer::calcTrimState(const TrimParameters_t trimParams)
{
    double trim_phi = trimParams.phi;
    double trim_theta = trimParams.theta;
    double trim_Va = trimParams.Va;
    double trim_alpha = trimParams.alpha;
    double trim_beta = trimParams.beta;
    double trim_r = trimParams.r;

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
SimState_t Trimmer::convertState4ll(const State_t p_state)
{
    SimState_t state;
    state.pose.position = p_state.position;
    state.pose.orientation = euler2quat(p_state.euler).conjugate();
    // state.pose.orientation = euler2quat(p_state.euler);
    state.velocity.linear = p_state.linearVel;
    state.velocity.angular = p_state.angularVel;
    return state;
}

// Convert vector input to Input_t for passing to UavModel
Input_t Trimmer::convertInput4ll(const vector<double> &u)
{
    Input_t input;
    input.value[0] = u[0];
    input.value[1] = u[1];
    input.value[2] = u[2];
    input.value[3] = u[3];
    return input;
}

// Calculate the optimization error cost, based on the optimized input and the corresponding derivative
double Trimmer::calcCost(const Derivatives_t stateDer, Eigen::Vector4d input)
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
// double Trimmer::costWrapper(const vector<double> &u, vector<double> &grad, void *trimStatePtr)
double Trimmer::costWrapper(const vector<double> &u, vector<double> &grad, TrimState_t trimState)
{
    ++funCallCount;

    // TrimState_t * trimState = reinterpret_cast<TrimState_t *>(trimStatePtr);

    Input_t input = convertInput4ll(u);
    // SimState_t state = convertState4ll(trimState->trimState);
    SimState_t state = convertState4ll(trimState.trimState);

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

double Trimmer::objFunWrapper(const vector<double> &u, vector<double> &grad, void *trimmerObjPtr)
{
    Trimmer * obj = static_cast<Trimmer *>(trimmerObjPtr);
    return obj->costWrapper(u, grad, obj->trimState);
}

OptimResult_t Trimmer::findTrimInput(const TrimParameters_t p_trimParams)
{
    // extract the trim states and state derivatives
    trimState = calcTrimState(p_trimParams);

    // Copy the initializing input
    result.trimInput = initInput;

    try
    {
        nlopt::result returnCode = opt.optimize(result.trimInput, result.cost);

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

string Trimmer::printOptimalResult()
{
    ostringstream oss;
    // Re-run the optimal state and input
    uav->setState(convertState4ll(trimState.trimState));
    uav->setInput(convertInput4ll(result.trimInput));
    uav->step();

    oss << "Optimal calculated propagated state:\n";
    oss << "position:\n" << vectorToString2(uav->state.pose.position, "\n");
    oss << "orientation:\n" << vectorToString2(uav->state.pose.orientation, "\n");
    oss << "linearVel:\n" << vectorToString2(uav->state.velocity.linear, "\n");
    oss << "angularVel:\n" << vectorToString2(uav->state.velocity.angular, "\n");
    oss << endl;
    oss << "Optimal state derivatives:\n";
    oss << "posDot:\n" << vectorToString2(uav->kinematics.stateDot.posDot, "\n");
    oss << "speedDot:\n" << vectorToString2(uav->kinematics.stateDot.speedDot, "\n");
    oss << "rateDot:\n" << vectorToString2(uav->kinematics.stateDot.rateDot, "\n");
    oss << endl;

    return oss.str();
}