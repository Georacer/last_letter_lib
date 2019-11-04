#include <math.h>
#include <iomanip>

#include <trimmer.hpp>

extern "C"
{
    TrimmerInput * trimmer_input_new(char * uavName)
    {
        TrimmerInput * newTrimmer = new TrimmerInput(uavName);
        return newTrimmer;
    }
    double * find_input_trim(TrimmerInput* trimmer, double * trimState)
    {
        static double result[6];
        trimmer->pyFindTrimInput(trimState, result);
        return result;
    }
    double print_optimal_input_result(TrimmerInput* trimmer)
    {
        trimmer->printOptimalResult(true);
        return 1;
    }

    TrimmerState * trimmer_state_new(char * uavName)
    {
        TrimmerState * newTrimmer = new TrimmerState(uavName);
        return newTrimmer;
    }
    double * find_state_trim(TrimmerState* trimmer, double * trimTrajectory)
    {
        static double result[14];
        trimmer->pyFindTrimState(trimTrajectory, result);
        return result;
    }
    double print_optimal_state_result(TrimmerState* trimmer)
    {
        trimmer->printOptimalResult(true);
        return 1;
    }
}

TrimmerState::TrimmerState(const string uavName):
    opt(nlopt::LN_BOBYQA, 12),
    initState(12, 0),
    trimInput(4,0)
{
    ConfigsStruct_t configs = loadModelConfig(uavName);
    uav = new UavModel(configs);

    // Optimization variables: phi, theta, Va, aoa, aos, p, q, r, da, de, dt, dr: 12 in total

    std::vector<double> lb(12, 0); // Initialize lower bound vector
    lb[0] = -M_PI/2; // Phi lower bound
    lb[1] = -M_PI/2; // Theta lower bound
    lb[2] = 1; // Va lower bound
    lb[3] = -M_PI/4; // aoa lower bound
    lb[4] = -M_PI/4; // aos lower bound
    lb[5] = -3; // p lower bound
    lb[6] = -3; // q lower bound
    lb[7] = -1; // r lower bound
    lb[8] = -1; // deltaa lower bound
    lb[9] = -1; // deltae lower bound
    lb[10] = 0; // deltat lower bound
    lb[11] = -1; // deltar lower bound
    opt.set_lower_bounds(lb);

    std::vector<double> ub(12, 0); // Initialize upper bound vector
    ub[0] = M_PI/2; // Phi upper bound
    ub[1] = M_PI/2; // Theta upper bound
    ub[2] = 50; // Va upper bound
    ub[3] = M_PI/4; // aoa upper bound
    ub[4] = M_PI/4; // aos upper bound
    ub[5] = 3; // p upper bound
    ub[6] = 3; // q upper bound
    ub[7] = 1; // r upper bound
    ub[8] = 1; // deltaa upper bound
    ub[9] = 1; // deltae upper bound
    ub[10] = 1; // deltat upper bound
    ub[11] = 1; // deltar upper bound
    opt.set_upper_bounds(ub);

    // Calculate trim derivatives

    opt.set_min_objective(objFunWrapper, this);

    // opt.set_xtol_abs(1e-2);
    opt.set_ftol_abs(0.001);
    // opt.set_xtol_rel(1e-4);
}
TrimmerState::~TrimmerState()
{}

void TrimmerState::setInitState(const vector<double> inputVect)
{
    initState = inputVect;
}

void TrimmerState::resetFunCallCount()
{
    funCallCount = 0;
}

// Calculate the optimization error cost, based on the optimized input and the corresponding derivative
double TrimmerState::calcCost(const SimState_t state, const Derivatives_t stateDer, Eigen::Vector4d input)
{
    double derPositionWeight = 10;
    double derSpeedWeight = 10;
    double derAngleWeight = 1000;
    double derRateWeight = 100;
    double airspeedWeight = 100;
    double aosWeight = 1;
    double inputWeight = 1;

    double targetVa = targetTrajectory.Va;
    double targetGamma = targetTrajectory.Gamma;
    double targetR = targetTrajectory.R;

    double posDownDotDesired = -targetVa*sin(targetGamma);
    double psiDotDesired = targetVa/targetR*cos(targetGamma);

    // Calculate position derivative error
    double posDotError = posDownDotDesired - stateDer.posDot(2);
    double posDerTerm = posDotError * derPositionWeight * posDotError;

    // Calculate velocity derivative error
    double speedDerTerm = stateDer.speedDot.transpose()*derSpeedWeight*stateDer.speedDot;

    // Calculate angle derivative error
    Vector3d angleDerDesired;
    angleDerDesired << 0, 0, psiDotDesired;
    Vector3d eulerAngles = quat2euler(state.pose.orientation);
    Vector3d eulerDerivatives(getEulerDerivatives(eulerAngles, state.velocity.angular));
    Vector3d angleDerError = angleDerDesired - eulerDerivatives;
    double angleDerTerm = angleDerError.transpose() * derAngleWeight * angleDerError;

    // Calculate rate derivative error
    double rateDerTerm = stateDer.rateDot.transpose()*derRateWeight*stateDer.rateDot;
    // The position and orientation derivatives are not taken into account, because they are designed to be optimal
    // and the optimized input vector does not affect them anyway.

    // Calculate output error
    Vector3d airdata = getAirData(state.velocity.linear);
    double airspeedError = targetVa - airdata(0);
    double airspeedTerm = airspeedError * airspeedWeight * airspeedError;

    double aosTerm = airdata(2) * aosWeight * airdata(2);

    // Calculate input cost
    double inputTerm = input.transpose()*inputWeight*input;

    return posDerTerm + speedDerTerm + angleDerTerm + rateDerTerm
           + airspeedTerm + aosTerm
           + inputTerm;
}

// Wrapper to the cost function. Accepts concatenated state and input x under optimization and also passes the trimState parameter to the cost function
double TrimmerState::costWrapper(const vector<double> &optim_arg, vector<double> &grad)
{
    ++funCallCount;

    // Setup model input
    std::vector<double> u(4,0);
    u[0] = optim_arg[8];
    u[1] = optim_arg[9];
    u[2] = optim_arg[10];
    u[3] = optim_arg[11];
    Input_t input = convertInput4ll(u);
    Eigen::Vector4d inputVec;
    inputVec << u[0], u[1], u[2], u[3];

    // Setup model state
    SimState_t state;

    state.pose.position(2) = -10; // Lift from the ground the aircraft a bit

    Vector3d euler;
    euler(0) = optim_arg[0];
    euler(1) = optim_arg[1];
    euler(2) = 0;
    Quaterniond orientation = euler2quat(euler).conjugate();
    state.pose.orientation = orientation;

    Vector3d airdata;
    airdata << optim_arg[2], optim_arg[3], optim_arg[4];
    Vector3d velLinear = getVelocityFromAirdata(airdata);
    state.velocity.linear = velLinear;

    state.velocity.angular(0) = optim_arg[5];
    state.velocity.angular(1) = optim_arg[6];
    state.velocity.angular(2) = optim_arg[7];

    // If your motors have state as well (RPM) you need to set it here again

    // Simulate the optimization arguments
    uav->setState(state);
    uav->setInput(input);
    uav->step();

    Derivatives_t stateDot = uav->kinematics.stateDot;

    // Calculate the optimization cost
    return calcCost(state, stateDot, inputVec);
}

double TrimmerState::objFunWrapper(const vector<double> &u, vector<double> &grad, void *trimmerObjPtr)
{
    TrimmerState * obj = static_cast<TrimmerState *>(trimmerObjPtr);
    return obj->costWrapper(u, grad);
}

OptimResult_t TrimmerState::findTrimState(const TrimTrajectoryParameters_t p_trimParams)
{
    resetFunCallCount();

    // Build the initializing input
    vector<double> vectorInit(12,0);
    vectorInit[1] = p_trimParams.Gamma; // Set initial theta to gamma
    vectorInit[2] = p_trimParams.Va; // Set initial Va to trim Va
    vectorInit[7] = p_trimParams.Va/p_trimParams.R*cos(p_trimParams.Gamma); // Set initial r to psi_dot
    vectorInit[10] = 0.5; // Set initial throttle to half
    setInitState(vectorInit);

    targetTrajectory = p_trimParams;

    result.trimInput = initState;

    try
    {
        nlopt::result returnCode = opt.optimize(result.trimInput, result.cost);

        result.returnCode = returnCode;
        result.success = true;

        // Store locally the trim state
        trimState.trimState.position(2) = -10;
        trimState.trimState.euler(0) = result.trimInput[0];
        trimState.trimState.euler(1) = result.trimInput[1];
        Vector3d airdata;
        airdata(0) = result.trimInput[2];
        airdata(1) = result.trimInput[3];
        airdata(2) = result.trimInput[4];
        Vector3d velocities = getVelocityFromAirdata(airdata);
        trimState.trimState.linearVel = velocities;
        trimState.trimState.angularVel(0) = result.trimInput[5];
        trimState.trimState.angularVel(1) = result.trimInput[6];
        trimState.trimState.angularVel(2) = result.trimInput[7];

        // Store locally the trim input
        trimInput[0] = result.trimInput[8];
        trimInput[1] = result.trimInput[9];
        trimInput[2] = result.trimInput[10];
        trimInput[3] = result.trimInput[11];
    }
    catch (exception &e)
    {
        cout << "nlopt failed: " << e.what() << endl;
        result.success=false;
    }

    return result;
}

// Simplified wrapper for calling via ctypes python API
// Input: a double array[3] holding the trim values for (in order):
//  Va, gamma, R
// Ouptut: a double array[14] holding:
//  Positions 0-8: the trim state (phi, theta, Va, alpha, beta, p, q, r)
//  Positions 9-11: the trim control inputs
//  Position 12: the optimization cost
//  Position 13: the success flag (0/1)
void TrimmerState::pyFindTrimState(double * trimParamArray, double * result)
{
    TrimTrajectoryParameters_t trimParams;
    trimParams.Va = trimParamArray[0];
    trimParams.Gamma = trimParamArray[1];
    trimParams.R = trimParamArray[2];

    OptimResult_t resultStruct = findTrimState(trimParams);

    for (int i=0; i<12; i++)
    {
        result[i] = resultStruct.trimInput[i];
    }
    result[12] = resultStruct.cost;
    result[13] = (double)resultStruct.success;
}

string TrimmerState::printOptimalResult(bool verbose)
{
    ostringstream oss;
    // Re-run the optimal state and input
    uav->setState(convertState4ll(trimState.trimState));
    uav->setInput(convertInput4ll(trimInput));
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
    oss << "Trim input:\n";
    for (int i=0; i<4; i++)
    {
        oss << uav->input.value[i] << ", ";
    }
    oss << endl;
    oss << endl;

    Eigen::Vector3d airdata = getAirData(trimState.trimState.linearVel);
    double gamma = trimState.trimState.euler(1) - airdata(1);
    Vector3d eulerDot = getEulerDerivatives(trimState.trimState.euler, trimState.trimState.angularVel);
    double R = airdata(0)*cos(gamma)/eulerDot(2);

    cout << setprecision(3);
    cout << "Resulting trajectory:\n";
    cout << "Airspeed:\t" << airdata(0) << "(" << targetTrajectory.Va << ")\n";
    cout << "Gamma:\t" << gamma*180/M_PI << "(" << targetTrajectory.Gamma*180/M_PI << ")\n";
    cout << "R:\t" << R << "(" << targetTrajectory.R << ")\n";
    oss << endl;

    if (verbose)
    {
        std::cout << oss.str();
    }

    return oss.str();
}


////////////////////////////////
// TrimmerInput class definition
////////////////////////////////

TrimmerInput::TrimmerInput(const string uavName):
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

    opt.set_min_objective(objFunWrapper, this);

    // opt.set_xtol_abs(1e-2);
    opt.set_ftol_abs(0.001);
    // opt.set_xtol_rel(1e-4);
}

TrimmerInput::~TrimmerInput()
{}

void TrimmerInput::setInitInput(const vector<double> inputVect)
{
    initInput = inputVect;
}

void TrimmerInput::resetFunCallCount()
{
    funCallCount = 0;
}

// Calculate the full trim state and its derivative given the 6 independent trim variables
TrimState_t TrimmerInput::calcTrimState(const TrimStateParameters_t trimParams)
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

    TrimState_t newTrimState;
    newTrimState.trimState.position = Vector3d(0, 0, -100);
    newTrimState.trimState.euler = Vector3d(trim_phi, trim_theta, 0);
    Vector3d velocity = getVelocityFromAirdata(Vector3d(trim_Va, trim_alpha, trim_beta));
    newTrimState.trimState.linearVel = velocity;
    newTrimState.trimState.angularVel = Vector3d(trim_p, trim_q, trim_r);

    // Calculate derived quantities
    double gamma = trim_theta - trim_alpha;

    newTrimState.trimDerivatives.position = Vector3d(0, 0, -trim_Va*sin(gamma));
    newTrimState.trimDerivatives.euler = Vector3d(0, 0, k); // k=Va/R*cos(gamm)
    newTrimState.trimDerivatives.linearVel = Vector3d::Zero();
    newTrimState.trimDerivatives.angularVel = Vector3d::Zero();

    return newTrimState;
}

// Calculate the optimization error cost, based on the optimized input and the corresponding derivative
double TrimmerInput::calcCost(const Derivatives_t stateDer, Eigen::Vector4d input)
{
    // The cost function should ideally check the u and v components of the linear velocity.
    // But since this model deals with body-frame forces, instead of wind-frame forces (as the Python model/trimmer does)
    // the moments cannot affect alpha and beta. As a result, it is numerically very difficult for thrust input to fix all
    // of the u and w velocities.
    // Thus, for now we leave only airspeed in the juristiction of thrust and hope that alpha and beta can stay close to their
    // trimmed values, since control inputs will stabilize angular velocities
    double derSpeedWeight = 100;
    double derAngleWeight = 10000;
    double derRateWeight = 100;
    double inputWeight = 10;

    double speedDerTerm = stateDer.speedDot.transpose()*derSpeedWeight*stateDer.speedDot;
    // double speedDerTerm = stateDer.speedDot[0]*derErrorWeight*stateDer.speedDot[0];
    double angleDerTerm = stateDer.quatDot.coeffs().segment<2>(0).transpose()*derAngleWeight*stateDer.quatDot.coeffs().segment<2>(0);
    double rateDerTerm = stateDer.rateDot.transpose()*derRateWeight*stateDer.rateDot;
    // The position and orientation derivatives are not taken into account, because they are designed to be optimal
    // and the optimized input vector does not affect them anyway.

    double inputTerm = input.transpose()*inputWeight*input;

    return speedDerTerm + angleDerTerm + rateDerTerm + inputTerm;
}

// Wrapper to the cost function. Accepts input u under optimization and also passes the trimState parameter to the cost function
double TrimmerInput::costWrapper(const vector<double> &u, vector<double> &grad, TrimState_t trimState)
{
    ++funCallCount;

    Input_t input = convertInput4ll(u);
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

double TrimmerInput::objFunWrapper(const vector<double> &u, vector<double> &grad, void *trimmerObjPtr)
{
    TrimmerInput * obj = static_cast<TrimmerInput *>(trimmerObjPtr);
    return obj->costWrapper(u, grad, obj->trimState);
}

OptimResult_t TrimmerInput::findTrimInput(const TrimStateParameters_t p_trimParams)
{
    resetFunCallCount();

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

// Simplified wrapper for calling via ctypes python API
// Input: a double array[6] holding the trim values for (in order):
//  phi, theta, Va, alpha, beta, r
// Ouptut: a double array[6] holding:
//  Positions 0-3: the trim control inputs
//  Position 4: the optimization cost
//  Position 5: the success flag (0/1)
void TrimmerInput::pyFindTrimInput(double * trimParamArray, double * result)
{
    TrimStateParameters_t trimParams;
    trimParams.phi = trimParamArray[0];
    trimParams.theta = trimParamArray[1];
    trimParams.Va = trimParamArray[2];
    trimParams.alpha = trimParamArray[3];
    trimParams.beta = trimParamArray[4];
    trimParams.r = trimParamArray[5];

    OptimResult_t resultStruct = findTrimInput(trimParams);

    // double result[6];
    result[0] = resultStruct.trimInput[0];
    result[1] = resultStruct.trimInput[1];
    result[2] = resultStruct.trimInput[2];
    result[3] = resultStruct.trimInput[3];
    result[4] = resultStruct.cost;
    result[5] = (double)resultStruct.success;
}

string TrimmerInput::printOptimalResult(bool verbose)
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

    if (verbose)
    {
        std::cout << oss.str();
    }

    return oss.str();
}

// Convert State_t to SimState_t for passing to UavModel
SimState_t convertState4ll(const State_t p_state)
{
    SimState_t state;
    state.pose.position = p_state.position;
    state.pose.orientation = euler2quat(p_state.euler).conjugate();
    state.velocity.linear = p_state.linearVel;
    state.velocity.angular = p_state.angularVel;
    return state;
}

// Convert vector input to Input_t for passing to UavModel
Input_t convertInput4ll(const vector<double> &u)
{
    Input_t input;
    input.value[0] = u[0];
    input.value[1] = u[1];
    input.value[2] = u[2];
    input.value[3] = u[3];
    return input;
}
