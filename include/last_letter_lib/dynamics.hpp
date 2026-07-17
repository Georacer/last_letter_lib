#include <Eigen/Eigen>
#include "yaml-cpp/yaml.h"

#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/aerodynamics.hpp"
#include "last_letter_lib/gravity.hpp"
#include "last_letter_lib/propulsion/propulsion.hpp"
#include "last_letter_lib/ground_reaction/ground_reaction.hpp"
#include "last_letter_lib/environment.hpp"

using namespace std;

using Eigen::Quaterniond;
using Eigen::Vector3d;

// Dynamic model aggregator class

namespace last_letter_lib
{

// Container class
class Dynamics
{
public:
    Dynamics(ParameterManager p_worldConfig, ParameterManager p_aeroConfig, ParameterManager p_propConfig, ParameterManager p_groundConfig);
    ~Dynamics();
    void readParametersAerodynamics(ParameterManager config);
    void readParametersProp(ParameterManager config);
    void readParametersGround(ParameterManager config);
    void readParametersWorld(ParameterManager config);
    void setInput(Input input);																	   // store and convert new input values
    void setInputPwm(InputPwm_t input);																   // store and convert new PWM input values
    void update_local_state(SimState_t states, Environment_t environment);
    void calc_model(SimState_t states); // Calculate the forces and torques for each component.

    WrenchSum_t wrench_sum;
    vector<unique_ptr<Component>> components;
    int nWings; // number of airfoils mounted on the aircraft
    vector<aerodynamics::Aerodynamics*> aerodynamics;
    int nMotors; // number of motors mounted on the aircraft
    vector<propulsion::Thruster*> thrusters;
    ground_reaction::GroundReaction* groundReaction;
};

} // namespace last_letter_lib
