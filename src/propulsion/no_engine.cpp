//////////////////
// No Engine Model
//////////////////

// Constructor
NoEngine::NoEngine(YAML::Node propConfig, YAML::Node worldConfig):Propulsion(propConfig, worldConfig)
{
	omega = 0.0;
}

// Destructor
NoEngine::~NoEngine()
{
}

void NoEngine::updateRadPS(SimState_t /* states */, Inertial_t /* inertial */, Environment_t /* environment */)
{
}

// Force calculation function
void NoEngine::getForce(SimState_t /* states */, Inertial_t /* inertial */, Environment_t /* environment */)
{
}

// Torque calculation function
void NoEngine::getTorque(SimState_t /* states */, Inertial_t /* inertial */, Environment_t /* environment */)
{
}
