#pragma once

#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"

using namespace last_letter_lib::programming_utils;
using namespace last_letter_lib::uav_utils;
using namespace last_letter_lib;

YAML::Node load_config_simple();
ParameterManager load_config_aircraft(std::string uav_name, bool randomize = false);
SimState_t build_aircraft_state_from_config(ParameterManager config);
