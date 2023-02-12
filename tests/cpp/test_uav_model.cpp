#include <iostream>
#include <gtest/gtest.h>

#include "yaml-cpp/yaml.h"
#include "last_letter_lib/prog_utils.hpp"
#include "last_letter_lib/uav_utils.hpp"
#include "last_letter_lib/uav_model.hpp"

#include "test_utils.hpp"

using namespace std;
using namespace Eigen;

using namespace last_letter_lib;

TEST(TestUavModel, TestInstantation)
{
     // Read parameter files
     auto config = load_config_aircraft("skywalker_2013");
     SimState_t state = build_aircraft_state_from_config(config);

     // Create model
     last_letter_lib::UavModel uavModel(config);

     ASSERT_TRUE(true);
}
