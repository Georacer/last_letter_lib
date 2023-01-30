#include <iostream>
#include <fstream>
#include <string>
#include "yaml-cpp/yaml.h"
#include "last_letter_lib/prog_utils.hpp"

std::string param_path = "last_letter_lib/test/";
std::string filename = "test_params.yaml";

int main(int, char**){

    std::string full_filename = param_path + filename;
    YAML::Node config = YAML::LoadFile(full_filename);
    std::cout << "Read param file" << "\n";

    if (config["/world/simRate"]) {
    std::cout << "Simulation Rate: " << config["/world/simRate"] << "\n";
    }

    // Test basic parameter access
    std::string timeControls = config["/world/timeControls"].as<std::string>();
    std::cout << "time controls: " << timeControls << "\n";
    std::string integratorType = config["/world/integratorType"].as<std::string>();
    std::cout << "integrator type: " << integratorType << "\n";
    config["lastLogin"] = 5.0;

    getParameter(config, "/world/integratorType", integratorType);
    std::cout << "integrator type (alternate access): " << integratorType << "\n";

    std::vector<int> intVector;
    getParameterList(config, "/anIntSequence", intVector);
    string vectorString = vectorToString(intVector);
    cout << "Int Sequence :" << vectorString << "\n";

    std::vector<double> doubleVector;
    getParameterList(config, "/aDoubleSequence", doubleVector);
    vectorString = vectorToString2(doubleVector);
    cout << "Double Sequence :" << vectorString << "\n";

    // Write-out new parameter file
    std::ofstream fout(filename);
    fout << config;
}
