
#ifndef PROG_UTILS
#define PROG_UTILS

#include <random>

#include <Eigen/Eigen>

#include "yaml-cpp/yaml.h"
#include "math_utils.hpp"

using namespace std;

// Model plane configuration structure
struct ConfigsStruct_t {
	YAML::Node world;
	YAML::Node env;
	YAML::Node init;
	YAML::Node inertial;
	YAML::Node aero;
	YAML::Node prop;
	YAML::Node ground;
};


template<typename T>
bool getParameter(const YAML::Node configFile, string paramName, T &targetVar, bool isFatal = true)
{
    if (configFile[paramName]) {
        targetVar = configFile[paramName].as<T>();
        return true;
    }
    else if (isFatal) {
        throw invalid_argument("Unknown parameter " + paramName);
    }
    else {
        return false;
    }
}

template<typename T>
bool setParameter(YAML::Node &configFile, string paramName, const T newValue, bool editOnly = true)
{
    if (configFile[paramName]) {
        configFile[paramName] = newValue;
        return true;
    }
    else if (editOnly) {
        throw invalid_argument("Unknown parameter " + paramName);
    }
    else {
        return false;
    }
}

// Same as above but for vectors
template<typename T>
bool getParameterList(YAML::Node configFile, string paramName, std::vector<T> &targetVector, bool isFatal = true)
{
    if (configFile[paramName]) {
        if (!configFile[paramName].IsSequence()) { // Test this with a counter example
            throw invalid_argument("Parameter " + paramName + " does not contain a sequence");
        }
        // cout << paramName << " array size: " << configFile[paramName].size() << endl;
        targetVector = configFile[paramName].as<std::vector<T>>();
        // for (int i=0; i<configFile[paramName].size(); i++) {
        //     targetVector.push_back(configFile[paramName][i].as<T>());
        // }
        return true;
    }
    else if (isFatal) {
        throw invalid_argument("Unknown parameter " + paramName);
    }
    else {
        return false;
    }
}

template<typename T>
string vectorToString(vector<T> p_vector ) {
    ostringstream oss;
    for (auto i = p_vector.begin(); i != p_vector.end(); i++) {
        oss << *i << ' ';
    }
    return oss.str();
}

template<typename T>
string vectorToString2(vector<T> p_vector, const string delimiter = ", ") {
    ostringstream oss;
    for (uint i = 0; i<p_vector.size(); i++) {
        oss << p_vector[i] << delimiter;
    }
    return oss.str();
}

string vectorToString2(Eigen::Vector3d vec3d, const string delimiter=", ");

string vectorToString2(Eigen::Quaterniond quat, const string delimiter=", ");

// Find if mainStr starts with startString
bool startsWith(std::string mainStr, std::string startStr);

// Filter a YAML::Node file to keep only a sub-parameter set
// Example: for start string /world, keep /world/timeControls but not /environment/rho
YAML::Node filterConfig(YAML::Node config, std::string prefix);

YAML::Node randomizeConfig(YAML::Node config, vector<string> stringVec, double std_dev);

ConfigsStruct_t randomizeConfigsStruct(const ConfigsStruct_t p_configStruct, const YAML::Node randomizerConfig);

ConfigsStruct_t loadModelConfig(string modelName);

string getHomeFolder();

/////////////////////////////////////////////////////////////////
// Build a new polynomial, reading from a configuration YAML Node
Polynomial * buildPolynomial(YAML::Node config);


#endif