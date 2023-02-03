
#ifndef PROG_UTILS
#define PROG_UTILS

#include <random>
#include <iostream>

#include <Eigen/Eigen>

#include "yaml-cpp/yaml.h"
#include "last_letter_lib/math_utils.hpp"

using namespace std;

namespace last_letter_lib
{
    namespace programming_utils
    {
        // Model plane configuration structure
        struct ConfigsStruct_t
        {
            YAML::Node world;
            YAML::Node env;
            YAML::Node init;
            YAML::Node inertial;
            YAML::Node aero;
            YAML::Node prop;
            YAML::Node ground;
        };

        typedef enum
        {
            PARAM_TYPE_WORLD = 0,
            PARAM_TYPE_ENV,
            PARAM_TYPE_INIT,
            PARAM_TYPE_INERTIAL,
            PARAM_TYPE_AERO,
            PARAM_TYPE_PROP,
            PARAM_TYPE_GROUND
        } ParamType_t;

        // Parameter functions
        //////////////////////

        template <typename T>
        bool getParameter(const YAML::Node configFile, string paramName, T &targetVar, bool isFatal = true)
        {
            // std::cout << "Parsing config:\n" << configFile << std::endl;
            if (configFile[paramName])
            {
                targetVar = configFile[paramName].as<T>();
                return true;
            }
            else if (isFatal)
            {
                std::cerr << "Unknown parameter " << paramName;
                std::cerr << " requested from config:\n" << configFile << std::endl;
                return false;
            }
            else
            {
                return false;
            }
        }

        template <typename T>
        bool setParameter(YAML::Node &configFile, string paramName, const T newValue, bool editOnly = true)
        {
            if (configFile[paramName])
            {
                configFile[paramName] = newValue;
                return true;
            }
            else
            {
                if (editOnly)
                {
                    std::cerr << "Unknown parameter " << paramName << std::endl;
                    return false;
                }
                else
                {
                    configFile[paramName] = newValue;
                    return true;
                }
            }
        }

        // Same as above but for vectors
        template <typename T>
        bool getParameterList(YAML::Node configFile, string paramName, std::vector<T> &targetVector, bool isFatal = true)
        {
            if (configFile[paramName])
            {
                if (!configFile[paramName].IsSequence())
                { // Test this with a counter example
                    throw invalid_argument("Parameter " + paramName + " does not contain a sequence");
                }
                // cout << paramName << " array size: " << configFile[paramName].size() << endl;
                targetVector = configFile[paramName].as<std::vector<T>>();
                // for (int i=0; i<configFile[paramName].size(); i++) {
                //     targetVector.push_back(configFile[paramName][i].as<T>());
                // }
                return true;
            }
            else if (isFatal)
            {
                throw invalid_argument("Unknown parameter " + paramName);
            }
            else
            {
                return false;
            }
        }

        // Filter a YAML::Node file to keep only a sub-parameter set
        // Example: for start string /world, keep /world/timeControls but not /environment/rho
        YAML::Node filterConfig(YAML::Node config, std::string prefix);

        YAML::Node randomizeConfig(YAML::Node config, vector<string> stringVec, double std_dev);

        ConfigsStruct_t randomizeConfigsStruct(const ConfigsStruct_t p_configStruct, const YAML::Node randomizerConfig);

        ConfigsStruct_t loadModelConfig(string modelName);

        string getHomeFolder();

        // Build a new polynomial, reading from a configuration YAML Node
        math_utils::Polynomial *buildPolynomial(YAML::Node config);

        // String manipulation
        //////////////////////

        template <typename T>
        string vectorToString(vector<T> p_vector)
        {
            ostringstream oss;
            for (auto i = p_vector.begin(); i != p_vector.end(); i++)
            {
                oss << *i << ' ';
            }
            return oss.str();
        }

        template <typename T>
        string vectorToString2(vector<T> p_vector, const string delimiter = ", ")
        {
            ostringstream oss;
            for (uint i = 0; i < p_vector.size(); i++)
            {
                oss << p_vector[i] << delimiter;
            }
            return oss.str();
        }

        string vectorToString2(Eigen::Vector3d vec3d, const string delimiter = ", ");

        string vectorToString2(Eigen::Quaterniond quat, const string delimiter = ", ");

        // Find if mainStr starts with startString
        bool startsWith(std::string mainStr, std::string startStr);

        // Split a string at a delimiter
        // From http://www.martinbroadhurst.com/how-to-split-a-string-in-c.html
        template <class Container>
        void split_string(const std::string &c_name, Container &cont, char delim)
        {
            std::istringstream iss(c_name);
            std::string token;
            while (std::getline(iss, token, delim))
            {
                cont.push_back(token);
            }
        }

        // Generic logic
        ////////////////

        // Toggler class
        // Read a 0-1 input and increase the internal state.
        // Loop over if reached maximum state.
        class TogglerInput
        {
        public:
            TogglerInput(const uint8_t num_states);
            void set_state(const uint8_t state);
            void set_threshold(const double threshold);
            void read_input(const double input);
            uint8_t get_state();
            double get_state_normalized(); // Return state as 0-1 value

        private:
            uint8_t num_states_;
            uint8_t state_{0};
            bool input_active_{false};
            double threshold_{0.5};
        };

        class TogglerPwm
        {
        public:
            TogglerPwm(const uint8_t num_states);
            void set_state(const uint8_t state);
            void set_threshold(const double threshold);
            void read_input(const unsigned int input);
            uint8_t get_state();
            double get_state_pwm(); // Return state as 1000-2000 value

        private:
            TogglerInput toggler_;
            unsigned int threshold_pwm_{1500};
        };
    } // namespace programming_utils
} // namespace last_letter_lib

#endif
