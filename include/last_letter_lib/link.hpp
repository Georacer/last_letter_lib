#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>
#include <last_letter_lib/uav_utils.hpp>
#include <last_letter_lib/aerodynamics.hpp>
#include <last_letter_lib/propulsion/propulsion.hpp>
#include <last_letter_lib/ground_reaction/ground_reaction.hpp>
#include <last_letter_lib/environment.hpp>

using Eigen::Vector3d;

namespace last_letter_lib
{
    class Link
    {
    public:
        Link(YAML::Node modelConfig, YAML::Node WorldConfig);
        virtual ~Link(){};
        void buildLink(YAML::Node modelConfig, YAML::Node worldConfig);                     // Capture link config and pass rest to dynamic model
        void parseParametersLink(YAML::Node config);                                        // Capture link-specific parameters
        void readParametersModel(YAML::Node config);                                        // Update link parameters
        void readParametersWorld(YAML::Node config);                                        // Update link world parameters
        virtual void passModelParametersToModel(YAML::Node config) = 0;                     // Update internal model parameters
        virtual void passWorldParametersToModel(YAML::Node config) = 0;                     // Update internal model world parameters
        virtual void buildDynamicModel(YAML::Node modelConfig, YAML::Node worldConfig) = 0; // Capture dynamic model parameters and build model
        // Input hanlding methods
        void setInput(Input_t input);
        void setInputPwm(InputPwm_t input);
        virtual void setModelInput(Input_t input) = 0;
        virtual void setModelInputPwm(InputPwm_t input) = 0;
        // Rotations-handling methods
        void rotateState(SimState_t states);
        void rotateEnvironment(Environment_t environment); // convert the wind to the propeller axes
        void rotateWrench(Inertial_t);
        // Other methods
        void step(SimState_t state, Inertial_t inertial, Environment_t environment); // Calculate the forces and torques for each Wrench_t source
        virtual void stepModelDynamics(SimState_t, Inertial_t, Environment_t) = 0;
        virtual Wrench_t getWrench(Environment_t) = 0;

        // Members
        // Eigen::Quaterniond q_bm, q_mg, q_bg;                                                                   // Quaternion rotations
        // Eigen::Transform<double, 3, Eigen::Affine> body_to_mount, mount_to_gimbal, body_to_gimbal;             // Transformations in the airfoil assembly for visual rendering
        // Eigen::Transform<double, 3, Eigen::Affine> body_to_mount_rot, mount_to_gimbal_rot, body_to_gimbal_rot; // Transformations in the airfoil assembly for force and moment rotation
        Wrench_t wrenchLinkFrame;
        Wrench_t wrenchWorldFrame;
        std::string name;

    private:
        double dt_; // Integration time
        // Vector3d CGOffset_;         // vector from CG to engine coordinates
        // Vector3d mountOrientation_; // YPR mounting orientation of the wing
        // double gimbalAngle_max_;    // Control inputs and maximum surface deflections
        // double inputGimbal_;
        // int chanGimbal_;
        SimState_t localState_;
        Environment_t localEnvironment_;
    };

    class LinkAerodynamic : public Link
    {
    public:
        LinkAerodynamic(YAML::Node modelConfig, YAML::Node worldConfig);
        ~LinkAerodynamic();
        void buildDynamicModel(YAML::Node modelConfig, YAML::Node worldConfig);
        void passModelParametersToModel(YAML::Node config); // Update internal model parameters
        void passWorldParametersToModel(YAML::Node config); // Update internal model world parameters
        void setModelInput(Input_t input);
        void setModelInputPwm(InputPwm_t input);
        virtual void stepModelDynamics(SimState_t, Inertial_t, Environment_t);
        Wrench_t getWrench(Environment_t);

    private:
        Aerodynamics *aerodynamics;
    };

    class LinkPropulsion : public Link
    {
    public:
        LinkPropulsion(YAML::Node prop_config, YAML::Node world_config);
        ~LinkPropulsion();
        void buildDynamicModel(YAML::Node prop_config, YAML::Node world_config);
        void passModelParametersToModel(YAML::Node config); // Update internal model parameters
        void passWorldParametersToModel(YAML::Node config); // Update internal model world parameters
        void setModelInput(Input_t input);
        void setModelInputPwm(InputPwm_t input);
        virtual void stepModelDynamics(SimState_t, Inertial_t, Environment_t);
        Wrench_t getWrench(Environment_t);
        // void updatePropTF();
        Eigen::Transform<double, 3, Eigen::Affine> gimbal_to_prop, body_to_prop; // Transformations in the propeller assembly for visual rendering
        propulsion::Propulsion *propulsion;

    private:
    };

    class LinkGroundReaction : public Link
    {
    public:
        LinkGroundReaction(YAML::Node modelConfig, YAML::Node worldConfig);
        ~LinkGroundReaction();
        void buildDynamicModel(YAML::Node modelConfig, YAML::Node worldConfig);
        void passModelParametersToModel(YAML::Node config); // Update internal model parameters
        void passWorldParametersToModel(YAML::Node config); // Update internal model world parameters
        void setModelInput(Input_t input);
        void setModelInputPwm(InputPwm_t input);
        void stepModelDynamics(SimState_t, Inertial_t, Environment_t);
        void stepModelDynamics(SimState_t, WrenchSum_t);
        Wrench_t getWrench(Environment_t);

    private:
        ground_reaction::GroundReaction *groundReaction_;
    };
} // namespace last_letter_lib
