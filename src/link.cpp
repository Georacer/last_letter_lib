#include <last_letter_lib/link.hpp>
#include <last_letter_lib/math_utils.hpp>
#include <last_letter_lib/prog_utils.hpp>

using namespace last_letter_lib::programming_utils;

namespace last_letter_lib
{

    Link::Link(ParameterManager /*modelConfig*/, ParameterManager /*worldConfig*/)
    {
    }

    void Link::buildLink(ParameterManager modelConfig, ParameterManager /*worldConfig*/)
    {
        parseParametersLink(modelConfig);

        // inputGimbal_ = 0.0;
    }

    void Link::parseParametersLink(ParameterManager config)
    {
        name = config.get<string>("name"); // Parse link name
        // vector<double> doubleVect;
        // getParameterList(config, "CGOffset", doubleVect);
        // CGOffset_ = Vector3d(doubleVect.data());

        // doubleVect.clear();
        // getParameterList(config, "mountOrientation", doubleVect);
        // mountOrientation_ = Vector3d(doubleVect.data());

        // if (!getParameter(config, "gimbalAngle_max", gimbalAngle_max_, false))
        // {
        //     gimbalAngle_max_ = -1;
        // }
        // if (!getParameter(config, "chanGimbal", chanGimbal_, false))
        // {
        //     chanGimbal_ = -1;
        // }
        // inputGimbal_ = 0.0;
    }

    void Link::readParametersModel(ParameterManager config)
    {
        passModelParametersToModel(config);
    }

    void Link::readParametersWorld(ParameterManager config)
    {
        dt_ = config.get<double>("deltaT");
        passWorldParametersToModel(config);
    }

    // Input hanlding methods
    void Link::setInput(Input input)
    {
        // if (chanGimbal_ > -1)
        // {
        //     inputGimbal_ = gimbalAngle_max_ * input.value[chanGimbal_];
        // }
        setModelInput(input);
    }

    void Link::setInputPwm(InputPwm_t p_input)
    {
        Input input;
        // if (chanGimbal_ > -1)
        // {
        //     input.value[chanGimbal_] = PwmToFullRange(p_input.value[chanGimbal_]);
        // }
        setModelInputPwm(p_input);
    }

    // Convert the link state from World frame to local (link) frame
    void Link::rotateState(SimState_t linkState)
    {
        localState_ = linkState; // Copy over pose
        Quaterniond q_wl = linkState.pose.orientation;
        localState_.velocity.linear = q_wl * linkState.velocity.linear;
        localState_.velocity.angular = q_wl * linkState.velocity.angular;
        localState_.acceleration.linear = q_wl * linkState.acceleration.linear;
        localState_.acceleration.angular = q_wl * linkState.acceleration.angular;

        // std::cout << name << std::endl;
        // std::cout << localState_.velocity.linear << std::endl;
        // std::cout << localState_.velocity.angular << std::endl;
    }

    void Link::rotateEnvironment(Environment_t p_environment)
    {
        localEnvironment_ = p_environment;
        auto q_wl = localState_.pose.orientation;
        // Transform the relative wind from body axes to propeller axes
        Eigen::Vector3d localWind = q_wl * p_environment.wind;
        if (!std::isfinite(localWind.x()))
        {
            throw runtime_error("link.cpp/rotateEnvironment: NaN value in localWind.x");
        }
        if (std::fabs(localWind.x()) > 1e+160)
        {
            throw runtime_error("link.cpp/rotateEnvironment: localWind.x over 1e+160");
        }
        localEnvironment_.wind = localWind;
    }

    // Convert the resulting force from the gimbal axes to the body axes
    void Link::rotateWrench(Inertial /*inertial*/)
    {
        Vector3d forceLink = wrenchLinkFrame.force;
        Vector3d torqueLink = wrenchLinkFrame.torque;

        auto q_el = localState_.pose.orientation;

        wrenchWorldFrame.force = q_el.conjugate() * forceLink;
        // Convert the torque from the motor frame to the body frame
        wrenchWorldFrame.torque = q_el.conjugate() * torqueLink;

        // // Calculate the increased moment of inertia due to off-center torque application
        // // TODO: This doesn't sound right. Check my UAV Modelling manuscript to find the reasoning for this
        // double x, y, z;
        // double ratio = inertial.J(0, 0) / (inertial.J(0, 0) + inertial.mass * CGOffset_.x() * CGOffset_.x());
        // x = ratio * torque.x();
        // ratio = inertial.J(1, 1) / (inertial.J(1, 1) + inertial.mass * CGOffset_.y() * CGOffset_.y());
        // y = ratio * torque.y();
        // ratio = inertial.J(2, 2) / (inertial.J(2, 2) + inertial.mass * CGOffset_.z() * CGOffset_.z());
        // z = ratio * torque.z();
        // torque = Vector3d(x, y, z);

        // wrenchWorldFrame.torque = torque + CGOffset_.cross(wrenchWorldFrame.force);
    }

    void Link::step(SimState_t state, Inertial inertial, Environment_t environment)
    {
        // std::cout << "NED link velocities for link: " << this->name << "\n"
        //           << state.velocity.linear << "\n"
        //           << std::endl;
        rotateState(state);
        rotateEnvironment(environment);
        stepModelDynamics(localState_, inertial, localEnvironment_);
        wrenchLinkFrame = getWrench(localEnvironment_);
        if (!wrenchLinkFrame.force.allFinite())
        {
            throw runtime_error("link.cpp: NaN member in " + name + " force vector");
        }
        if (!wrenchLinkFrame.torque.allFinite())
        {
            throw runtime_error("link.cpp: NaN member in " + name + " torque vector");
        }
        rotateWrench(inertial); // Rotate wrench from link frame to world frame
    }

    // LinkAerodynamic methods

    LinkAerodynamic::LinkAerodynamic(ParameterManager modelConfig, ParameterManager worldConfig) : Link(modelConfig, worldConfig)
    {
        buildLink(modelConfig, worldConfig);
        buildDynamicModel(modelConfig, worldConfig);
    }

    LinkAerodynamic::~LinkAerodynamic()
    {
        delete aerodynamics;
    }

    void LinkAerodynamic::buildDynamicModel(ParameterManager modelConfig, ParameterManager /*worldConfig*/)
    {
        aerodynamics = buildAerodynamics(modelConfig);
    }

    void LinkAerodynamic::passModelParametersToModel(ParameterManager config)
    {
        aerodynamics->readParametersAerodynamics(config);
    }

    void LinkAerodynamic::passWorldParametersToModel(ParameterManager /*config*/)
    {
    }

    void LinkAerodynamic::setModelInput(Input input)
    {
        aerodynamics->setInput(input);
    }

    void LinkAerodynamic::setModelInputPwm(InputPwm_t input)
    {
        aerodynamics->setInputPwm(input);
    }

    void LinkAerodynamic::stepModelDynamics(SimState_t state, Inertial inertial, Environment_t environment)
    {
        aerodynamics->stepDynamics(state, inertial, environment);
    }

    Wrench_t LinkAerodynamic::getWrench(Environment_t /*environment*/)
    {
        return aerodynamics->wrenchAero;
    }

    // Propulsion link methods
    LinkPropulsion::LinkPropulsion(ParameterManager modelConfig, ParameterManager worldConfig) : Link(modelConfig, worldConfig)
    {
        buildLink(modelConfig, worldConfig);
        buildDynamicModel(modelConfig, worldConfig);
    }

    LinkPropulsion::~LinkPropulsion()
    {
        delete propulsion;
    }

    void LinkPropulsion::buildDynamicModel(ParameterManager modelConfig, ParameterManager worldConfig)
    {
        propulsion = propulsion::buildPropulsion(modelConfig, worldConfig);
    }

    void LinkPropulsion::passModelParametersToModel(ParameterManager config)
    {
        propulsion->readParametersProp(config);
    }

    void LinkPropulsion::passWorldParametersToModel(ParameterManager config)
    {
        propulsion->readParametersWorld(config);
    }

    void LinkPropulsion::setModelInput(Input input)
    {
        propulsion->setInput(input);
    }

    void LinkPropulsion::setModelInputPwm(InputPwm_t input)
    {
        propulsion->setInputPwm(input);
    }

    void LinkPropulsion::stepModelDynamics(SimState_t state, Inertial inertial, Environment_t environment)
    {
        propulsion->stepEngine(state, inertial, environment);
        // updatePropTF();
    }

    Wrench_t LinkPropulsion::getWrench(Environment_t)
    {
        return propulsion->wrenchProp;
    }

    // void LinkPropulsion::updatePropTF()
    // {
    //     gimbal_to_prop = Eigen::AngleAxis<double>(propulsion->theta, Vector3d::UnitX());
    //     body_to_prop = body_to_mount * mount_to_gimbal * gimbal_to_prop;
    // }

    // Ground Reactions link

    LinkGroundReaction::LinkGroundReaction(ParameterManager modelConfig, ParameterManager worldConfig) : Link(modelConfig, worldConfig)
    {
        buildLink(modelConfig, worldConfig);
        buildDynamicModel(modelConfig, worldConfig);
    }

    LinkGroundReaction::~LinkGroundReaction()
    {
        delete groundReaction_;
    }

    void LinkGroundReaction::buildDynamicModel(ParameterManager modelConfig, ParameterManager worldConfig)
    {
        groundReaction_ = ground_reaction::buildGroundReaction(modelConfig, worldConfig);
    }

    void LinkGroundReaction::passModelParametersToModel(ParameterManager config)
    {
        groundReaction_->readParametersGround(config);
    }

    void LinkGroundReaction::passWorldParametersToModel(ParameterManager config)
    {
        groundReaction_->readParametersWorld(config);
    }

    void LinkGroundReaction::setModelInput(Input input)
    {
        groundReaction_->setInput(input);
    }
    void LinkGroundReaction::setModelInputPwm(InputPwm_t input)
    {
        groundReaction_->setInputPwm(input);
    }

    void LinkGroundReaction::stepModelDynamics(SimState_t, Inertial, Environment_t)
    {
        throw runtime_error("Do not use this interface. Use stepModelDynamics(state, wrenchSum) instead.");
    }

    void LinkGroundReaction::stepModelDynamics(SimState_t state, WrenchSum_t wrenchSum)
    {
        wrenchLinkFrame.force = groundReaction_->getForce(state, wrenchSum);
        wrenchLinkFrame.torque = groundReaction_->getTorque(state, wrenchSum);
    }

    Wrench_t LinkGroundReaction::getWrench(Environment_t)
    {
        return wrenchLinkFrame;
    }
} // namespace last_letter_lib
