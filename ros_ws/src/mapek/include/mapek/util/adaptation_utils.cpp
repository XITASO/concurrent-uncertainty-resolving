#include "mapek/util/adaptation_utils.hpp"
#include "system_interfaces/msg/adaptation_type.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include <iostream>

namespace adaptations_utils{

/**
 * @brief AdaptationTypeEnum from String 
 * @param type The type of the adaptation as String
 *
 * @return the adaptation type as set in system_interfaces::msg::AdaptationType based on the input string
*/
uint8_t deduceAdaptationType(std::string type){
    if (type == "action_activate") return system_interfaces::msg::AdaptationType::ACTION_ACTIVATE;
    if (type == "action_deactivate") return system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE;
    if (type == "action_restart") return system_interfaces::msg::AdaptationType::ACTION_RESTART;
    if (type == "action_redeploy") return system_interfaces::msg::AdaptationType::ACTION_REDEPLOY;
    if (type == "action_change_communication") return system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION;
    if (type == "action_set_parameter") return system_interfaces::msg::AdaptationType::ACTION_SET_PARAMETER;
    if (type == "action_increase_parameter") return system_interfaces::msg::AdaptationType::ACTION_INCREASE_PARAMETER;
    if (type == "action_decrease_parameter") return system_interfaces::msg::AdaptationType::ACTION_DECREASE_PARAMETER;
    if (type == "action_change_mode") return system_interfaces::msg::AdaptationType::ACTION_CHANGE_MODE;
    throw std::runtime_error ("unknown adaptation type: "+type);
}

int getAdaptationExecutionEstimate(uint8_t adaptation_type){
    switch(adaptation_type){
        case system_interfaces::msg::AdaptationType::ACTION_ACTIVATE :
            return 5;
        case system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE :
            return 1;
        case system_interfaces::msg::AdaptationType::ACTION_RESTART :
            return 10;
        case system_interfaces::msg::AdaptationType::ACTION_REDEPLOY :
            return 15;
        case system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION :
            return 2;
        case system_interfaces::msg::AdaptationType::ACTION_SET_PARAMETER :
            return 1;
        case system_interfaces::msg::AdaptationType::ACTION_INCREASE_PARAMETER :
            return 1;
        case system_interfaces::msg::AdaptationType::ACTION_DECREASE_PARAMETER :
            return 1;
        case system_interfaces::msg::AdaptationType::ACTION_CHANGE_MODE :
            return 1;
        default:
            throw std::runtime_error ("unknown adaptation type");
    }
}

/**
 * @brief Converts a Generic Adaptation to a Mode of the system_modes package
 * @param adaptation The generic adaptation
 *
 * @return a Mode based on the given generic adaptation  
*/
system_modes_msgs::msg::Mode toSystemMode(GenericAdaptation adaptation)
{
    system_modes_msgs::msg::Mode mode{};
    mode.label = adaptation.parameter.value.string_value;
    return mode;
}

GenericAdaptation toGenericAdaptation(system_modes_msgs::msg::Mode mode)
{
    GenericAdaptation adaptation;
    adaptation.action = system_interfaces::msg::AdaptationType::ACTION_CHANGE_MODE;
    adaptation.parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    adaptation.parameter.value.string_value = mode.label;
    return adaptation;
}

/**
 * @brief Converts a Generic Adaptation to a CommChangeAdaptation
 * @param adaptation The generic adaptation
 *
 * @return a CommChangeAdaptation based on the given generic adaptation  
*/
system_interfaces::msg::CommChangeInput toCommChangeAdaptation(GenericAdaptation adaptation){
    system_interfaces::msg::CommChangeInput comm_adaptation{};
    // TODO sanity check
    comm_adaptation.communication_name.data = adaptation.parameter.name;
    comm_adaptation.value.data = adaptation.parameter.value.string_value;
    return comm_adaptation;
}

/**
 * @brief Converts a Generic Adaptation to a ParametrizationAdaptation
 * @param adaptation The generic adaptation
 *
 * @return a ParametrizationAdaptation based on the given generic adaptation  
*/
system_interfaces::msg::ParametrizationInput toParametrizationAdaptation(GenericAdaptation adaptation){
    system_interfaces::msg::ParametrizationInput para_adaptation{};
    para_adaptation.action = adaptation.action;
    para_adaptation.parameter = adaptation.parameter;
    return para_adaptation;
}

/**
 * @brief Converts a Generic Adaptation to a LifecyleAdaptation
 * @param adaptation The generic adaptation
 *
 * @return a LifecyleAdaptation based on the given generic adaptation  
*/
std::vector<lifecycle_msgs::msg::Transition> toLifecyleAdaptation(GenericAdaptation adaptation){
    std::vector<lifecycle_msgs::msg::Transition> transitions; 

    lifecycle_msgs::msg::Transition adaptation_deactivate{};
    adaptation_deactivate.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
    adaptation_deactivate.label = "deactivation from managing system";

    lifecycle_msgs::msg::Transition adaptation_activate{};
    adaptation_activate.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    adaptation_activate.label = "activation from managing system";

    if (adaptation.action == system_interfaces::msg::AdaptationType::ACTION_ACTIVATE){
        transitions.push_back(adaptation_activate);
    } else if (adaptation.action == system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE){
        transitions.push_back(adaptation_deactivate);
    } else if (adaptation.action == system_interfaces::msg::AdaptationType::ACTION_RESTART){
        transitions.push_back(adaptation_deactivate);
        transitions.push_back(adaptation_activate);
    }
    return transitions;
}

/**
 * @brief Converts a CommChangeAdaptation to a Generic Adaptation
 * @param commm_adaptation The CommChangeAdaptation
 *
 * @return a Generic Adaptation on the given CommChangeAdaptation
*/
GenericAdaptation toGenericAdaptation(system_interfaces::msg::CommChangeInput comm_adaptation){
    GenericAdaptation adaptation;
    adaptation.action = system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION;
    adaptation.parameter.name = comm_adaptation.communication_name.data;
    adaptation.parameter.value.string_value = comm_adaptation.value.data;
    return adaptation;
}

/**
 * @brief Converts a ParametrizationAdaptation to a Generic Adaptation
 * @param s_adaptation The LifecyleAdaptation
 * 
 * @return a Generic Adaptation on the given ParametrizationAdaptation
*/
GenericAdaptation toGenericAdaptation(system_interfaces::msg::ParametrizationInput s_adaptation){
    GenericAdaptation adaptation;
    adaptation.action = s_adaptation.action;
    adaptation.parameter = s_adaptation.parameter;
    return adaptation;

}

/**
 * @brief Converts a LifecyleAdaptation to a Generic Adaptation
 * @param s_adaptation The LifecyleAdaptation
 *
 * @return a Generic Adaptation on the given LifecyleAdaptation
*/
GenericAdaptation toGenericAdaptation(lifecycle_msgs::msg::Transition s_adaptation){
    GenericAdaptation adaptation;
    if (s_adaptation.id == lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE){
        adaptation.action = system_interfaces::msg::AdaptationType::ACTION_ACTIVATE;
    }else if(s_adaptation.id == lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE){
        adaptation.action = system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE;
    }
    return adaptation;
}

/**
 * @brief Checks Adaptation
 * @param adaptation The Adaptation to be checked
 *
 * @return true if the adaptation is in order, returns false, if the adaptation is flawed, e.g. if values are missing or of the wrong type
*/
bool sanityCheck(GenericAdaptation adaptation){
    if (
        adaptation.action == system_interfaces::msg::AdaptationType::ACTION_REDEPLOY ||
        adaptation.action == system_interfaces::msg::AdaptationType::ACTION_ACTIVATE ||
        adaptation.action == system_interfaces::msg::AdaptationType::ACTION_DEACTIVATE ||
        adaptation.action == system_interfaces::msg::AdaptationType::ACTION_RESTART 
    ){
        return true;
    }

    // Change mode adaptation doesn't need a parameter name
    if (adaptation.action == system_interfaces::msg::AdaptationType::ACTION_CHANGE_MODE)
    {
        if (adaptation.parameter.value.string_value == "")
            return false;
        else
            return true;
    }

    if (adaptation.parameter.name == ""){
        std::cout<<"adaptation has no parameter name"<<std::endl;
        return false;
    }

    if(adaptation.action == system_interfaces::msg::AdaptationType::ACTION_CHANGE_COMMUNICATION){
        if (
            adaptation.parameter.value.type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING ||
            adaptation.parameter.value.string_value == ""
        )
        {
            std::cout<<"communication change adaptation must be of type string"<<std::endl;
            return false;
        }
    }

    return true;
}

// Define a function to split the adaptations based on desired actions
std::pair<std::vector<GenericAdaptation>, std::vector<GenericAdaptation>>
splitAdaptations(const std::vector<GenericAdaptation>& adaptations,
                 const std::vector<uint8_t>& desired_actions) {
    
    std::vector<GenericAdaptation> matching_adaptations;
    std::vector<GenericAdaptation> non_matching_adaptations;
    
    for (const auto& adaptation : adaptations) {
        if (std::find(desired_actions.begin(), desired_actions.end(), adaptation.action)
            != desired_actions.end()) {
            matching_adaptations.push_back(adaptation);
        } else {
            non_matching_adaptations.push_back(adaptation);
        }
    }
    
    return std::make_pair(matching_adaptations, non_matching_adaptations);
}

size_t countAdaptations(
    const std::vector<GenericAdaptation>& adaptations,
    const std::vector<uint8_t>& desired_actions) {
    
    size_t number_adaptations = 0;
    
    for (size_t i = 0; i < adaptations.size(); ++i) {
        if (std::find(desired_actions.begin(), desired_actions.end(), adaptations[i].action)
            != desired_actions.end()) {
                number_adaptations++;
        }
    }
    
    return number_adaptations;
}

}