#pragma once

#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "system_interfaces/msg/adaptation_type.hpp"
#include "system_interfaces/msg/generic_adaptation.hpp"
#include "system_interfaces/msg/heartbeat.hpp"
#include "system_interfaces/msg/parametrization_input.hpp"
#include <map>

using GenericAdaptation = system_interfaces::msg::GenericAdaptation;
using ParametrizationInput = system_interfaces::msg::ParametrizationInput;
using VectorGenericAdaptations = std::vector<GenericAdaptation>;
using AdaptationMap = std::map<std::string, VectorGenericAdaptations>;
using ParamValue = rcl_interfaces::msg::ParameterValue;
using ParamType = rcl_interfaces::msg::ParameterType;
using AdaptationType = system_interfaces::msg::AdaptationType;
using ValueStore = std::map<std::string, ParamValue>;
using ValueStorePtr = std::shared_ptr<ValueStore>;
using AdaptationGenerator = std::function<GenericAdaptation(ValueStorePtr)>;
using Heartbeat = system_interfaces::msg::Heartbeat;

enum class ServiceStatus
{
    RUNNING,
    SUCCESS,
    FAILURE
};