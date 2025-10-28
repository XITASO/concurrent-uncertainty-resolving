#pragma once

#include <vector>
#include "system_interfaces/msg/generic_adaptation.hpp"
#include "mapek/rules/expression/ExpressionFactory.hpp"
#include "mapek/rules/Adaptation.hpp"
#include "mapek/util/definitions.hpp"
#include <optional>
#include "mapek/util/string_utils.hpp"
#include "mapek/util/definitions.hpp"
#include <map>


// Helper factory to generate Adaptations (lambda functions)
struct AdaptationFactory{
    // creates Adaptations
    Adaptation produce(std::string) const;
    
    private:
    // creates Adaptations w/o parameters, input: action type
    AdaptationGenerator produceNoParamAdaptation(uint8_t type) const;

    // creates Adaptations w/ parameters, intput: action type and params as string to parse
    AdaptationGenerator produceParametrizedAdaptation(uint8_t type, std::string) const;

    ExpressionFactory expression_factory{};
};