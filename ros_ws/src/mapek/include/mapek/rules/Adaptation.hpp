#pragma once 
#include "mapek/util/definitions.hpp"

class Adaptation{
    public: 
        Adaptation(std::string component, uint8_t adaptation_type, AdaptationGenerator generator);
        std::string getComponent() const;
        GenericAdaptation getAdaptation(ValueStorePtr) const;
        uint8_t getAdaptationType() const;
    private:
        std::string component;
        uint8_t adaptation_type;
        AdaptationGenerator generator;
};