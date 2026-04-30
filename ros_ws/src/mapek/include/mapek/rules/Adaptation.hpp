#pragma once 
#include "mapek/util/definitions.hpp"

class Adaptation{
    public: 
        Adaptation(std::string component, uint8_t adaptation_type, AdaptationGenerator generator, int system_impact = 0);
        std::string getComponent() const;
        GenericAdaptation getAdaptation(ValueStorePtr) const;
        uint8_t getAdaptationType() const;
        int getSystemImpact() const;
    private:
        std::string component;
        uint8_t adaptation_type;
        AdaptationGenerator generator;
        int system_impact;
};