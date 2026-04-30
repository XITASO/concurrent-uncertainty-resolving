#pragma once 
#include "mapek/rules/Adaptation.hpp"

Adaptation::Adaptation(std::string component, uint8_t adaptation_type, AdaptationGenerator generator, int system_impact):
    component(component),
    adaptation_type(adaptation_type),
    generator(generator),
    system_impact(system_impact)
    {}
    
std::string Adaptation::getComponent() const {
    return component;
}

GenericAdaptation Adaptation::getAdaptation(ValueStorePtr value_store) const {
    return generator(value_store);
}

uint8_t Adaptation::getAdaptationType() const {
    return adaptation_type;
}

int Adaptation::getSystemImpact() const {
    return system_impact;
}