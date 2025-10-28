#include "mapek/rules/Strategy.hpp"
#include "mapek/util/adaptation_utils.hpp"

Strategy::Strategy(std::vector<Adaptation> adaptations, std::string name, std::size_t hash, double success_rate, double system_impact) : adaptations(adaptations),
                                                                                                                                         name(name),
                                                                                                                                         hash(hash),
                                                                                                                                         success_rate(success_rate),
                                                                                                                                         system_impact(system_impact) {}

// Strategy::Strategy(const Strategy& other):
//     adaptations(other.adaptations),
//     name(other.name),
//     hash(other.hash),
//     success_rate(other.success_rate),
//     system_impact(other.system_impact){
//     already_tried_this = other.already_tried_this;
// }

// // Strategy::Strategy(const Strategy&&);
// // Strategy& Strategy::operator= (const Strategy&){
// //     if ()
// // }
// Strategy& Strategy::operator=(Strategy&& other){
//     if (&other == this)
// 		return *this;

// 		// Copy the resource
// 		m_ptr = new T;
// 		*m_ptr = *a.m_ptr;

// 		return *this;
// }

bool Strategy::operator==(const Strategy &other) const
{
    // only use hash for comparison, bcs thats what a hash is good for
    return hash == other.getHash();
}

bool Strategy::didWeTryThis() const
{
    return already_tried_this;
}

void Strategy::markAsTried() const
{
    already_tried_this = true;
}

void Strategy::reset() const
{
    already_tried_this = false;
}

std::vector<GenericAdaptation> Strategy::evaluateAdaptations(ValueStorePtr value_store) const
{
    std::vector<GenericAdaptation> adaptation_msgs;
    for (const auto &ag : adaptations)
    {
        adaptation_msgs.push_back(ag.getAdaptation(value_store));
    }
    return adaptation_msgs;
}

std::vector<Adaptation> Strategy::getAdaptations() const
{
    return adaptations;
}

std::string Strategy::getName() const { return name; }
std::size_t Strategy::getHash() const { return hash; }

double Strategy::getCost(bool consider_system_impact) const
{
    if (consider_system_impact)
    {
        // Normalize probability from [0, 100] to [0, 1]
        double normalizedFailureCost = (100.0 - success_rate) / 100.0;
        // Normalize time from [1, 15] to [0, 1]
        double normalizedTime = (system_impact - 1.0) / adaptations_utils::getAdaptationExecutionEstimate(system_interfaces::msg::AdaptationType::ACTION_REDEPLOY - 1.0);

        // Calculate weighted cost
        return weight_probability * normalizedFailureCost + weight_time * normalizedTime;
    }
    else
    {
        return 100. - success_rate;
    }
}

// void Strategy::updateSuccessRate(double usr){
//     success_rate = usr;

//     // TODO save to file
// }
// void Strategy::updateSystemImpact(double usi){
//     system_impact = usi;
//     // TODO save to file
// }

std::set<std::string> Strategy::getAffectedComponents() const
{
    std::set<std::string> affected_components;
    for (const auto &a : adaptations)
        affected_components.insert(a.getComponent());
    return affected_components;
}

bool Strategy::affectsComponent(std::string component) const
{
    for (const auto &a : adaptations)
        if (a.getComponent() == component)
            return true;
    return false;
}

int Strategy::getExecutionEstimate() const
{
    int ee = 0;
    for (const auto &adaptation : adaptations)
    {
        ee = std::max(ee, adaptations_utils::getAdaptationExecutionEstimate(adaptation.getAdaptationType()));
    }
    return ee;
}