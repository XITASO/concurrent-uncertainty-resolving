#pragma once
#include "mapek/util/definitions.hpp"
#include "mapek/rules/Adaptation.hpp"
#include <mapek/util/adaptation_utils.hpp>
#include <system_interfaces/msg/adaptation_type.hpp>
#include <set>

class Strategy{
    public:
        Strategy() {};
        Strategy(std::vector<Adaptation>, std::string name, std::size_t hash, double success_rate, double success_rate_confidence = 100, double system_impact=0);
        // Strategy(const Strategy&);
        // Strategy(const Strategy&&);
        // Strategy& operator= (const Strategy&);
        // Strategy& operator=(Strategy&& other);
        // ~Strategy() = default; 


        bool operator==(const Strategy& other) const;

        std::vector<GenericAdaptation> evaluateAdaptations(ValueStorePtr value_store) const;
        std::vector<Adaptation> getAdaptations() const;

        void updateSuccessRate(double);
        void updateSystemImpact(double);

        bool didWeTryThis() const;
        void reset() const;
        void markAsTried() const;

        int getExecutionEstimate() const;

        std::string getName() const;
        std::size_t  getHash() const;

        double getSuccessRate()const {return alpha/(alpha+beta);}
        std::pair<double, double> getBetaDistribution(){
            return {alpha, beta};
        }

        // double getSystemImpact()const {return system_impact;} 
        double getCost(bool consider_system_impact) const;

        std::set<std::string> getAffectedComponents() const;
        bool affectsComponent(std::string) const;

        void updateSuccessRate(bool successful);

    private:
        std::vector<Adaptation> adaptations;
        std::string name;
        std::size_t hash;

        
        double system_impact;
        double weight_probability = 0.5;
        double weight_time = 0.5;
        mutable bool already_tried_this = false;
        mutable double alpha; // alpha of Beta distr
        mutable double beta;  // beta of Beta distr
};