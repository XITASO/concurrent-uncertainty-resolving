#pragma once

#include <string>
#include <utility>

void updateProbabilityInFile(const std::string& path, const std::string& rule_name, const std::string& strat_name, const std::pair<double, double>& alpha_beta);