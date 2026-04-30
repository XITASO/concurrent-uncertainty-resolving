#pragma once

#include <iostream>
#include <fstream>
#include "mapek/util/string_utils.hpp"
#include "mapek/rules/RuleUpdater.hpp"


void updateProbabilityInFile(const std::string& path, const std::string& rule_name, const std::string& strat_name, const std::pair<double, double>& alpha_beta){
    
    std::ifstream file;
    file.open(path); 

    if (!file.is_open())
        throw std::runtime_error( "rule updater did not find file at: "+ path);

    uint line_num{};

    int found_line = -1;
    bool found_rule = false;
    std::string line;
    std::string word;

    std::vector<std::string> lines;

    while(std::getline(file, line)){
        
        lines.push_back(line);
        line = string_utils::strip(line);
        auto lr = string_utils::read_word(line);
        line = lr.second;
        word = lr.first;

        if (word == "RULE"){
            lr = string_utils::read_word(line);
            line = lr.second;
            word = lr.first;
            if (word == rule_name){
                if (found_rule){
                    // we doubly found the rule that cannot be --> PANIC
                    throw std::runtime_error("could not update strategy");
                }
                found_rule = true;
            }
        }
        if (word == "STRATEGY"){
            lr = string_utils::read_word(line);
            line = lr.second;
            word = lr.first;
            if (word == strat_name && found_rule && found_line==-1){
                found_line = line_num;
            }
        }

        line_num += 1;
    }

    file.close();

    if (found_line == -1){
        throw std::runtime_error("could not update strategy");
    }

    const double& alpha = alpha_beta.first;
    const double& beta = alpha_beta.second;

    double prob_percentage = alpha/(alpha+beta)*100.;
    double confidence = alpha+beta; 

    lines[found_line] = "        STRATEGY "+ strat_name + " " + string_utils::to_string(prob_percentage) + " " + string_utils::to_string(confidence);

    std::ofstream out_file;
    out_file.open(path); 

    if (!out_file.is_open())
        throw std::runtime_error( "rule updater did not find file for writing at: "+ path);

    for (const auto& line : lines){
        out_file << line<<std::endl;
    } 
    out_file.close();
}