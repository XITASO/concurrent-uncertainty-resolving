#include "mapek/rules/RuleParser.hpp"
#include <fstream>
#include "mapek/util/string_utils.hpp"
#include "mapek/util/parameter_message_utils.hpp"
#include <stdexcept>
#include "mapek/rules/AdaptationFactory.hpp"

RuleParser::RuleParser(ValueStorePtr value_store):
value_store(value_store)
{}


/**
 * @brief Parses Adaptations for rule file 
 * 
 * first adds constants to blackboard
 * then parses rules
 * 
 * @param path the path to the rule file
 * 
 * @throws Runtime exception If the file input is unexpected
 * 
 * @return Vector of rules, pared from the file
 */
std::vector<RulePtr> RuleParser::parse(std::string path) const{
    std::vector<std::shared_ptr<Rule>> rules {};

    std::ifstream file;
    file.open(path); 

    if (!file.is_open())
        throw std::runtime_error( "did not find file at: "+ path);

    std::string line;

    // readconsts
    if (!std::getline(file, line)) throw std::runtime_error( "unexpected file ending");
    if (line != "BEGIN CONSTS") throw std::runtime_error( "expected BEGIN CONSTS");
    while (true){
        if (!std::getline(file, line)) throw std::runtime_error( "unexpected file ending");
        if (line == "END CONSTS") break;
        log_constant(line);
    }

    if (!std::getline(file, line)) std::runtime_error( "unexpected file ending");
    if (line != "BEGIN RULES") throw std::runtime_error( "expected BEGIN RULES");;

    // read rules
    while (true){

        current_rule_all_string = "";
        
        if (!std::getline(file, line)) throw std::runtime_error( "unexpected file ending");

        std::string rule_name{};
        uint crit_level{};
        Rule::TriggerPolicy trig_policy{};
        Rule::FilterPolicy filt_policy{};
        parse_header(line, rule_name);
        current_rule_all_string += line;

        if (!std::getline(file, line)) throw std::runtime_error( "unexpected file ending");
        parse_policies(line, crit_level, trig_policy, filt_policy);
        current_rule_all_string += line;

        if (!std::getline(file, line)) throw std::runtime_error( "unexpected file ending");
        auto trigger = parse_triggers(line);
        current_rule_all_string += line;

        auto strategies = parse_strategies(&file, rule_name);
        auto hash = hasher(current_rule_all_string);

        // construct and add rule
        auto new_rule = std::make_shared<Rule>(rule_name, trigger, strategies, crit_level, hash, filt_policy, trig_policy);
        if (!new_rule->sanityCheck()){
            throw std::runtime_error("rule failed sanity check: ensure strategy success rate add to 100");
        }

        rules.push_back(new_rule);
        std::cout<<"added rule: '"<<rule_name<<std::endl;

        if (!std::getline(file, line)) std::runtime_error( "unexpected file ending");
        if (line == "END RULES") break;        
    }
    file.close();
    return rules;
}

/**
 * @brief Adds constant to the black board
 * 
 * Reads the string and expects 3 words:
 * first word: type (bool, string, int, double)
 * second word: parameter name
 * third word: parameter value
 * 
 * @param line the string to parse
 * 
 * @throws Runtime exception If an unexpected type is requested
 * @throws Runtime exception If the value cannot be casted
 * @throws Runtime exception If the number of words is unexpected
 * 
 */
void RuleParser::log_constant(std::string line) const{
    auto vec = string_utils::split(line);
    if (vec.size() != 3){
        throw std::runtime_error( line + "\n Expected 3 words in constant");
    }

    if (vec[0] == "bool"){
        std::cout<<"adding bool const: " << vec[1] <<" = "<<string_utils::to_bool(vec[2]) <<std::endl;
        value_store->insert({vec[1], param_utils::getParamValue(string_utils::to_bool(vec[2]))});
    } else if (vec[0] == "string"){
        std::cout<<"adding string const: " << vec[1] <<" = "<<vec[2] <<std::endl;
        value_store->insert({vec[1], param_utils::getParamValue(vec[2])});
    } else if (vec[0] == "double"){
         std::cout<<"adding double const: " << vec[1] <<" = "<<std::stod(vec[2]) <<std::endl;
         value_store->insert({vec[1], param_utils::getParamValue(std::stod(vec[2]))});
    }else if (vec[0] == "int"){
         std::cout<<"adding int const: " << vec[1] <<" = "<<std::stoi(vec[2]) <<std::endl;
         value_store->insert({vec[1], param_utils::getParamValue(std::stoi(vec[2]))});
    }else{
        throw std::runtime_error( vec[0] + "\n Expected 'double', 'int', 'string' or 'bool'");
    }
}

/**
 * @brief Parses the first line of a rule
 * 
 * Parses rule name and trigger type 
 * Reads the string and expects 2 or 3 words:
 * first word: keyword RULE
 * second word: rule name
 * third word: rule trigger type
 * 
 * @param line the string to parse
 * 
 * @throws Runtime exception If an unexpected type is requested
 * @throws Runtime exception If the keyword 'RULE' is missing
 * @throws Runtime exception If an unexpected trigger type is requested
 * 
 */

void RuleParser::parse_header(std::string line, std::string & name) const{
    auto vec = string_utils::split(line);
    if (vec[0] != "RULE"){
        throw std::runtime_error( "read unexpected Line \n "  + line + "\nExpected Keyword 'RULE'");
    }
    if (vec.size() < 2) throw std::runtime_error("every rule needs a name");
    
    name = vec[1];
    return;
}

void RuleParser::parse_policies(std::string line, uint& crit_level,  Rule::TriggerPolicy& policy, Rule::FilterPolicy& filter) const{
    auto vec = string_utils::split(line);
    if (vec[0] != "POLICIES"){
        throw std::runtime_error( "read unexpected Line \n "  + line + "\nExpected Keyword 'POLICIES'");
    }
    if (vec.size() < 3) throw std::runtime_error("every rule needs a criticallitiy level, and trigger policy and a filter policy");

    auto crit = vec[1];
    auto trig = vec[2];
    auto filt = vec[3];
    
    if (crit == "OK") crit_level = Heartbeat::HB_STATUS_OK;
    else if(crit == "DEGRADED") crit_level = Heartbeat::HB_STATUS_DEGRADED;
    else if(crit == "FAILURE") crit_level = Heartbeat::HB_STATUS_FAILURE;
    else throw std::runtime_error(crit + " unknown trigger type, expected: OK, DEGRADED, FAILURE");
    
    policy = Rule::TriggerPolicy::ON_TICK;
    if (trig == "ON_CHANGE") policy = Rule::TriggerPolicy::ON_CHANGE;
    else if(trig == "ON_TICK") policy = Rule::TriggerPolicy::ON_TICK;
    else throw std::runtime_error(trig + " unknown trigger type, expected: ON_TRIGGER_CHANGE or ON_EVERY_TRIGGER");

    try{
        auto nof = string_utils::split(filt, '/');
        int n;
        int of;
        string_utils::to_int(nof[0],n);
        string_utils::to_int(nof[0],of);
        filter.n = n;
        filter.of = of;
    }catch(...){
        throw std::runtime_error("filter policy must be given in the form \"n/of\", e.g. 2/5");
    }


    return;
}

/**
 * @brief Parses the third line of a rule
 * 
 * Parses rule components
 * Reads the string and expects at least
 * first word: keyword IF
 * following words: conditions in brackes '()'
 * 
 * @param line the string to parse
 * 
 * @throws Runtime exception If more than one high level exists
 * @throws Runtime exception If the keyword 'IF' is missing
 * 
 * @return parsed trigger
 * 
 */
std::shared_ptr<Expression>  RuleParser::parse_triggers(std::string line) const{
    auto wr = string_utils::read_word(line);
    if (wr.first != "TRIGGER"){
        throw std::runtime_error( "read unexpected Line \n "  + line + "\nExpected Keyword 'TRIGGER'");
    }

    return expression_factory(wr.second);
}

Strategy RuleParser::parse_strategy(std::ifstream* file, const std::string & rule_name) const{
    std::string line;
    std::string def_string = rule_name;
    
    if (!std::getline(*file, line)) throw std::runtime_error( "unexpected file ending");    

    def_string+=line;
    current_rule_all_string += line;

    auto strategy_def_words = string_utils::split(line);
    if (strategy_def_words.size()!=3){
        throw std::runtime_error( "expected three words words for strategy: keyword 'STRATEGY', name, and success rate");
    }

    if (strategy_def_words[0] != "STRATEGY"){
        throw std::runtime_error( "read unexpected Line \n "  + line + "\nExpected Keyword 'STRATEGY'");
    }

    auto s_name = strategy_def_words[1];

    double success_rate{};
    if (!string_utils::to_double(strategy_def_words[2], success_rate)){
        throw std::runtime_error("Third entry of strategy (success rate) must be double!");
    }

    std::vector<Adaptation> adaptations;
    double system_impact = 0.0;
    while(string_utils::read_word(string_utils::peekLine(file)).first == "ADAPTATION"){        
        if (!std::getline(*file, line)) throw std::runtime_error( "unexpected file ending");        
        Adaptation current_adaptation = parse_adaptation(line);
        adaptations.push_back(current_adaptation);
        def_string+=line;
        current_rule_all_string += line;
        system_impact += adaptations_utils::getAdaptationExecutionEstimate(current_adaptation.getAdaptationType());
    }
    std::cout<<"parsed Strategy"<<std::endl;
    return Strategy(adaptations, s_name, hasher(def_string), success_rate, system_impact);
}

std::vector<Strategy> RuleParser::parse_strategies(std::ifstream* file, const std::string & rule_name) const{
    std::vector<Strategy> strategies; 
    while(string_utils::read_word(string_utils::peekLine(file)).first == "STRATEGY"){
        strategies.push_back(parse_strategy(file, rule_name));
    }
    return strategies;
}

/**
 * @brief Parses the fourth line of the rule
 * 
 * Parses the Adaptation to trigger, using the adaptation factory
 * Reads the string and expects 2 or 4 words:
 * first word: keyword THEN
 * second word: adaptation type
 * third + fourth word: adaptation specification 
 * 
 * @param line the string to parse
 * 
 * @throws Runtime exception If first word is not keyword "THEN"
 * 
 * @returns Adaptaion generating function
 * 
 */
Adaptation RuleParser::parse_adaptation(std::string line) const{
    auto wr = string_utils::read_word(line);
    if (wr.first != "ADAPTATION"){
        throw std::runtime_error( "read unexpected Line \n "  + line + "\nExpected Keyword 'THEN'");
    }
    return adaptation_factory.produce(wr.second);
}

