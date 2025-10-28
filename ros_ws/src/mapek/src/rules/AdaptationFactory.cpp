#include "mapek/rules/AdaptationFactory.hpp"
#include "mapek/util/adaptation_utils.hpp"
#include "system_interfaces/msg/adaptation_type.hpp"
#include "mapek/util/parameter_message_utils.hpp"

#include <stdexcept>

/**
 * @brief Creates a adaptation from a given vecor of strings
 * 
 *  the first entry of the vector holds the adaption's type 
 *  the second entry of the vector holds the adaption's parameter name
 *  the third entry of the vector holds the adaption's parameter value as blackboard key
 * 
 * @param vec vector of strings instructions on how to build the adaptation
 * @param value_store value store pointer to be captured by the adaptaion lambda
 * 
 * @return Adaptation Lambda function that returns the adaptation with the current value
 */
Adaptation AdaptationFactory::produce(std::string vec) const {
    auto component_vec = string_utils::read_word(vec);
    auto component = component_vec.first;
    vec = component_vec.second;

    auto type_vec = string_utils::read_word(vec);
    auto type  = adaptations_utils::deduceAdaptationType(type_vec.first);
    if (   type == AdaptationType::ACTION_ACTIVATE
        || type == AdaptationType::ACTION_DEACTIVATE
        || type == AdaptationType::ACTION_RESTART
        || type == AdaptationType::ACTION_REDEPLOY
    ){
        if (type_vec.second != ""){
            throw std::runtime_error("AdaptationFactory. Unnecessary parameter was given to simple adaptation. Please update the rules file!");
        }
        return Adaptation (component, type, produceNoParamAdaptation(type));
    }

    return Adaptation (component, type, produceParametrizedAdaptation(type, type_vec.second));
}

/**
 * @brief Creates a adaptation without parameters
 * 
 * @param type type of the adaptation
 * 
 * @return Adaptation Lambda function that statically returns the adaptation of a given type
 */
AdaptationGenerator AdaptationFactory::produceNoParamAdaptation(uint8_t type) const {
    return [type](ValueStorePtr value_store)-> GenericAdaptation {
        GenericAdaptation msg;
        msg.action = type;
        return msg;
    };
}

/**
 * @brief Creates an adaptation with parameters
 * 
 * builds adynamic adaptation function from a string vector:
 * the second entry of the vector holds the adaption's parameter name
 * the third entry of the vector holds the adaption's parameter value as blackboard key
 * 
 * @param type type of the adaptation
 * @param vec vector of strings instructions on how to build the adaptation
 * @param value_store value store pointer to be captured by the adaptaion lambda
 * 
 * @return Adaptation Lambda function that dynamically fills the adaptation value with the current blackboard entry
 */
AdaptationGenerator AdaptationFactory::produceParametrizedAdaptation(uint8_t type, std::string vec) const {
    auto vlist = string_utils::split(vec);
    if (type == AdaptationType::ACTION_CHANGE_MODE && vlist.size() != 1)
        throw std::runtime_error("AdaptationFactory.produceParametrizedAdaptation: Expected two arguments for change mode adaptation. Please update the rules file:" + vec);
    else if (type != AdaptationType::ACTION_CHANGE_MODE && vlist.size() < 2)
        throw std::runtime_error("AdaptationFactory.produceParametrizedAdaptation: Expected three arguments in adaptation. Please update the rules file:" + vec);

    if (type == AdaptationType::ACTION_CHANGE_MODE){
        return [type, vec](ValueStorePtr value_store)-> GenericAdaptation {
            GenericAdaptation msg;
            msg.action = type;
            msg.parameter.value.type =ParamType::PARAMETER_STRING;
            msg.parameter.value.string_value = vec;
            return msg;
        };
    }

    auto para_name_expression = string_utils::read_word(vec);
    auto para_name = para_name_expression.first;
    auto expression = expression_factory(para_name_expression.second);


    return [type, para_name, expression](ValueStorePtr value_store)-> GenericAdaptation {

        GenericAdaptation msg;
        msg.action = type;
        msg.parameter.name = para_name;

        // intentional no catch: no silent errors
        auto token = expression->evaluate(value_store);
        auto ttype = token.getType(value_store);

        switch(ttype){
            case Token::Type::STRING:
                msg.parameter.value = param_utils::getParamValue(token.getValue<std::string>(value_store));
                break;
        case Token::Type::NUMBER:
                msg.parameter.value = param_utils::getParamValue(token.getValue<double>(value_store));
                break;
        case Token::Type::BOOL:
                msg.parameter.value = param_utils::getParamValue(token.getValue<bool>(value_store));
                break;
        default:
            throw std::runtime_error("non processable token type (token to param)");
        }

        if (msg.parameter.value.type != ParamType::PARAMETER_STRING && type == AdaptationType::ACTION_CHANGE_COMMUNICATION){
            throw std::runtime_error ("for 'change communication' action only string-type parameters are supported");
        }

        if (msg.parameter.value.type == ParamType::PARAMETER_STRING){
            if (type != AdaptationType::ACTION_SET_PARAMETER && type != AdaptationType::ACTION_CHANGE_COMMUNICATION)
                throw std::runtime_error ("for string-type parameters only 'set' and 'change communication' action is supported");
        }
        if(msg.parameter.value.type == ParamType::PARAMETER_BOOL){
            if (type != AdaptationType::ACTION_SET_PARAMETER)
                throw std::runtime_error ("for bool-type parameters only 'set' action is supported");
        }

        return msg;
    };
}

