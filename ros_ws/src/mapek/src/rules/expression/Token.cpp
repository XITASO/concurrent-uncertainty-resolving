#include <stdexcept>
#include <cassert>
#include <iostream>
#include <cmath>
#include "mapek/util/string_utils.hpp"
#include "mapek/util/parameter_message_utils.hpp"
#include "mapek/rules/expression/Token.hpp" 

Token::Type tokenTypeFromParamType(uint ptype){
    switch (ptype){
        case(ParamType::PARAMETER_DOUBLE):
            return Token::Type::NUMBER;
        case(ParamType::PARAMETER_INTEGER):
            return Token::Type::NUMBER;
        case(ParamType::PARAMETER_BOOL):
            return Token::Type::BOOL;
        case(ParamType::PARAMETER_STRING):
            return Token::Type::STRING;
        default:
            throw std::runtime_error("unsupport param type");
    }
}

template <> double Token::getValue(ValueStorePtr value_store) const{
    auto val = resolveValue(value_store);
    double a;
    if (string_utils::to_double(val, a)){
        return a;}
    throw std::runtime_error("Variable cannot be casted");
}

template <> std::string Token::getValue(ValueStorePtr value_store) const{
    auto val = resolveValue( value_store);
    return val;
} 

template <> bool Token::getValue(ValueStorePtr value_store) const{
    auto val = resolveValue(value_store);
    bool a;
    if (string_utils::to_bool(val, a)){
        return a;
    }
    throw std::runtime_error("Variable cannot be casted");
} 

Token::Token(std::string value, Type type):value(value),type(type){
    // little hacky: assert, that the string representation has the same number of digits
    if (type == Token::Type::NUMBER){
        this->value = string_utils::to_string(getValue<double>());
    }
}

Token::Type Token::getType(ValueStorePtr value_store) const{
    if (type != Token::Type::VAR || value_store == nullptr)
        return type;
    return tokenTypeFromParamType(value_store->at(value).type);
}

bool Token::isOperator() const{
    return(
        getType() == Type::OPERATOR ||
        getType() == Type::UNARY_OPERATOR
    );
}

std::string Token::resolveValue(ValueStorePtr value_store) const{
    if (type == Token::Type::VAR){
        if (value_store == nullptr)
            throw std::runtime_error("Token is variable, but no value store provided");
        if (!value_store->count(value))
            throw std::runtime_error("No value with this name in value store");
        return param_utils::getDataAsString(value_store->at(value));
    }
    return value;
}