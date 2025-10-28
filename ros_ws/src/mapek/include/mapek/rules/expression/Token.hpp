#pragma once

#include <string>
#include "mapek/util/definitions.hpp"


class Token{
    public:
    enum class Type{
        UNARY_OPERATOR,
        OPERATOR,
        PARENTHESIS,
        VAR,
        STRING,
        NUMBER,
        BOOL,
        NONE
    };
    Token(std::string value, Type type);
    Token() = default;

    Type getType(ValueStorePtr value_store = nullptr) const;

    bool isOperator() const;

    template <typename t>
    t getValue(ValueStorePtr value_store = nullptr) const; 

    private:
    std::string resolveValue(ValueStorePtr value_store) const;
    std::string value = "";
    Type type = Type::NONE;
};