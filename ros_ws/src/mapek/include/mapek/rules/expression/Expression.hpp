#pragma once

#include <string>
#include <array>
#include <vector>
#include <queue>
#include <map>
#include <memory>
#include <regex>
#include "mapek/rules/expression/Token.hpp"
#include "mapek/util/definitions.hpp"

class Expression{
    public:
        Expression(std::shared_ptr<Expression> left, std::shared_ptr<Expression> right, Token op);
        Expression(Token value);
        Token evaluate(ValueStorePtr value_store) const;
    private:
        Token compute(const Token&, const Token&, const Token&, ValueStorePtr) const;
        bool is_leaf{};
        std::shared_ptr<Expression> left;
        std::shared_ptr<Expression> right;
        Token opr;
        Token value;
};