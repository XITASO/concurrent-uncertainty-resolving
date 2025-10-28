#pragma once

#include <string>
#include <array>
#include <vector>
#include <queue>
#include <regex>
#include "mapek/rules/expression/Expression.hpp"

class ExpressionFactory{
    public:
        std::shared_ptr<Expression>  operator() (std::string) const;

    private:
        struct TokenRule
        {
            TokenRule(const Token::Type& type, const std::string& rule)
                : rule{rule}, type{type}
            {}
            std::regex rule;
            Token::Type type;
        };

        //shunting yard
        std::queue<Token> tokenize(std::string) const;
        std::queue<Token> toRPN(std::queue<Token>) const;
        std::shared_ptr<Expression> evaluateRPN(std::queue<Token> rpn) const;
        
        const std::map<std::string, uint> operator_rank{
            // unary operators
            {"!", 4},
            {"u-", 4},
            {"u+", 4},
            // algorithmic operators
            {"^", 4},
            {"*", 3},
            {"/", 3},
            {"+", 2},
            {"-", 2},
            // Comparator after calculation
            {"==", 1},
            {"!=", 1},
            {"<=", 1},
            {">=", 1},
            {"<",  1},
            {">",  1},
            // Logic last
            {"||",  0},
            {"&&",  0},
        };
        
        std::array<const TokenRule, 8> rules {
            TokenRule{Token::Type::NONE, "//.*"},
            TokenRule{Token::Type::NUMBER, "\\d+(\\.\\d+)?"},
            TokenRule{Token::Type::BOOL, "(\\b(true)\\b|\\b(false)\\b)"},
            TokenRule{Token::Type::VAR, "[a-zA-Z_]\\w*"},
            TokenRule{Token::Type::OPERATOR, "(\\+|\\-|\\*|\\/|\\^|!=|!|==|>=|<=|<|>|&&|\\|\\|)"},
            TokenRule{Token::Type::PARENTHESIS, "[\\(\\)]"},
            TokenRule{Token::Type::NONE, "\\s"},
            TokenRule{Token::Type::STRING, "\"([^\"]+)\""},
        };
    };
