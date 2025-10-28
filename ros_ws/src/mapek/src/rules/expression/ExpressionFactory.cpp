#include <stdexcept>
#include <cassert>
#include <iostream>
#include <cmath>
#include "mapek/rules/expression/ExpressionFactory.hpp"


std::shared_ptr<Expression> ExpressionFactory::operator() (std::string input) const{

    auto tokens = tokenize(input);
    auto rpn = toRPN(tokens);
    return evaluateRPN(rpn);
}

// implemented this code https://de.wikipedia.org/wiki/Shunting-yard-Algorithmus
std::queue<Token> ExpressionFactory::toRPN(std::queue<Token> input) const{
    std::vector<Token> operator_stack;
    std::queue<Token> output;
    Token last_token;
    bool got_last = false;
    double _;
    while (!input.empty()){
        auto token = input.front();
        input.pop();
        if (token.getType() == Token::Type::NUMBER || 
            token.getType() == Token::Type::BOOL ||
            token.getType() == Token::Type::STRING ||
            token.getType() == Token::Type::VAR
        ){
            output.push(token);
        }else if (token.getType() == Token::Type::OPERATOR){
            // check if unary operator (+-): if two consequent operators, or operator as first sign of sub expression (after parenthesis)
            if (
                (
                    token.getValue<std::string>() == "+"  ||
                    token.getValue<std::string>() == "-"
                ) && (
                    !got_last || 
                    last_token.isOperator()
                ) 
            ){ /* if so tranform it to unary operator */
                if (token.getValue<std::string>() == "+"){
                    token = Token("u+", Token::Type::UNARY_OPERATOR);
                }else if(token.getValue<std::string>() == "-"){
                    token = Token("u-", Token::Type::UNARY_OPERATOR);
                }
            }

            if (
                token.getValue<std::string>() == "!"
            ){ 
                token = Token("!", Token::Type::UNARY_OPERATOR);
            }

            while(
                operator_stack.size() && 
                operator_stack.back().isOperator() &&
                operator_rank.at(token.getValue<std::string>()) <= operator_rank.at(operator_stack.back().getValue<std::string>())
            ){
                output.push(operator_stack.back());
                operator_stack.pop_back();
            }
            operator_stack.push_back(token);
        }else if (
                token.getType() == Token::Type::PARENTHESIS && 
                token.getValue<std::string>() == "("){
            if (
                !(
                    (!got_last) ||
                    last_token.isOperator()||
                    last_token.getValue<std::string>() == "("
                    
                )
            ){
                // implicit * to the stack
                operator_stack.push_back(Token("*", Token::Type::OPERATOR));
            }
            operator_stack.push_back(token);
        }else if (
            token.getType() == Token::Type::PARENTHESIS && 
            token.getValue<std::string>() == ")"
        ){
            while(
                operator_stack.back().getValue<std::string>() != "("
            ){
                output.push(operator_stack.back());
                operator_stack.pop_back();
                if (!operator_stack.size())
                    throw std::runtime_error("missing opening bracket");
            }
            assert(operator_stack.back().getValue<std::string>() == "(");
            // std::cout<<operator_stack.back()<<std::endl;
            // remove opne bracket
            operator_stack.pop_back();
        }else{
            throw std::runtime_error("unsupported token "+token.getValue<std::string>());
        }
        last_token = token;
        got_last = true;
    }
    while(operator_stack.size()){
        if (operator_stack.back().getValue<std::string>() == "(")
            throw std::runtime_error("missing closing bracket");
        output.push(operator_stack.back());
        operator_stack.pop_back();
    }
    return output;
}

// implemented this https://de.wikipedia.org/wiki/Umgekehrte_polnische_Notation
std::shared_ptr<Expression> ExpressionFactory::evaluateRPN(std::queue<Token> rpn) const{
    std::vector<std::shared_ptr<Expression>> stack;
    std::shared_ptr<Expression> right;
    std::shared_ptr<Expression> left;
    Token token;

    while (rpn.size()){
        token = rpn.front();
        rpn.pop();
        if (
            (token.getType() == Token::Type::NUMBER) ||
            (token.getType() == Token::Type::BOOL) ||
            (token.getType() == Token::Type::STRING) ||
            (token.getType() == Token::Type::VAR)
        ){
            stack.push_back(std::make_shared<Expression>(token));
        }else if (token.getType() == Token::Type::UNARY_OPERATOR){
            if (stack.size()<1)
                throw std::runtime_error("invalid expression: unary operator has no operand");
            right = stack.back();
            stack.pop_back();
            stack.push_back(std::make_shared<Expression>(std::make_shared<Expression>(Token("", Token::Type::NONE)), right, token));
        }else if(token.getType() == Token::Type::OPERATOR){
            if (stack.size()<2)
                throw std::runtime_error("invalid expression: binary operator has less than two operand");
            right = stack.back();
            stack.pop_back();
            left = stack.back();
            stack.pop_back();
            stack.push_back(std::make_shared<Expression>(left, right, token));
        }
    }
    assert(stack.size() == 1);
    return stack[0];
}

std::queue<Token> ExpressionFactory::tokenize(std::string data) const{
    std::string rest{data};

    std::smatch match;

    std::queue<Token> tokens;

    while (!rest.empty())
    {
        bool cant_tokenize = true;
        for(const auto& rule : rules)
        {
            if(std::regex_search(rest, match, rule.rule,std::regex_constants::match_continuous))
            {
                Token token{rest.substr(0, match.length()), rule.type};
                // hack to get string match[0] also contains the quotes and cpp does not support lookahead/-behind
                if (rule.type == Token::Type::STRING){
                    token = Token(match[1], rule.type);
                }
                rest = rest.substr(match.length());

                cant_tokenize = false;

                if(rule.type != Token::Type::NONE)
                    tokens.push(token);
                break;
            }
        }

        if (cant_tokenize)
        {
            std::cerr << "can't tokenize: "<< rest << std::endl;
            break;
        }
    }
    return tokens;
}
