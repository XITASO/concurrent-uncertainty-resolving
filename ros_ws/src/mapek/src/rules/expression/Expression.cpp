#include <stdexcept>
#include <cassert>
#include <iostream>
#include <cmath>
#include "mapek/util/string_utils.hpp"
#include "mapek/rules/expression/Expression.hpp"


Expression::Expression(std::shared_ptr<Expression> left, std::shared_ptr<Expression> right, Token op):
    left(left), right(right), opr(op), is_leaf(false){
        assert(op.isOperator());
    }
Expression::Expression(Token value):
    value(value), is_leaf(true){}

Token Expression::evaluate(ValueStorePtr value_store) const{
    if(is_leaf){
        return value;
    }
    return compute(left->evaluate(value_store), right->evaluate(value_store), opr, value_store);
}

Token Expression::compute(const Token& left, const Token& right, const Token& op, ValueStorePtr value_store) const{
    // must be any type of operator
    if(op.getType(value_store) != Token::Type::OPERATOR && op.getType(value_store) != Token::Type::UNARY_OPERATOR)
        throw std::runtime_error("unsupported operator "+ op.getValue<std::string>(value_store));
    
    // first handle unary operator
    if(op.getType(value_store) == Token::Type::UNARY_OPERATOR){
        if(left.getType(value_store) != Token::Type::NONE)
            throw std::runtime_error("unary operator only takes a single input");
        auto op_value = op.getValue<std::string>(value_store);

        if (op_value == "!"){
            if (right.getType(value_store) != Token::Type::BOOL)
                throw std::runtime_error("logic operations can only be performed on bools");
            return Token(string_utils::to_string(!right.getValue<bool>(value_store)), Token::Type::BOOL);
        }

        if (op_value == "u+" || op_value == "u-"){
            if (right.getType(value_store) != Token::Type::NUMBER)
                throw std::runtime_error("unary + - operations can only be performed on numbers");
            if (op_value == "u+")
                return right;
            if (op_value == "u-")
                return Token(string_utils::to_string(-right.getValue<double>(value_store)), Token::Type::NUMBER);
        }

        throw std::runtime_error("unsupported operator: " + op_value);
    }

    // then handle binary operator
    if(left.getType(value_store) != right.getType(value_store))
        throw std::runtime_error("cannot process tokens of different types");
    auto op_value = op.getValue<std::string>(value_store);

    if (
        (
            op_value == "&&" ||
            op_value == "||"
        )
    ){
        if (left.getType(value_store) != Token::Type::BOOL)
            throw std::runtime_error("logic operations can only be performed on bools");
        bool a = left.getValue<bool>(value_store);
        bool b = right.getValue<bool>(value_store);

        if (op_value == "&&")
            return Token(string_utils::to_string(a&&b), Token::Type::BOOL);
        else if (op_value == "||")
            return Token(string_utils::to_string(a||b), Token::Type::BOOL);
    }
    if (
        // math operations
        op_value == "+"  ||
        op_value == "-"  ||
        op_value == "*"  ||
        op_value == "/"  ||
        op_value == "^"  ||
        // logic operations
        op_value == "<"  ||
        op_value == ">"  ||
        op_value == "<=" ||
        op_value == ">="
    ){
       if (left.getType(value_store) != Token::Type::NUMBER)
            throw std::runtime_error("math operations and le/gt opterations can only be performed on numbers");

        double value;
        double left_val = left.getValue<double>(value_store);
        double right_val = right.getValue<double>(value_store);
        if (op_value == "+")
            return Token(string_utils::to_string(left_val+right_val), Token::Type::NUMBER);
        if (op_value == "-")
            return Token(string_utils::to_string(left_val-right_val), Token::Type::NUMBER);
        if (op_value == "*")
            return Token(string_utils::to_string(left_val*right_val), Token::Type::NUMBER);
        if (op_value == "/")
            return Token(string_utils::to_string(left_val/right_val), Token::Type::NUMBER);
        if (op_value == "^")
            return Token(string_utils::to_string(std::pow(left_val,right_val)), Token::Type::NUMBER);
        if (op_value == "<=")
            return Token(string_utils::to_string( left_val <= right_val), Token::Type::BOOL);       
        if (op_value == ">=")
            return Token(string_utils::to_string( left_val >= right_val), Token::Type::BOOL);        
        if (op_value == "<")
            return Token(string_utils::to_string( left_val <  right_val), Token::Type::BOOL);        
        if (op_value == ">")
            return Token(string_utils::to_string( left_val >  right_val), Token::Type::BOOL);  
    }

    if (op_value == "=="){
        return Token(string_utils::to_string(left.getValue<std::string>(value_store) == right.getValue<std::string>(value_store)), Token::Type::BOOL);
    }
    if (op_value == "!="){
        return Token(string_utils::to_string(left.getValue<std::string>(value_store) != right.getValue<std::string>(value_store)), Token::Type::BOOL);
    }

 throw std::runtime_error("unsupported operator "+ op_value);

}