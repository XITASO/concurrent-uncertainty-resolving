#include <gtest/gtest.h>
#include <mapek/planning.hpp>
#include <mapek/util/definitions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mapek/rules/expression/ExpressionFactory.hpp>
#include <mapek/util/parameter_message_utils.hpp>
#include "test_utils.hpp"


TEST (test_engel_expression, test_value_store){
    ValueStorePtr value_store;
    ASSERT_NO_THROW(value_store = getValueStore());
    ASSERT_NO_THROW(param_utils::getData<double>((*value_store)["double_val"]));
    ASSERT_NO_THROW(param_utils::getData<std::string>((*value_store)["string_val"]));
    ASSERT_NO_THROW(param_utils::getData<bool>((*value_store)["bool_val"]));
}

TEST(test_engel_expression, test_token_no_fail){
    auto value_store = getValueStore();
    Token token_1("1", Token::Type::NUMBER);
    Token token_true("true", Token::Type::BOOL);
    Token token_false("false", Token::Type::BOOL);
    Token token_var1("double_val", Token::Type::VAR);
    Token token_var2("serzi", Token::Type::VAR);

    // correct casting
    ASSERT_NO_THROW(token_true.getValue<bool>());
    ASSERT_TRUE(token_true.getValue<bool>());
    ASSERT_NO_THROW(token_false.getValue<bool>());
    ASSERT_FALSE(token_false.getValue<bool>());
    ASSERT_NO_THROW(token_1.getValue<double>());
    ASSERT_EQ(token_1.getValue<double>(), 1);

    // wrong cast
    ASSERT_ANY_THROW(token_false.getValue<double>());
    ASSERT_ANY_THROW(token_1.getValue<bool>());

    // no value store, no value in store
    ASSERT_ANY_THROW(token_var1.getValue<double>());
    ASSERT_ANY_THROW(token_var2.getValue<double>(value_store));

    // correct resolving
    ASSERT_NO_THROW(token_var1.getValue<double>(value_store));
    ASSERT_EQ(token_var1.getValue<double>(value_store), 1);

    // anything can be string
    ASSERT_NO_THROW(token_1.getValue<std::string>(value_store));
    ASSERT_EQ(token_1.getValue<std::string>(value_store), "1.000000");
    ASSERT_NO_THROW(token_true.getValue<std::string>(value_store));
    ASSERT_EQ(token_true.getValue<std::string>(value_store), "true");
    ASSERT_NO_THROW(token_var1.getValue<std::string>(value_store));
    ASSERT_EQ(token_var1.getValue<std::string>(value_store), "1.000000");

}


TEST(test_engel_expression, test_simple_expression_no_value_store_no_fail){
    auto value_store = getValueStore();
    ExpressionFactory expr_fac;

    std::string def_expr0 = "((1+2) * 3) + 5^2 / 5";
    std::string def_expr1 = "(((1+2) * 3) + 5^2) / 5";
    std::string def_expr2 = "(5+2.3) > 7.3";
    std::string def_expr3 = "((5+2.3) > 7.3) || ((5 < 6) == true)";
    std::string def_expr4 = "\"Hallo\" == \"Moin\"";
    std::string def_expr5 = "(1+2)/(2(1+2)) > 1 || (1 < 2) && 1==1";
    std::string def_expr6 = "1+2==3";

    std::shared_ptr<Expression> expr0;
    ASSERT_NO_THROW(expr0 = expr_fac(def_expr0));
    std::shared_ptr<Expression> expr1;
    ASSERT_NO_THROW(expr1 = expr_fac(def_expr1));
    std::shared_ptr<Expression> expr2;
    ASSERT_NO_THROW(expr2 = expr_fac(def_expr2));
    std::shared_ptr<Expression> expr3;
    ASSERT_NO_THROW(expr3 = expr_fac(def_expr3));
    std::shared_ptr<Expression> expr4;
    ASSERT_NO_THROW(expr4 = expr_fac(def_expr4));
    std::shared_ptr<Expression> expr5;
    ASSERT_NO_THROW(expr5 = expr_fac(def_expr5));
    std::shared_ptr<Expression> expr6;
    ASSERT_NO_THROW(expr6 = expr_fac(def_expr6));

    auto result0 = expr0->evaluate(value_store);
    auto result1 = expr1->evaluate(value_store);
    auto result2 = expr2->evaluate(value_store);
    auto result3 = expr3->evaluate(value_store);
    auto result4 = expr4->evaluate(value_store);
    auto result5 = expr5->evaluate(value_store);
    auto result6 = expr6->evaluate(value_store);

    ASSERT_NO_THROW(result0.getValue<double>(value_store));
    ASSERT_EQ(result0.getValue<double>(value_store), 14.0);
    ASSERT_NO_THROW(result1.getValue<double>(value_store));
    ASSERT_EQ(result1.getValue<double>(value_store), 6.8);
    ASSERT_NO_THROW(result2.getValue<bool>(value_store));
    ASSERT_EQ(result2.getValue<bool>(value_store), false);
    ASSERT_NO_THROW(result3.getValue<bool>(value_store));
    ASSERT_EQ(result3.getValue<bool>(value_store), true);
    ASSERT_NO_THROW(result4.getValue<bool>(value_store));
    ASSERT_EQ(result4.getValue<bool>(value_store), false);
    ASSERT_NO_THROW(result5.getValue<bool>(value_store));
    ASSERT_EQ(result5.getValue<bool>(value_store), true);
    ASSERT_NO_THROW(result6.getValue<bool>(value_store));
    ASSERT_EQ(result6.getValue<bool>(value_store), true);
}

TEST(test_engel_expression, test_simple_expression_with_value_store_no_fail){
    auto value_store = getValueStore();

    ExpressionFactory expr_fac;
    std::string def_expr0 = "((1+double_val) * double_val)"; // (1+1) * 1 =2
    std::string def_expr1 = "((5 < double_val) == bool_val)";// 5 < 1 == true
    std::string def_expr2 = "string_val == \"Moin\"";

    std::shared_ptr<Expression> expr0;
    expr0 = expr_fac(def_expr0);
    ASSERT_NO_THROW(expr0 = expr_fac(def_expr0));
    std::shared_ptr<Expression> expr1;
    ASSERT_NO_THROW(expr1 = expr_fac(def_expr1));
    std::shared_ptr<Expression> expr2;
    ASSERT_NO_THROW(expr2 = expr_fac(def_expr2));

    auto result0 = expr0->evaluate(value_store);
    auto result1 = expr1->evaluate(value_store);
    auto result2 = expr2->evaluate(value_store);

    ASSERT_EQ(result0.getValue<double>(value_store), 2);
    ASSERT_EQ(result1.getValue<bool>(value_store), false);
    ASSERT_EQ(result2.getValue<bool>(value_store), false);

    // change variable values and reevaluate
    (*value_store)["double_val"] = param_utils::getParamValue(2);
    (*value_store)["string_val"] = param_utils::getParamValue("Moin");
    (*value_store)["bool_val"]   = param_utils::getParamValue(false);

    result0 = expr0->evaluate(value_store);
    result1 = expr1->evaluate(value_store);
    result2 = expr2->evaluate(value_store);

    ASSERT_EQ(result0.getValue<double>(value_store), 6);
    ASSERT_EQ(result1.getValue<bool>(value_store), true);
    ASSERT_EQ(result2.getValue<bool>(value_store), true);

}

