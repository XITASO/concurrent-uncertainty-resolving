#include <gtest/gtest.h>
#include <mapek/monitoring.hpp>

#include <map>
#include <memory>
#include "system_interfaces/msg/set_blackboard_group.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"

TEST(test_monitoring, test_update_existing_value)
{
    // Arrange
    ValueStorePtr random_value_blackboard = std::make_shared<ValueStore>();

    rcl_interfaces::msg::ParameterValue old_value;
    old_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    old_value.integer_value = 10;
    random_value_blackboard->emplace("existing_key", old_value);

    auto msg = std::make_shared<system_interfaces::msg::SetBlackboardGroup>();
    rcl_interfaces::msg::ParameterValue new_value;
    new_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    new_value.integer_value = 20;

    rcl_interfaces::msg::Parameter param_msg;
    param_msg.name = "existing_key";
    param_msg.value = new_value;
    msg->bb_params.push_back(param_msg);

    Monitoring monitoring(true);

    // Act
    monitoring.update_key_value_storage(random_value_blackboard, msg);

    // Assert
    ASSERT_EQ(random_value_blackboard->at("existing_key").integer_value, 20);
}

TEST(test_monitoring, test_add_new_value)
{
    // Arrange
    ValueStorePtr random_value_blackboard = std::make_shared<ValueStore>();
    auto msg = std::make_shared<system_interfaces::msg::SetBlackboardGroup>();
    rcl_interfaces::msg::ParameterValue new_value;
    new_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    new_value.integer_value = 30;

    rcl_interfaces::msg::Parameter param_msg;
    param_msg.name = "new_key";
    param_msg.value = new_value;
    msg->bb_params.push_back(param_msg);

    Monitoring monitoring(true);

    // Act
    monitoring.update_key_value_storage(random_value_blackboard, msg);

    // Assert
    ASSERT_EQ(random_value_blackboard->at("new_key").integer_value, 30);
    ASSERT_EQ(random_value_blackboard->size(), 1); // Ensure only one element
}