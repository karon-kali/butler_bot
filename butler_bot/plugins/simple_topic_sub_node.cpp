#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <string>
#include <memory>

class SimpleTopicSubscriberBTNode : public BT::StatefulActionNode
{
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    std::string last_received_message_;
    bool new_message_received_;
    std::string topic_name_;
    std::string expected_value_;

public:
    SimpleTopicSubscriberBTNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), new_message_received_(false)
    {
        // Initialize ROS2 if not already done
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name", "Name of the ROS topic to subscribe to"),
            BT::InputPort<std::string>("expected_value", "Expected message value"),
            BT::OutputPort<std::string>("received_message", "Last received message")
        };
    }

    BT::NodeStatus onStart() override
    {
        // Get topic name and expected value
        if (!getInput<std::string>("topic_name", topic_name_))
        {
            throw BT::RuntimeError("Missing required input [topic_name]");
        }

        if (!getInput<std::string>("expected_value", expected_value_))
        {
            throw BT::RuntimeError("Missing required input [expected_value]");
        }

        // Create node and subscriber
        node_ = rclcpp::Node::make_shared("bt_simple_subscriber_" + name());
        
        subscriber_ = node_->create_subscription<std_msgs::msg::String>(
            topic_name_, 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->messageCallback(msg);
            });

        RCLCPP_INFO(node_->get_logger(), "Subscribed to topic: %s, waiting for: %s", 
                   topic_name_.c_str(), expected_value_.c_str());
        
        // Reset state
        new_message_received_ = false;
        last_received_message_.clear();

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // Spin to process callbacks
        rclcpp::spin_some(node_);

        // Check if we have a new message that matches expected value
        if (new_message_received_)
        {
            // Set output port
            setOutput("received_message", last_received_message_);
            
            if (last_received_message_ == expected_value_)
            {
                RCLCPP_INFO(node_->get_logger(), "Received expected message: '%s'", last_received_message_.c_str());
                
                // Reset flag
                new_message_received_ = false;
                
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(), "Received '%s', but expected '%s'", 
                           last_received_message_.c_str(), expected_value_.c_str());
                
                // Reset flag and keep waiting
                new_message_received_ = false;
                
                return BT::NodeStatus::RUNNING;
            }
        }

        // Keep waiting
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        if (node_)
        {
            RCLCPP_INFO(node_->get_logger(), "SimpleTopicSubscriber node halted: %s", name().c_str());
        }
        subscriber_.reset();
        node_.reset();
    }

private:
    void messageCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        last_received_message_ = msg->data;
        new_message_received_ = true;
        RCLCPP_INFO(node_->get_logger(), "Callback received: '%s'", msg->data.c_str());
    }
};

#include "behaviortree_cpp_v3/bt_factory.h"

// Register the node
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<SimpleTopicSubscriberBTNode>("SimpleTopicSubscriber");
}