#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <iostream>
#include <string>
#include <memory>
#include <mutex>
#include <chrono>
#include <unordered_map>

class TopicSubscriberBTNode : public BT::StatefulActionNode
{
private:
    // Static shared data across all instances
    static std::unordered_map<std::string, rclcpp::Node::SharedPtr> shared_nodes_;
    static std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> shared_subscribers_;
    static std::unordered_map<std::string, std::string> shared_values_;
    static std::unordered_map<std::string, bool> shared_data_received_;
    static std::unordered_map<std::string, std::chrono::steady_clock::time_point> shared_timestamps_;
    static std::mutex shared_mutex_;
    
    std::string topic_name_;
    std::string message_type_;
    const std::chrono::milliseconds MESSAGE_TIMEOUT{10000}; // 10 second timeout

public:
    TopicSubscriberBTNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
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
            BT::InputPort<std::string>("message_type", "Type of message (string, float, int)"),
            BT::InputPort<std::string>("expected_value", "Value to compare against"),
            BT::OutputPort<std::string>("current_value", "Current value from topic")
        };
    }

    BT::NodeStatus onStart() override
    {
        std::string topic_name;
        std::string message_type;

        // Get input parameters
        if (!getInput<std::string>("topic_name", topic_name))
        {
            throw BT::RuntimeError("Missing required input [topic_name]");
        }

        if (!getInput<std::string>("message_type", message_type))
        {
            throw BT::RuntimeError("Missing required input [message_type]");
        }

        topic_name_ = topic_name;
        message_type_ = message_type;

        // Initialize shared subscriber if not already done
        std::lock_guard<std::mutex> lock(shared_mutex_);
        std::string key = topic_name + "_" + message_type;
        
        if (shared_nodes_.find(key) == shared_nodes_.end())
        {
            initializeSharedSubscriber(key);
        }

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        std::string expected_value;

        if (!getInput<std::string>("expected_value", expected_value))
        {
            throw BT::RuntimeError("Missing required input [expected_value]");
        }

        std::string key = topic_name_ + "_" + message_type_;

        // Spin the shared node to process callbacks
        {
            std::lock_guard<std::mutex> lock(shared_mutex_);
            if (shared_nodes_[key])
            {
                rclcpp::spin_some(shared_nodes_[key]);
            }
        }

        // Check if we have received data
        std::lock_guard<std::mutex> lock(shared_mutex_);
        
        if (!shared_data_received_[key])
        {
            RCLCPP_INFO_THROTTLE(shared_nodes_[key]->get_logger(), *shared_nodes_[key]->get_clock(), 1000, 
                                "Waiting for data on topic: %s (Node: %s)", topic_name_.c_str(), name().c_str());
            return BT::NodeStatus::RUNNING;
        }

        // Check if message is too old
        auto now = std::chrono::steady_clock::now();
        if (now - shared_timestamps_[key] > MESSAGE_TIMEOUT)
        {
            RCLCPP_INFO(shared_nodes_[key]->get_logger(), "Message too old, clearing");
            shared_data_received_[key] = false;
            shared_values_[key].clear();
            return BT::NodeStatus::RUNNING;
        }

        std::string current_message = shared_values_[key];
    
    // Set output port with current value
    setOutput("current_value", current_message);
    
    // ALWAYS clear the message after reading it (consume it)
    shared_data_received_[key] = false;
    shared_values_[key].clear();

    // Compare with expected value
    if (current_message == expected_value)
    {
        RCLCPP_INFO(shared_nodes_[key]->get_logger(), "Topic value matches expected value: %s (Node: %s)", 
                   expected_value.c_str(), name().c_str());
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_INFO(shared_nodes_[key]->get_logger(), "Topic value (%s) does not match expected value (%s) (Node: %s)", 
                   current_message.c_str(), expected_value.c_str(), name().c_str());
        return BT::NodeStatus::FAILURE;
    }
    }

    void onHalted() override
    {
        // Clean up if needed when the node is halted
        std::string key = topic_name_ + "_" + message_type_;
        std::lock_guard<std::mutex> lock(shared_mutex_);
        if (shared_nodes_[key])
        {
            RCLCPP_INFO(shared_nodes_[key]->get_logger(), "TopicSubscriber node halted: %s", name().c_str());
        }
    }

private:
    void initializeSharedSubscriber(const std::string& key)
    {
        std::string sanitized_key = key;
        std::replace(sanitized_key.begin(), sanitized_key.end(), '/', '_');
    
        // Create shared node with sanitized name
        shared_nodes_[key] = rclcpp::Node::make_shared("bt_topic_subscriber_" + sanitized_key);
        
        // Reset shared state
        shared_data_received_[key] = false;
        shared_values_[key].clear();

        // Subscribe based on message type
        if (message_type_ == "string")
        {
            shared_subscribers_[key] = shared_nodes_[key]->create_subscription<std_msgs::msg::String>(
                topic_name_, 10,
                [this, key](const std_msgs::msg::String::SharedPtr msg) {
                    this->stringCallback(msg, key);
                });
        }
        else if (message_type_ == "float")
        {
            shared_subscribers_[key] = shared_nodes_[key]->create_subscription<std_msgs::msg::Float64>(
                topic_name_, 10,
                [this, key](const std_msgs::msg::Float64::SharedPtr msg) {
                    this->floatCallback(msg, key);
                });
        }
        else if (message_type_ == "int")
        {
            shared_subscribers_[key] = shared_nodes_[key]->create_subscription<std_msgs::msg::Int32>(
                topic_name_, 10,
                [this, key](const std_msgs::msg::Int32::SharedPtr msg) {
                    this->intCallback(msg, key);
                });
        }
        else
        {
            throw BT::RuntimeError("Unsupported message type: " + message_type_ + 
                                 ". Supported types: string, float, int");
        }

        RCLCPP_INFO(shared_nodes_[key]->get_logger(), "Subscribed to topic: %s with message type: %s", 
                   topic_name_.c_str(), message_type_.c_str());
    }

    void stringCallback(const std_msgs::msg::String::SharedPtr msg, const std::string& key)
    {
        std::lock_guard<std::mutex> lock(shared_mutex_);
        shared_values_[key] = msg->data;
        shared_data_received_[key] = true;
        shared_timestamps_[key] = std::chrono::steady_clock::now();
        
        RCLCPP_INFO(shared_nodes_[key]->get_logger(), "Received string message: '%s'", msg->data.c_str());
    }

    void floatCallback(const std_msgs::msg::Float64::SharedPtr msg, const std::string& key)
    {
        std::lock_guard<std::mutex> lock(shared_mutex_);
        shared_values_[key] = std::to_string(msg->data);
        shared_data_received_[key] = true;
        shared_timestamps_[key] = std::chrono::steady_clock::now();
        
        RCLCPP_INFO(shared_nodes_[key]->get_logger(), "Received float message: %f", msg->data);
    }

    void intCallback(const std_msgs::msg::Int32::SharedPtr msg, const std::string& key)
    {
        std::lock_guard<std::mutex> lock(shared_mutex_);
        shared_values_[key] = std::to_string(msg->data);
        shared_data_received_[key] = true;
        shared_timestamps_[key] = std::chrono::steady_clock::now();
        
        RCLCPP_INFO(shared_nodes_[key]->get_logger(), "Received int message: %d", msg->data);
    }
};

// Define static members
std::unordered_map<std::string, rclcpp::Node::SharedPtr> TopicSubscriberBTNode::shared_nodes_;
std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> TopicSubscriberBTNode::shared_subscribers_;
std::unordered_map<std::string, std::string> TopicSubscriberBTNode::shared_values_;
std::unordered_map<std::string, bool> TopicSubscriberBTNode::shared_data_received_;
std::unordered_map<std::string, std::chrono::steady_clock::time_point> TopicSubscriberBTNode::shared_timestamps_;
std::mutex TopicSubscriberBTNode::shared_mutex_;

#include "behaviortree_cpp_v3/bt_factory.h"

// Register the node
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<TopicSubscriberBTNode>("TopicSubscriber");
}