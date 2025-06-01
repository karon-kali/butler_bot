#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <string>
#include <memory>

class RemoveFromQueueBTNode : public BT::SyncActionNode
{
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr removal_publisher_;
    static bool ros_initialized_;

public:
    RemoveFromQueueBTNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        // Initialize ROS2 if not already done
        if (!ros_initialized_)
        {
            if (!rclcpp::ok())
            {
                rclcpp::init(0, nullptr);
            }
            ros_initialized_ = true;
        }
        
        // Create a node for this BT node
        node_ = rclcpp::Node::make_shared("bt_remove_from_queue");
        
        // Create publisher for queue removal notifications
        removal_publisher_ = node_->create_publisher<std_msgs::msg::String>("/butler/table_queue_remove", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("queue_topic", "/butler/table_queue", "Topic name for the queue"),
            BT::InputPort<std::string>("item_to_remove", "Specific item to remove from queue"),
            BT::InputPort<std::string>("removal_strategy", "current", "Strategy: 'current' or 'specific'"),
            BT::OutputPort<std::string>("removal_status", "Status of removal operation"),
            BT::OutputPort<std::string>("removed_item", "Item that was removed")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string queue_topic;
        std::string removal_strategy;
        std::string item_to_remove;

        // Get input parameters
        if (!getInput<std::string>("queue_topic", queue_topic))
        {
            queue_topic = "/butler/table_queue"; // Default value
        }

        if (!getInput<std::string>("removal_strategy", removal_strategy))
        {
            removal_strategy = "current"; // Default strategy
        }

        try
        {
            std::string target_item;

            if (removal_strategy == "current")
            {
                // Try to get current table from blackboard or parent context
                // This would typically be set by the ForEach node
                if (config().blackboard->get<std::string>("current_table", target_item))
                {
                    RCLCPP_INFO(node_->get_logger(), "Removing current table from queue: %s", target_item.c_str());
                }
                else
                {
                    RCLCPP_WARN(node_->get_logger(), "No current table found in blackboard for removal");
                    setOutput("removal_status", "no_current_item");
                    return BT::NodeStatus::FAILURE;
                }
            }
            else if (removal_strategy == "specific")
            {
                if (!getInput<std::string>("item_to_remove", item_to_remove))
                {
                    throw BT::RuntimeError("Missing required input [item_to_remove] for specific removal strategy");
                }
                target_item = item_to_remove;
                RCLCPP_INFO(node_->get_logger(), "Removing specific item from queue: %s", target_item.c_str());
            }
            else
            {
                throw BT::RuntimeError("Invalid removal strategy: " + removal_strategy + ". Use 'current' or 'specific'");
            }

            // Publish removal request
            auto removal_msg = std::make_unique<std_msgs::msg::String>();
            removal_msg->data = "remove:" + target_item;
            removal_publisher_->publish(std::move(removal_msg));

            // Spin to ensure message is sent
            rclcpp::spin_some(node_);

            // Set output ports
            setOutput("removed_item", target_item);
            setOutput("removal_status", "removed");

            RCLCPP_INFO(node_->get_logger(), "Successfully requested removal of %s from queue", target_item.c_str());

            return BT::NodeStatus::SUCCESS;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Error removing item from queue: %s", e.what());
            setOutput("removal_status", "error");
            return BT::NodeStatus::FAILURE;
        }
    }
};

// Static member initialization
bool RemoveFromQueueBTNode::ros_initialized_ = false;

#include "behaviortree_cpp_v3/bt_factory.h"

// Register the node
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<RemoveFromQueueBTNode>("RemoveFromQueue");
}