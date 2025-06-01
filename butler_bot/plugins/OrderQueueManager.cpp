#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <memory>

class OrderQueueManagerBTNode : public BT::SyncActionNode
{
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr queue_publisher_;
    static bool ros_initialized_;

public:
    OrderQueueManagerBTNode(const std::string &name, const BT::NodeConfiguration &config)
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
        node_ = rclcpp::Node::make_shared("bt_order_queue_manager");
        
        // Create publisher for queue updates
        queue_publisher_ = node_->create_publisher<std_msgs::msg::String>("/butler/table_queue", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("order_type", "single or multiple"),
            BT::InputPort<std::string>("table_list", "Comma-separated list of table IDs (e.g., '1,3,5')"),
            BT::OutputPort<int>("queue_size", "Number of tables in the queue"),
            BT::OutputPort<std::string>("queue_status", "Queue initialization status")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string order_type;
        std::string table_list;

        // Get input parameters
        if (!getInput<std::string>("order_type", order_type))
        {
            throw BT::RuntimeError("Missing required input [order_type]");
        }

        if (!getInput<std::string>("table_list", table_list))
        {
            throw BT::RuntimeError("Missing required input [table_list]");
        }

        try
        {
            // Parse table list
            std::vector<std::string> tables;
            if (!table_list.empty())
            {
                size_t start = 0;
                size_t end = table_list.find(',');
                
                while (end != std::string::npos)
                {
                    tables.push_back(table_list.substr(start, end - start));
                    start = end + 1;
                    end = table_list.find(',', start);
                }
                tables.push_back(table_list.substr(start));
            }

            // Validate order type and table list consistency
            if (order_type == "single" && tables.size() != 1)
            {
                RCLCPP_ERROR(node_->get_logger(), "Single order type requires exactly one table, got %zu", tables.size());
                setOutput("queue_status", "error");
                return BT::NodeStatus::FAILURE;
            }

            if (order_type == "multiple" && tables.size() <= 1)
            {
                RCLCPP_ERROR(node_->get_logger(), "Multiple order type requires more than one table, got %zu", tables.size());
                setOutput("queue_status", "error");
                return BT::NodeStatus::FAILURE;
            }

            // Publish each table to the queue
            for (const auto& table : tables)
            {
                auto msg = std::make_unique<std_msgs::msg::String>();
                msg->data = "table:" + table;
                queue_publisher_->publish(std::move(msg));
                
                RCLCPP_INFO(node_->get_logger(), "Added table %s to delivery queue", table.c_str());
            }

            // Spin once to ensure messages are sent
            rclcpp::spin_some(node_);

            // Set output ports
            setOutput("queue_size", static_cast<int>(tables.size()));
            setOutput("queue_status", "initialized");

            RCLCPP_INFO(node_->get_logger(), "Order queue initialized: %s order with %zu tables", 
                       order_type.c_str(), tables.size());

            return BT::NodeStatus::SUCCESS;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Error initializing order queue: %s", e.what());
            setOutput("queue_status", "error");
            return BT::NodeStatus::FAILURE;
        }
    }
};

// Static member initialization
bool OrderQueueManagerBTNode::ros_initialized_ = false;

#include "behaviortree_cpp_v3/bt_factory.h"

// Register the node
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<OrderQueueManagerBTNode>("OrderQueueManager");
}