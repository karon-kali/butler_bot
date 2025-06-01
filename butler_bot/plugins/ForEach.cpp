#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/control_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <mutex>

class ForEachBTNode : public BT::ControlNode
{
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr queue_subscriber_;
    std::vector<std::string> table_queue_;
    std::mutex queue_mutex_;
    size_t current_index_;
    bool queue_initialized_;
    static bool ros_initialized_;

public:
    ForEachBTNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ControlNode(name, config), current_index_(0), queue_initialized_(false)
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
        node_ = rclcpp::Node::make_shared("bt_for_each_node");
        
        // Create subscriber for queue updates
        queue_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
            "/butler/table_queue", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->queueCallback(msg);
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("queue_topic", "/butler/table_queue", "Topic to subscribe for queue items"),
            BT::OutputPort<std::string>("current_table", "Current table being processed"),
            BT::OutputPort<int>("current_index", "Current iteration index"),
            BT::OutputPort<int>("total_items", "Total number of items in queue"),
            BT::OutputPort<std::string>("current_table_pose", "Pose for current table navigation")
        };
    }

    BT::NodeStatus tick() override
    {
        // Spin to process any pending messages
        rclcpp::spin_some(node_);

        std::lock_guard<std::mutex> lock(queue_mutex_);

        // Wait for queue to be populated
        if (!queue_initialized_ || table_queue_.empty())
        {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                "Waiting for table queue to be populated...");
            return BT::NodeStatus::RUNNING;
        }

        // Check if we've processed all items
        if (current_index_ >= table_queue_.size())
        {
            RCLCPP_INFO(node_->get_logger(), "Completed all table deliveries");
            current_index_ = 0; // Reset for next run
            return BT::NodeStatus::SUCCESS;
        }

        // Set current table information
        std::string current_table = table_queue_[current_index_];
        setOutput("current_table", current_table);
        setOutput("current_index", static_cast<int>(current_index_));
        setOutput("total_items", static_cast<int>(table_queue_.size()));

        // Generate pose for current table (simplified - you might want more complex logic)
        std::string table_pose = generateTablePose(current_table);
        setOutput("current_table_pose", table_pose);

        RCLCPP_INFO(node_->get_logger(), "Processing table %s (%zu/%zu)", 
                   current_table.c_str(), current_index_ + 1, table_queue_.size());

        // Execute child node for current table
        if (children().size() > 0)
        {
            BT::NodeStatus child_status = children()[0]->executeTick();
            
            if (child_status == BT::NodeStatus::SUCCESS)
            {
                // Move to next table
                current_index_++;
                RCLCPP_INFO(node_->get_logger(), "Successfully completed delivery to %s", current_table.c_str());
                
                // Continue with next iteration
                return BT::NodeStatus::RUNNING;
            }
            else if (child_status == BT::NodeStatus::FAILURE)
            {
                // Child failed, but we continue with next table
                RCLCPP_WARN(node_->get_logger(), "Failed delivery to %s, continuing with next table", current_table.c_str());
                current_index_++;
                return BT::NodeStatus::RUNNING;
            }
            else // RUNNING
            {
                return BT::NodeStatus::RUNNING;
            }
        }

        // No child nodes
        current_index_++;
        return BT::NodeStatus::RUNNING;
    }

    void halt() override
    {
        BT::ControlNode::halt();
        RCLCPP_INFO(node_->get_logger(), "ForEach node halted");
    }

private:
    void queueCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        // Add new table to queue if it's not already there
        if (std::find(table_queue_.begin(), table_queue_.end(), msg->data) == table_queue_.end())
        {
            table_queue_.push_back(msg->data);
            queue_initialized_ = true;
            RCLCPP_INFO(node_->get_logger(), "Added %s to queue (total: %zu)", 
                       msg->data.c_str(), table_queue_.size());
        }
    }

    std::string generateTablePose(const std::string& table_id)
    {
        // Extract table number from "table:X" format
        std::string table_num = table_id;
        if (table_id.find("table:") == 0)
        {
            table_num = table_id.substr(6);
        }

        // Simple pose generation based on table number
        // In a real implementation, you'd have a lookup table or service
        try
        {
            int table_number = std::stoi(table_num);
            double x = 2.0 + (table_number % 3) * 2.0;  // Arrange tables in grid
            double y = 2.0 + (table_number / 3) * 2.0;
            
            return "{position: {x: " + std::to_string(x) + ", y: " + std::to_string(y) + ", z: 0.0}, "
                   "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}";
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(), "Could not parse table number from %s, using default pose", table_id.c_str());
            return "{position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}";
        }
    }
};

// Static member initialization
bool ForEachBTNode::ros_initialized_ = false;

#include "behaviortree_cpp_v3/bt_factory.h"

// Register the node
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<ForEachBTNode>("ForEach");
}