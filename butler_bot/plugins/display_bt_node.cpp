#include <behaviortree_cpp_v3/action_node.h>
#include <iostream>
#include <string>

class DisplayBTNode : public BT::SyncActionNode
{
public:
    DisplayBTNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("message"),
            BT::InputPort<std::string>("color")};
    }

    BT::NodeStatus tick() override
    {
        std::string message;
        std::string color;

        if (!getInput<std::string>("message", message))
        {
            throw BT::RuntimeError("Missing required input [message]");
        }

        if (!getInput<std::string>("color", color))
        {
            throw BT::RuntimeError("Missing required input [color]");
        }

        std::string color_code;
        if (color == "red")
        {
            color_code = "\033[31m"; // Red color
        }
        else if (color == "green")
        {
            color_code = "\033[32m"; // Green color
        }
        else if (color == "blue")
        {
            color_code = "\033[34m"; // Blue color
        }
        else
        {
            color_code = "\033[0m"; // Default
        }

        // Print the message with the chosen color
        std::cout << color_code << message << "\033[0m" << std::endl;

        return BT::NodeStatus::SUCCESS;
    }
};

#include "behaviortree_cpp_v3/bt_factory.h"
// Register the node
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<DisplayBTNode>("Display");
}
