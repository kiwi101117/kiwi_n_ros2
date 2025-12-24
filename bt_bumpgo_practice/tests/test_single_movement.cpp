#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "rclcpp/rclcpp.hpp"
#include <unordered_map>
#include <string>

int main(int argc, char * argv[])
{
    std::unordered_map<std::string, std::string> nameAndNode = {
        {"Forward", "br2_forward_bt_node"},
        {"Back", "br2_back_bt_node"},
        {"Turn", "br2_turn_bt_node"}
    };

    std::string toTest = "Back";
    
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared(toTest + "_node");
    
    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;

    factory.registerFromPlugin(loader.getOSName(nameAndNode[toTest]));

    std::string xml_bt = 
        R"(
        <?xml version="1.0"?>
        <root main_tree_to_execute="BehaviorTree">
            <BehaviorTree ID="BehaviorTree">
                <Action ID=")" + toTest;
    xml_bt += R"("/>
            </BehaviorTree>
        </root>)";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

    auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10);

    rclcpp::Rate rate(10);
    bool finish = false;
    while (!finish && rclcpp::ok())
    {
        finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}