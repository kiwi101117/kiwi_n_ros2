#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("forward_node");

    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;

    factory.registerFromPlugin(loader.getOSName("br2_forward_bt_node"));
    
    std::string xml_bt = 
        R"(
        <?xml version="1.0"?>
        <root main_tree_to_execute="BehaviorTree">
            <BehaviorTree ID="BehaviorTree">
                <Action ID="Forward"/>
            </BehaviorTree>
        </root>)";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

    auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

    rclcpp::Rate rate(10);
    bool finish = false;
    while (!finish && rclcpp::ok()) {
        finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}