#include "myrobot_behavior_tree/compute_place_pose_action.hpp"

ComputePlacePoseAction::ComputePlacePoseAction(
    const std::string &xml_tag_name,
    const std::string &action_name,
    const BT::NodeConfiguration &conf)
    : nav2_behavior_tree::BtActionNode<myrobot_interfaces::action::ComputePlacePose>(xml_tag_name, action_name, conf)
{

    std::string place = "";
    //getInput("goal_place", place);
    goal_.place = place;
}

void ComputePlacePoseAction::on_tick()
{
    increment_recovery_count();
}

BT::NodeStatus ComputePlacePoseAction::on_success()
{
    geometry_msgs::msg::PoseStamped output_pose;
    output_pose.header.frame_id = "map";
    output_pose.pose = result_.result->goal.pose;
    setOutput("goal", output_pose);
    return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
        [](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<ComputePlacePoseAction>(name, "compute_place_pose", config);
    };

    factory.registerBuilder<ComputePlacePoseAction>("ComputePlacePose", builder);
}