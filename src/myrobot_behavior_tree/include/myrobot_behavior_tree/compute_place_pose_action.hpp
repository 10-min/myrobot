#ifndef COMPUTE_PLACE_POSE_ACTION_HPP_
#define COMPUTE_PLACE_POSE_ACTION_HPP_

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "myrobot_interfaces/action/compute_place_pose.hpp"
#include <string>

class ComputePlacePoseAction : public nav2_behavior_tree::BtActionNode<myrobot_interfaces::action::ComputePlacePose> {
private:
    using Action = myrobot_interfaces::action::ComputePlacePose;
    using ActionResult = Action::Result;
public:

    ComputePlacePoseAction(
        const std::string &xml_tag_name,
        const std::string &action_name,
        const BT::NodeConfiguration &conf);

    void on_tick() override;

    BT::NodeStatus on_success() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
            {
                BT::InputPort<std::string>("goal_place", "", "Goal Place"),
                BT::OutputPort<ActionResult::_goal_type>("goal", "Goal pose")
            });
    }
};

#endif