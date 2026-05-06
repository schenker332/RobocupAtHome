#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <point_to/PointToAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <string>

class PointToNode
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<point_to::PointToAction> as_;
    std::string action_name_;
    point_to::PointToFeedback feedback_;
    point_to::PointToResult result_;
    moveit::planning_interface::MoveGroupInterface move_group_;

public:
    PointToNode(std::string name) :
        as_(nh_, name, boost::bind(&PointToNode::executeCB, this, _1), false),
        action_name_(name),
        move_group_("arm_torso") // Assuming TIAGo's group name
    {
        as_.start();
        ROS_INFO("PointTo Action Server started.");
        
        // Settings for MoveIt
        move_group_.setMaxVelocityScalingFactor(0.5);
        move_group_.setMaxAccelerationScalingFactor(0.5);
    }

    void executeCB(const point_to::PointToGoalConstPtr &goal)
    {
        ROS_INFO("Received request for Pose ID: %d", goal->pose_id);
        bool success = true;

        // 1. Save current state
        std::vector<double> start_joints = move_group_.getCurrentJointValues();
        ROS_INFO("Current joint state saved (%lu joints).", start_joints.size());

        // 2. Load target pose from parameter server
        std::vector<double> target_joints;
        std::string param_key = "point_to_poses/pose_" + std::to_string(goal->pose_id);
        
        if (!nh_.getParam(param_key, target_joints))
        {
            ROS_ERROR("Could not load params for '%s'. Check config file!", param_key.c_str());
            result_.success = false;
            as_.setAborted(result_);
            return;
        }

        if (target_joints.size() != start_joints.size())
        {
            ROS_ERROR("Mismatch in joint count! Current: %lu, Config: %lu", start_joints.size(), target_joints.size());
            result_.success = false;
            as_.setAborted(result_);
            return;
        }

        // 3. Move to target
        feedback_.status = "Moving to target pose...";
        as_.publishFeedback(feedback_);
        ROS_INFO("%s", feedback_.status.c_str());

        move_group_.setJointValueTarget(target_joints);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        
        bool plan_success = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (plan_success)
        {
            move_group_.move();
            ROS_INFO("Target reached.");
        }
        else
        {
            ROS_ERROR("Planning to target failed!");
            result_.success = false;
            as_.setAborted(result_);
            return;
        }

        // 4. Wait briefly (simulation of 'pointing')
        ros::Duration(2.0).sleep();

        // 5. Move back to start
        feedback_.status = "Returning to start pose...";
        as_.publishFeedback(feedback_);
        ROS_INFO("%s", feedback_.status.c_str());

        move_group_.setJointValueTarget(start_joints);
        plan_success = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (plan_success)
        {
            move_group_.move();
            ROS_INFO("Returned to start.");
        }
        else
        {
            ROS_ERROR("Planning return path failed!");
            // We consider the action somewhat failed if we can't return, 
            // though the main task was done.
            result_.success = false;
            as_.setAborted(result_);
            return;
        }

        if(success)
        {
            result_.success = true;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_to_node");
    
    // MoveIt requires an AsyncSpinner
    ros::AsyncSpinner spinner(1);
    spinner.start();

    PointToNode point_to("point_to_action");

    ros::waitForShutdown();
    return 0;
}
