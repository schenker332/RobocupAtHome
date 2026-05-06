#ifndef OPEN_DOOR_H
#define OPEN_DOOR_H

#include <ros/ros.h>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Messages
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_msgs/CollisionObject.h>

// TF
#include <tf/tf.h>
#include <tf/transform_listener.h>

// Action Server
#include <actionlib/server/simple_action_server.h>
#include <open_door/OpenDoorAction.h>

// Odometry
#include <nav_msgs/Odometry.h>

enum State
{
    WAIT_FOR_HANDLE,
    OPEN_GRIPPER,
    MOVE_PREGRASP,
    CLOSE_GRIPPER,
    MOVE_BACK,
    RELEASE_HANDLE,
    MOVE_AWAY,
    ROTATE_AWAY,
    TASK_COMPLETE,
    IDLE
};

typedef actionlib::SimpleActionServer<open_door::OpenDoorAction> OpenDoorServer;

class OpenDoor
{
public:
    OpenDoor();
    ~OpenDoor();

    bool initialize(ros::NodeHandle& nh);
    void run(); 

private:
    // Action Callback
    void executeCB(const open_door::OpenDoorGoalConstPtr &goal);

    // Callbacks
    void handleCentroidCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void holeCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    // Helpers
    bool controlGripper(const std::string& command);
    bool moveBaseBack(double distance);
    bool rotateBase(double angle_rad);
    bool moveArmToPosition(double x, double y, double z, double roll, double pitch, double yaw);
    bool moveToPregrasp(const geometry_msgs::PointStamped& handle);
    bool moveToStart();
    bool getPoseInMap(double& x, double& y, double& yaw);
    
    // Collision management
    void addDoorCollision(const geometry_msgs::PointStamped& handle);
    void addHandleCollision(const sensor_msgs::PointCloud2& cloud);
    void addHandleDetailCollision(const sensor_msgs::PointCloud2& cloud); // NEU: Zweites Modul
    void removeDoorCollision();
    void removeHandleCollision();

    // Logic
    bool stateMachine();

private:
    ros::NodeHandle nh_;
    
    // Action Server
    std::shared_ptr<OpenDoorServer> as_;
    open_door::OpenDoorFeedback feedback_;
    open_door::OpenDoorResult result_;
    
    // ROS Communication
    ros::Subscriber handle_centroid_sub_;
    ros::Subscriber hole_cloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber joint_state_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher gripper_cmd_pub_;
    
    // MoveIt
    moveit::planning_interface::MoveGroupInterface* move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    
    // TF
    tf::TransformListener tf_listener_;
    
    // State & Data
    State current_state_;
    std::vector<double> start_joint_values_;
    
    // Thread Safety
    std::atomic<bool> handle_detected_;
    std::mutex handle_mutex_; 
    geometry_msgs::PointStamped latest_handle_centroid_;

    std::atomic<bool> hole_cloud_received_;
    std::mutex hole_cloud_mutex_;
    sensor_msgs::PointCloud2 latest_hole_cloud_;
    
    std::atomic<bool> joint_state_received_;
    std::mutex joint_state_mutex_;
    sensor_msgs::JointState latest_joint_state_;

    // Parameters
    double pregrasp_distance_;
    double move_back_distance_;
    std::string base_frame_;
    std::string planning_group_;
};

#endif // OPEN_DOOR_H