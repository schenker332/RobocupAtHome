#include <open_door/open_door.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_datatypes.h>

// ============================================================================
// Output Helper Functions
// ============================================================================

namespace OutputHelper
{
    void printHeader(const std::string& t)
    {
        ROS_INFO("========== %s ==========", t.c_str());
    }

    void printSuccess(const std::string& m)
    {
        ROS_INFO("[SUCCESS] %s", m.c_str());
    }

    void printError(const std::string& m)
    {
        ROS_ERROR("[ERROR] %s", m.c_str());
    }
}

// ============================================================================
// Constructor & Destructor
// ============================================================================

OpenDoor::OpenDoor()
    : current_state_(WAIT_FOR_HANDLE)
    , handle_detected_(false)
    , hole_cloud_received_(false)
    , joint_state_received_(false)
    , pregrasp_distance_(0.22) // 23 ist gut
    , move_back_distance_(0.35)
    , base_frame_("base_footprint")
    , planning_group_("arm_torso")
    , move_group_(nullptr)
{
}

OpenDoor::~OpenDoor()
{
    removeHandleCollision();
    removeDoorCollision();

    if (move_group_)
    {
        delete move_group_;
    }
}

// ============================================================================
// Initialization
// ============================================================================

bool OpenDoor::initialize(ros::NodeHandle& nh)
{
    nh_ = nh;

    // Clean up collision objects at startup
    removeDoorCollision();
    removeHandleCollision();

    // Subscribers
    handle_centroid_sub_ = nh_.subscribe("handle_synthetic_centroid", 1,
                                          &OpenDoor::handleCentroidCallback, this);
    hole_cloud_sub_ = nh_.subscribe("hole_cloud", 1,
                                     &OpenDoor::holeCloudCallback, this);
    joint_state_sub_ = nh_.subscribe("/joint_states", 1,
                                      &OpenDoor::jointStateCallback, this);

    // Publishers
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);
    gripper_cmd_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 1);

    // MoveIt
    move_group_ = new moveit::planning_interface::MoveGroupInterface(planning_group_);
    move_group_->setPlanningTime(10.0);
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);

    // Action Server
    as_.reset(new OpenDoorServer(nh_, "open_door_action",
                                  boost::bind(&OpenDoor::executeCB, this, _1), false));
    as_->start();

    ROS_INFO("OpenDoor Node Initialized. Waiting for Action Goal.");
    return true;
}

// ============================================================================
// Callbacks
// ============================================================================

void OpenDoor::handleCentroidCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(handle_mutex_);
    latest_handle_centroid_ = *msg;
    handle_detected_ = true;
}

void OpenDoor::holeCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(hole_cloud_mutex_);
    latest_hole_cloud_ = *msg;
    hole_cloud_received_ = true;
}

void OpenDoor::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    latest_joint_state_ = *msg;
    joint_state_received_ = true;
}

bool OpenDoor::getPoseInMap(double& x, double& y, double& yaw)
{
    tf::StampedTransform transform;
    try {
        tf_listener_.waitForTransform("map", base_frame_, ros::Time(0), ros::Duration(1.0));
        tf_listener_.lookupTransform("map", base_frame_, ros::Time(0), transform);
        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
        yaw = tf::getYaw(transform.getRotation());
        return true;
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

// ============================================================================
// Base Movement
// ============================================================================

bool OpenDoor::moveBaseBack(double distance)
{
    OutputHelper::printHeader("BACK: Map Control");

    double start_x, start_y, start_yaw;
    if (!getPoseInMap(start_x, start_y, start_yaw)) {
        ROS_ERROR("Failed to get start pose in map");
        return false;
    }

    ros::Rate rate(50);
    while (ros::ok())
    {
        if (as_->isPreemptRequested()) {
            ROS_WARN("MoveBaseBack preempted");
            cmd_vel_pub_.publish(geometry_msgs::Twist());
            return false;
        }

        double current_x, current_y, current_yaw;
        if (!getPoseInMap(current_x, current_y, current_yaw)) continue;

        double dx = current_x - start_x;
        double dy = current_y - start_y;
        double traveled = std::sqrt(dx * dx + dy * dy);

        if (traveled >= distance) break;

        geometry_msgs::Twist cmd;
        cmd.linear.x = -0.15; // Backwards
        cmd_vel_pub_.publish(cmd);

        ros::spinOnce();
        rate.sleep();
    }
    
    // Stop
    cmd_vel_pub_.publish(geometry_msgs::Twist());
    return true;
}

bool OpenDoor::rotateBase(double rad)
{
    OutputHelper::printHeader("ROTATE: Map Control");

    double start_x, start_y, start_yaw;
    if (!getPoseInMap(start_x, start_y, start_yaw)) {
        ROS_ERROR("Failed to get start pose in map");
        return false;
    }

    ros::Rate rate(50);
    while (ros::ok())
    {
        if (as_->isPreemptRequested()) {
            ROS_WARN("RotateBase preempted");
            cmd_vel_pub_.publish(geometry_msgs::Twist());
            return false;
        }

        double current_x, current_y, current_yaw;
        if (!getPoseInMap(current_x, current_y, current_yaw)) continue;

        // Simple check, assumes no wrapping issue for small rotations or handles it via angles
        double diff = current_yaw - start_yaw;
        // Normalize angle
        while (diff > M_PI) diff -= 2*M_PI;
        while (diff < -M_PI) diff += 2*M_PI;

        if (std::abs(diff) >= std::abs(rad)) break;

        geometry_msgs::Twist cmd;
        cmd.angular.z = (rad > 0) ? 0.3 : -0.3;
        cmd_vel_pub_.publish(cmd);

        ros::spinOnce();
        rate.sleep();
    }

    // Stop
    cmd_vel_pub_.publish(geometry_msgs::Twist());
    return true;
}

// ============================================================================
// Arm & Gripper Control
// ============================================================================

bool OpenDoor::moveArmToPosition(double x, double y, double z, double r, double p, double yaw)
{
    ROS_INFO("[ARM] Moving to Target: [x:%.3f, y:%.3f, z:%.3f | r:%.2f, p:%.2f, y:%.2f]", 
             x, y, z, r, p, yaw);

    geometry_msgs::PoseStamped target;
    target.header.frame_id = base_frame_;
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;
    target.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, yaw);

    move_group_->setPoseTarget(target);
    
    moveit::planning_interface::MoveItErrorCode result = move_group_->move();
    
    if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("[ARM] Move Succeeded.");
        return true;
    }
    else
    {
        ROS_ERROR("[ARM] Move Failed with error code: %d", result.val);
        return false;
    }
}

bool OpenDoor::moveToPregrasp(const geometry_msgs::PointStamped& handle)
{
    ROS_INFO("[PREGRASP] Handle found at: [x:%.3f, y:%.3f, z:%.3f]", 
             handle.point.x, handle.point.y, handle.point.z);

    // Pre-grasp: in front of handle
    double x = handle.point.x - pregrasp_distance_;
    double y = handle.point.y;
    double z = handle.point.z + 0.01; 
    // double x = 0.7; // TIAGo spezifisch
    // double y = 0.0;
    // double z = 1.0;
    
    ROS_INFO("[PREGRASP] Calculating target (Offset %.2fm): [x:%.3f, y:%.3f, z:%.3f]",
             pregrasp_distance_, x, y, z);

    // Fixed orientation for handle grasping: 
    // Roll=0, Pitch=0 (horizontal), Yaw=0 (forward)
    // Adjust as needed for your handle type
    return moveArmToPosition(x, y, z, 0.0, 0.0, 0.0);
}

bool OpenDoor::controlGripper(const std::string& command)
{
    ROS_INFO("[GRIPPER] Executing command: '%s'", command.c_str());

    trajectory_msgs::JointTrajectory traj;
    traj.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};

    trajectory_msgs::JointTrajectoryPoint pt;
    pt.time_from_start = ros::Duration(1.0);

    if (command == "open")
    {
        pt.positions = {0.044, 0.044}; // Max open
    }
    else
    {
        pt.positions = {0.0, 0.0}; // Closed
    }

    traj.points.push_back(pt);
    gripper_cmd_pub_.publish(traj);

    ros::Duration(1.5).sleep();
    return true;
}

bool OpenDoor::moveToStart()
{
    if(start_joint_values_.empty()) return false;
    move_group_->setJointValueTarget(start_joint_values_);
    return (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

// ============================================================================
// Collision Objects
// ============================================================================

void OpenDoor::addDoorCollision(const geometry_msgs::PointStamped& h)
{
    moveit_msgs::CollisionObject obj;
    obj.header.frame_id = base_frame_;
    obj.id = "door";

    shape_msgs::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {0.05, 1.0, 2.0}; // Thin door

    geometry_msgs::Pose p;
    p.position.x = h.point.x + 0.085; // Slightly behind handle
    p.position.y = h.point.y;
    p.position.z = 1.0;
    p.orientation.w = 1.0;

    obj.primitives.push_back(box);
    obj.primitive_poses.push_back(p);
    obj.operation = obj.ADD;

    planning_scene_interface_.applyCollisionObject(obj);
}

void OpenDoor::addHandleCollision(const sensor_msgs::PointCloud2& cloud)
{
    if (cloud.width == 0) return;

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud, pcl_cloud);
    if (pcl_cloud.empty()) return;

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(pcl_cloud, min_pt, max_pt);

    moveit_msgs::CollisionObject obj;
    obj.header.frame_id = cloud.header.frame_id;
    obj.id = "handle_collision";

    shape_msgs::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {
        (max_pt.x - min_pt.x),
        (max_pt.y - min_pt.y) + 0.03,
        (max_pt.z - min_pt.z) + 0.01
    };

    geometry_msgs::Pose p;
    p.position.x = (min_pt.x + max_pt.x) / 2.0;
    p.position.y = (min_pt.y + max_pt.y) / 2.0;
    p.position.z = (min_pt.z + max_pt.z) / 2.0;
    p.orientation.w = 1.0;

    obj.primitives.push_back(box);
    obj.primitive_poses.push_back(p);
    obj.operation = obj.ADD;

    planning_scene_interface_.applyCollisionObject(obj);
}

void OpenDoor::addHandleDetailCollision(const sensor_msgs::PointCloud2& cloud)
{
    // 1. Convert ROS Cloud to PCL
    if (cloud.width == 0) return;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud, pcl_cloud);
    if (pcl_cloud.empty()) return;


    
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(pcl_cloud, min_pt, max_pt);
    
    moveit_msgs::CollisionObject obj;
    obj.header.frame_id = cloud.header.frame_id;
    obj.id = "handle_detail"; 

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX; 
    
    primitive.dimensions = {
        (max_pt.x - min_pt.x) + 0.02,
        0.03,
        0.04
    };

    geometry_msgs::Pose p;
    p.position.x = (min_pt.x + max_pt.x) / 2.0;
    p.position.y = max_pt.y;
    p.position.z = min_pt.z - 0.06;
    p.orientation.w = 1.0;

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(p);
    obj.operation = obj.ADD;

    ROS_INFO("Adding Handle Detail Collision Model");
    planning_scene_interface_.applyCollisionObject(obj);
}

void OpenDoor::removeDoorCollision()
{
    moveit_msgs::CollisionObject obj;
    obj.id = "door";
    obj.operation = obj.REMOVE;
    planning_scene_interface_.applyCollisionObject(obj);
}

void OpenDoor::removeHandleCollision()
{
    // Remove Main Handle Box
    moveit_msgs::CollisionObject obj;
    obj.id = "handle_collision";
    obj.operation = obj.REMOVE;
    planning_scene_interface_.applyCollisionObject(obj);

    // Remove Detail Object
    moveit_msgs::CollisionObject obj_detail;
    obj_detail.id = "handle_detail";
    obj_detail.operation = obj_detail.REMOVE;
    planning_scene_interface_.applyCollisionObject(obj_detail);
}

// ============================================================================
// State Machine
// ============================================================================

bool OpenDoor::stateMachine()
{
    ros::Rate rate(10);
    ROS_INFO("[SM] Starting State Machine");

    while (ros::ok())
    {
        if (as_->isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("[SM] Preempted");
            as_->setPreempted();
            return false;
        }

        switch (current_state_)
        {
            case WAIT_FOR_HANDLE:
            {
                if (handle_detected_)
                {
                    bool has_cloud = false;
                    {
                        std::lock_guard<std::mutex> lock(hole_cloud_mutex_);
                        has_cloud = hole_cloud_received_;
                    }

                    if(has_cloud) {
                        OutputHelper::printHeader("HANDLE DETECTED");
                        
                        // Collision
                        geometry_msgs::PointStamped h_copy;
                        sensor_msgs::PointCloud2 c_copy;
                        {
                            std::lock_guard<std::mutex> lock(handle_mutex_);
                            h_copy = latest_handle_centroid_;
                        }
                        {
                            std::lock_guard<std::mutex> lock(hole_cloud_mutex_);
                            c_copy = latest_hole_cloud_;
                        }

                        addDoorCollision(h_copy);
                        addHandleCollision(c_copy);
                        addHandleDetailCollision(c_copy); // NEU: Detail-Modell hinzufügen

                        current_state_ = OPEN_GRIPPER;
                    }
                }
                break;
            }

            case OPEN_GRIPPER:
                OutputHelper::printHeader("OPEN GRIPPER");
                controlGripper("open");
                current_state_ = MOVE_PREGRASP;
                break;

            case MOVE_PREGRASP:
                OutputHelper::printHeader("MOVE PREGRASP");
                {
                    geometry_msgs::PointStamped h_copy;
                    {
                        std::lock_guard<std::mutex> lock(handle_mutex_);
                        h_copy = latest_handle_centroid_;
                    }
                    
                    static int retries = 0;
                    if(moveToPregrasp(h_copy)) {
                        retries = 0; // Reset for next time (though state machine exits)
                        current_state_ = CLOSE_GRIPPER;
                    } else {
                        retries++;
                        if (retries > 5) {
                             ROS_ERROR("Max retries for pregrasp reached. Aborting.");
                             retries = 0;
                             return false;
                        }
                        ROS_WARN("Retrying pregrasp (%d/5)... Reloading Collision Objects", retries);
                        
                        // Clean
                        removeDoorCollision();
                        removeHandleCollision();
                        ros::Duration(0.5).sleep();

                        // Update Data
                        sensor_msgs::PointCloud2 c_copy;
                        {
                            std::lock_guard<std::mutex> lock(handle_mutex_);
                            h_copy = latest_handle_centroid_; // Update Handle Pos
                        }
                        {
                            std::lock_guard<std::mutex> lock(hole_cloud_mutex_);
                            c_copy = latest_hole_cloud_;
                        }
                        
                        // Re-Add
                        addDoorCollision(h_copy);
                        addHandleCollision(c_copy);
                        addHandleDetailCollision(c_copy);

                        ros::Duration(0.5).sleep();
                    }
                }
                break;

            case CLOSE_GRIPPER:
                OutputHelper::printHeader("CLOSE GRIPPER");
                controlGripper("close");
                current_state_ = MOVE_BACK;
                break;

            case MOVE_BACK:
                OutputHelper::printHeader("MOVE BACK (OPEN DOOR)");
                if(!moveBaseBack(0.35)) return false;
                current_state_ = RELEASE_HANDLE;
                break;

            case RELEASE_HANDLE:
                OutputHelper::printHeader("RELEASE HANDLE");
                controlGripper("open");
                current_state_ = MOVE_AWAY;
                break;

            case MOVE_AWAY:
                OutputHelper::printHeader("MOVE AWAY");
                if(!moveBaseBack(0.1)) return false;
                current_state_ = TASK_COMPLETE;
                break;


            case TASK_COMPLETE:
                OutputHelper::printSuccess("TASK COMPLETE");
                moveToStart();
                removeDoorCollision();
                removeHandleCollision();
                return true;
            
            case IDLE:
                break;
        }

        ros::spinOnce();
        rate.sleep();
    }
    return false;
}

// ============================================================================
// Action Server Callback
// ============================================================================

void OpenDoor::executeCB(const open_door::OpenDoorGoalConstPtr& goal)
{
    ROS_INFO("Action Goal Received");
    
    // Clean up collision objects before new run
    removeDoorCollision();
    removeHandleCollision();
    
    // Save start state for return
    start_joint_values_ = move_group_->getCurrentJointValues();

    current_state_ = WAIT_FOR_HANDLE;
    handle_detected_ = false;
    hole_cloud_received_ = false;

    bool success = stateMachine();

    if(success)
    {
        result_.success = true;
        result_.message = "Success";
        as_->setSucceeded(result_);
    }
    else
    {
        result_.success = false;
        result_.message = "Aborted";
        as_->setAborted(result_); // Only if active
    }
}

// ============================================================================
// Run
// ============================================================================

void OpenDoor::run()
{
    ros::waitForShutdown();
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv)
{
    ros::init(argc, argv, "open_door_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    OpenDoor od;
    if (od.initialize(nh))
    {
        od.run();
    }

    return 0;
}