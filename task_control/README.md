# Robocup Project - Task Control

This package contains the launch files, configurations, and scripts for controlling the robot's tasks.

## How to Start Navigation

This guide explains how to start the TIAGo robot's navigation stack and send goals.

### Receptionist Runbook

Use this checklist for the receptionist demo flow. It assumes the robot is already on the correct ROS master.

Select the receptionist map and verify the active map:

```bash
rosservice call /pal_map_manager/change_map "input: 'receptionist_office'"
rosservice call /pal_map_manager/current_map "{}"
```

Follow the operational sequence:

1. Start the navigation launch file.
2. Restart the move_base and localization web manager service.
3. Open RViz with the navigation configuration.
4. Align the robot pose in RViz.
5. Start the perception node.

Commands for the sequence:

```bash
roslaunch task_control receptionist_nav_FINAL.launch
rviz -d $(rospack find task_control)/rviz/navigation\ config.rviz
rosrun yolo_perception yolo_node.py
```

Apply on-the-fly tolerances and costmap tuning:

```bash
rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS "{xy_goal_tolerance: 0.10, yaw_goal_tolerance: 0.04}"
rosrun dynamic_reconfigure dynparam set /move_base/PalLocalPlanner "{xy_goal_tolerance: 0.10, yaw_goal_tolerance: 0.04}"

rosrun dynamic_reconfigure dynparam set /move_base/local_costmap/inflation_layer inflation_radius 0.30
rosrun dynamic_reconfigure dynparam set /move_base/local_costmap/inflation_layer cost_scaling_factor 10.0

rosrun dynamic_reconfigure dynparam set /move_base/global_costmap/inflation_layer inflation_radius 0.40
rosrun dynamic_reconfigure dynparam set /move_base/global_costmap/inflation_layer cost_scaling_factor 15.0
```

### Prerequisites

- The robot's main launch file is running on the robot (`tiago_public.launch`).
- Your terminal is configured to connect to the robot's ROS master. Open a new terminal and run:

  ```bash
  source ~/project_ws/devel/setup.bash
  export ROS_MASTER_URI=http://tiago-4c:11311
  export ROS_IP=<your_computer_ip>
  ```

### 1. Launch the Navigation

Run the following command to start the map server, AMCL for localization, and the `move_base` navigation stack:

```bash
roslaunch task_control receptionist_nav_FINAL.launch
```

### 2. Set the Initial Pose in RViz

Once everything is running, you need to tell the robot where it is on the map.

1.  Open RViz with the navigation configuration:
    ```bash
    rviz -d $(rospack find task_control)/rviz/navigation\ config.rviz
    ```
2.  In RViz, find the **"2D Pose Estimate"** button in the top toolbar.
3.  Click it, then click on the map (`my_private_map`) at the robot's current location and drag in the direction the robot is facing. This will initialize AMCL, and you should see the laser scan align with the map walls.

### 3. Send a Test Navigation Goal

To confirm the navigation is working, send a simple goal.

1.  In RViz, click the **"2D Nav Goal"** button in the top toolbar.
2.  Click on the map slightly in front of the robot and drag to set the desired orientation.
3.  The robot should turn and move to the goal.

### 4. Publish a Goal from Code (Python)

You can send goals programmatically. The recommended way is to use the `actionlib` client for `move_base`, as it provides feedback. Here is a Python example.

```python
#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion

def send_goal(x, y, yaw):
    """
    Sends a navigation goal to the move_base action server.
    """
    # Create a client for the move_base action server
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Action server found.")

    # Create a goal message
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    
    # Convert yaw to quaternion
    from tf.transformations import quaternion_from_euler
    q = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation = Quaternion(*q)

    # Send the goal
    rospy.loginfo("Sending goal to ({}, {}) with yaw {}".format(x, y, yaw))
    client.send_goal(goal)

    # Wait for the result
    client.wait_for_result()
    rospy.loginfo("Goal execution finished.")
    return client.get_state()

if __name__ == '__main__':
    try:
        rospy.init_node('goal_publisher_node', anonymous=True)
        # Example: send a goal to x=1.0, y=1.0, with yaw=0 (radians)
        result = send_goal(1.0, 1.0, 0.0)
        rospy.loginfo("Result: " + str(result))
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

```
**Note:** For very simple, one-off goals where you don't need feedback, you can also publish a single `geometry_msgs/PoseStamped` message directly to the `/move_base_simple/goal` topic.
