# RoboCup Receptionist Challenge

Unified documentation and execution guide for the Receptionist task. This project combines Navigation, Manipulation, Audio, and Vision.

## Overview

A service robot that autonomously manages guest arrivals in a home environment. The robot detects door signals, welcomes guests, engages in dialogue to gather information, introduces guests to each other, and guides them to available seats. The system integrates perception, dialogue, navigation, and manipulation capabilities through a modular ROS architecture coordinated by a central state machine.

## Demo

[![Demo Video](demo/demo_preview.png)](demo/demo.mp4)

Watch the full demonstration video in the [demo](demo/) folder (`demo.mp4`).

## Setup & Build

Ensure all dependencies are installed and the workspace is built.

```bash
cd ~/project_ws
pip install -r requirements.txt
catkin build
source devel/setup.bash
```

---

## Architecture Overview

*   **task_control (Brain):** Orchestrates the receptionist flow and stores guest identities (face embeddings).
*   **yolo_perception (Eyes):** Uses YOLOv11m-seg for person detection/clothing segmentation and DeepFace for identity recognition.
*   **messages (Data):** Defines custom ROS message types (`GuestInfo`, `GuestList`) for communication.
*   **Navigation Stack:** Localizes the robot on the map and drives to goals using `move_base` and AMCL.

---

## Operational Sequence

### 1. Map Setup

Select and verify the active map before starting navigation.

```bash
rosservice call /pal_map_manager/change_map "input: 'receptionist_office'"
rosservice call /pal_map_manager/current_map "{}"
```

### 2. Navigation

Start the navigation stack (map + localization + move_base).

```bash
roslaunch task_control receptionist_navigation.launch
```

**⚠️ Critical Step:**
After launching, `move_base` and localization might crash or behave unstably.
**Go to the Web Commander (robot web interface) and restart `move_base` and `localization`.**

**Initialize Localization (AMCL):**
1. Open RViz: `rviz -d $(rospack find task_control)/rviz/navigation\ config.rviz`
2. Click **"2D Pose Estimate"**.
3. Click on the map at the robot's position and drag in the facing direction.

### 3. Perception & Vision

Run the vision core in a new terminal.

```bash
rosrun yolo_perception yolo_node.py
```

### 4. Logic & Control

Start the main state machine.

```bash
rosrun task_control receptionist_sm.py
```

### 5. Additional Modules

Run these components in separate terminals as needed.

*   **Smart Brain (LLM):** `rosrun audio_capture smart_brain.py`
*   **Audio Detector:** `rosrun audio_detection audio.py`
*   **Guest Stylist:** `rosrun guest_stylist stylist_node.py`
*   **Face Tracker:** `rosrun yolo_perception robust_face_tracker.py _base_frame:="base_link"`
*   **Seat Detector:** `rosrun seat_detector smart_seat_detector.py`

### 6. Manipulation

*   **Plane Segmentation:** `roslaunch plane_segmentation plane_segmentation_tiago.launch`
*   **Open Door:** `roslaunch open_door open_door.launch`
*   **Point To:** `roslaunch point_to point_to.launch`

---

## On-the-fly Tuning

Adjust navigation tolerances and costmap inflation without restarting.

**Goal Tolerances:**
```bash
rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS "{xy_goal_tolerance: 0.10, yaw_goal_tolerance: 0.04}"
rosrun dynamic_reconfigure dynparam set /move_base/PalLocalPlanner "{xy_goal_tolerance: 0.10, yaw_goal_tolerance: 0.04}"
```

**Costmap Inflation:**
```bash
rosrun dynamic_reconfigure dynparam set /move_base/local_costmap/inflation_layer inflation_radius 0.30
rosrun dynamic_reconfigure dynparam set /move_base/local_costmap/inflation_layer cost_scaling_factor 10.0
```

---

## Robot Side (TIAGo)

Connect via SSH to the robot to run the hardware drivers for audio capture.

```bash
ssh pal@192.168.1.200
```

**Inside the robot shell:**
```bash
cd audio_ws
source devel/setup.bash
roslaunch audio_capture audio_capture.launch
```

---

## Manual Testing & Triggers

**Manipulation:**
```bash
# Open Door Action
rostopic pub /open_door_action/goal open_door/OpenDoorActionGoal "goal: {}"

# Point to Seat (e.g., ID 3)
rostopic pub /point_to_action/goal point_to/PointToActionGoal "goal: {pose_id: 3}"
```

**Tracking:**
```bash
# Enable/Disable Face Tracking
rostopic pub -1 /tracking_command std_msgs/String "data: 'on'"
rostopic pub -1 /tracking_command std_msgs/String "data: 'off'"
```

**Vision Debug:**
```bash
rostopic echo /vision/guest_info
```