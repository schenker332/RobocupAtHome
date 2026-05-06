#!/usr/bin/env python3
# Keyboard tuner for manual localization (Robot Centric!)
#
# Controls:
#   Arrow Up/Down  : Move robot Forward/Backward
#   Arrow L/R      : Move robot Left/Right (slide)
#   [ and ]        : Rotate map relative to robot
#   s              : PRINT current pose (for AMCL)
#   S (Shift+s)    : SAVE current pose to 'manual_pose.yaml'
#   q              : Quit
#

import sys
import math
import time
import select
import termios
import tty
import yaml
import os

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def get_key(timeout=0.0):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if not rlist:
        return None
    ch1 = sys.stdin.read(1)
    if ch1 != '\x1b':
        return ch1
    if select.select([sys.stdin], [], [], 0.001)[0]:
        ch2 = sys.stdin.read(1)
        if ch2 == '[' and select.select([sys.stdin], [], [], 0.001)[0]:
            ch3 = sys.stdin.read(1)
            return '\x1b[' + ch3
    return ch1

def main():
    rospy.init_node("map_odom_keyboard_tuner")

    map_frame = rospy.get_param("~map_frame", "map")
    odom_frame = rospy.get_param("~odom_frame", "odom")
    base_frame = rospy.get_param("~base_frame", "base_footprint")

    # Initial transform (Map -> Odom)
    # We adjust x, y, yaw of the Odom frame origin expressed in Map frame
    x = float(rospy.get_param("~x0", 0.0))
    y = float(rospy.get_param("~y0", 0.0))
    yaw = float(rospy.get_param("~yaw0_deg", 0.0)) * math.pi / 180.0

    step_xy = 0.05
    step_yaw = 1.0 * math.pi / 180.0

    # TF Setup
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    br = tf2_ros.TransformBroadcaster()
    
    # Wait a sec for TF to fill
    rospy.sleep(1.0)

    print("\n=== ROBOT CENTRIC TUNER ===")
    print("UP/DOWN    : Move Forward/Backward")
    print("LEFT/RIGHT : ROTATE Left/Right")
    print("a / d      : STRAFE Left/Right")
    print("s          : PRINT AMCL coordinates")
    print("S          : SAVE to 'manual_pose.yaml'")
    print("q          : Quit")
    print("===========================\n")

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            # 1. Get current robot yaw in Odom frame (to know what "Forward" is)
            try:
                # We need Odom -> Base
                trans = tf_buffer.lookup_transform(odom_frame, base_frame, rospy.Time(0))
                q = trans.transform.rotation
                (_, _, robot_yaw_in_odom) = euler_from_quaternion([q.x, q.y, q.z, q.w])
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # If TF is missing, assume 0 (robot aligned with odom)
                robot_yaw_in_odom = 0.0

            # 2. Handle Key Input
            k = get_key()
            
            # The robot's total heading in Map = (Map->Odom rotation) + (Odom->Base rotation)
            total_heading = yaw + robot_yaw_in_odom

            if k is not None:
                dx_local = 0.0
                dy_local = 0.0
                d_yaw = 0.0

                if k == '\x1b[A':    # UP (Forward)
                    dx_local = step_xy
                elif k == '\x1b[B':  # DOWN (Backward)
                    dx_local = -step_xy
                elif k == '\x1b[D':  # LEFT (Rotate Left)
                    d_yaw = step_yaw
                elif k == '\x1b[C':  # RIGHT (Rotate Right)
                    d_yaw = -step_yaw
                elif k == 'a':       # Strafe Left
                    dy_local = step_xy
                elif k == 'd':       # Strafe Right
                    dy_local = -step_xy
                elif k == 's' or k == 'S':
                    # Calculate Robot Pose in Map
                    # Pose_Map = T_Map_Odom * T_Odom_Base
                    # Simplified 2D:
                    # RobX_Map = OdomX_Map + RobX_Odom * cos(MapOdomYaw) - RobY_Odom * sin(MapOdomYaw) ...
                    # Let's just use the TF logic we already have implicitly:
                    # We have x,y,yaw (Map->Odom). We need Robot->Map.
                    
                    # Current Map->Odom translation (x,y)
                    # Current Map->Odom rotation (yaw)
                    
                    # Robot in Odom (rx, ry)
                    rx = trans.transform.translation.x
                    ry = trans.transform.translation.y
                    
                    # Robot in Map
                    # X_map = x + rx * cos(yaw) - ry * sin(yaw)
                    # Y_map = y + rx * sin(yaw) + ry * cos(yaw)
                    rob_x_map = x + rx * math.cos(yaw) - ry * math.sin(yaw)
                    rob_y_map = y + rx * math.sin(yaw) + ry * math.cos(yaw)
                    rob_th_map = total_heading

                    msg = f"AMCL POSE: x={rob_x_map:.3f}, y={rob_y_map:.3f}, a={rob_th_map:.3f}"
                    print(f"\n{msg}")
                    
                    if k == 'S':
                        path = os.path.join(os.getcwd(), "manual_pose.yaml")
                        with open(path, "w") as f:
                            f.write(f"initial_pose_x: {rob_x_map}\n")
                            f.write(f"initial_pose_y: {rob_y_map}\n")
                            f.write(f"initial_pose_a: {rob_th_map}\n")
                        print(f"SAVED to {path}")

                elif k == 'q':
                    break

                # Apply changes to Map->Odom transform
                # We want to move the robot by (dx_local, dy_local) in GLOBAL map space
                # But aligned with its heading.
                
                # Global delta vector
                dx_global = dx_local * math.cos(total_heading) - dy_local * math.sin(total_heading)
                dy_global = dx_local * math.sin(total_heading) + dy_local * math.cos(total_heading)
                
                # Apply to Map->Odom origin
                x += dx_global
                y += dy_global
                yaw += d_yaw

            # 3. Publish Map -> Odom TF
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = map_frame
            t.child_frame_id = odom_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            br.sendTransform(t)

            r.sleep()

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == "__main__":
    main()
