#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class NavigationController:
    def __init__(self):
        # Initialisierung des ActionClients
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # Wir warten nicht im __init__, um den Start nicht zu blockieren, 
        # sondern beim ersten Aufruf oder separat.
        # Aber für einfache Nutzung ist ein kurzes Warten hier oft okay, 
        # solange move_base läuft.
        self.client.wait_for_server()


    def go_to(self, target_pose):
        """
        Fährt zu einer Koordinate.
        target_pose: Ein Dict oder Tuple mit {'x': ..., 'y': ..., 'z': ..., 'w': ...}
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Koordinaten setzen (unterstützt Dict Zugriff)
        goal.target_pose.pose.position.x = target_pose['x']
        goal.target_pose.pose.position.y = target_pose['y']
        goal.target_pose.pose.orientation.z = target_pose['z']
        goal.target_pose.pose.orientation.w = target_pose['w']

        rospy.loginfo(f"🚗 Fahre zu Ziel: x={target_pose['x']:.2f}, y={target_pose['y']:.2f} ...")
        
        self.client.send_goal(goal)
        self.client.wait_for_result()
        
        state = self.client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("✅ Ziel erreicht!")
            return True
        else:
            rospy.loginfo(f"❌ Ziel nicht erreicht. Status: {state}")
            return False
