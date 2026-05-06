#!/usr/bin/env python3
"""
State: Open Door
================
Sends door opening command to open_door action server and waits for result.
Also speaks while doing it.
"""

import rospy
import smach
import actionlib
from pal_interaction_msgs.msg import TtsActionGoal
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Header

class StateOpenDoor(smach.State):
    """
    Sends goal to /open_door_action and waits for success result.
    Equivalent to: rostopic pub /open_door_action/goal open_door/OpenDoorActionGoal "goal: {}"
    """
    
    def __init__(self, timeout=60.0, pre_delay=3.0):
        smach.State.__init__(self, outcomes=['done', 'timeout'])
        self.timeout = timeout  # Max wait time for action to complete
        self.pre_delay = pre_delay  # Delay before sending command
        self.action_client = None
    
    def execute(self, userdata):
        rospy.loginfo("🚪 STATE: Opening door...")
        
        try:
            # Import action types from open_door package
            from open_door.msg import OpenDoorAction, OpenDoorGoal, OpenDoorResult
            
            # Pre-delay (Puffer bevor wir den Befehl senden)
            if self.pre_delay > 0:
                rospy.loginfo(f"   ⏳ Warte {self.pre_delay}s vor dem Greifen...")
                rospy.sleep(self.pre_delay)
            
            # Create action client if not exists
            if self.action_client is None:
                rospy.loginfo("   🔌 Connecting to /open_door_action server...")
                self.action_client = actionlib.SimpleActionClient('/open_door_action', OpenDoorAction)
            
            # Wait for action server
            server_found = self.action_client.wait_for_server(timeout=rospy.Duration(10.0))
            if not server_found:
                rospy.logwarn("   ⚠️ Action server not found! Continuing anyway...")
                return 'timeout'
            
            rospy.loginfo("   ✅ Connected to action server!")
            
            # Send goal
            goal = OpenDoorGoal()
            rospy.loginfo("   📤 Sending open door goal...")
            self.action_client.send_goal(goal, feedback_cb=self._feedback_cb)
            
            # Wait for result with timeout
            rospy.loginfo(f"   ⏳ Waiting for result (max {self.timeout}s)...")
            finished = self.action_client.wait_for_result(timeout=rospy.Duration(self.timeout))
            
            if finished:
                result = self.action_client.get_result()
                if result and result.success:
                    rospy.loginfo(f"   ✅ Door opened successfully! Message: {result.message}")
                    return 'done'
                else:
                    msg = result.message if result else "No result"
                    rospy.logwarn(f"   ⚠️ Door action finished but not successful: {msg}")
                    return 'done'  # Trotzdem weitermachen
            else:
                rospy.logwarn(f"   ⏰ Timeout after {self.timeout}s!")
                self.action_client.cancel_goal()
                return 'timeout'
            
        except ImportError as e:
            rospy.logerr(f"   ❌ Could not import open_door action: {e}")
            rospy.logerr("   ❌ Did you run: catkin build open_door && source devel/setup.bash ?")
            return 'timeout'
        except Exception as e:
            rospy.logerr(f"   ❌ Error during door opening: {e}")
            return 'timeout'
    
    def _feedback_cb(self, feedback):
        """Callback for action feedback."""
        rospy.loginfo(f"   📡 Feedback: {feedback.current_state}")
