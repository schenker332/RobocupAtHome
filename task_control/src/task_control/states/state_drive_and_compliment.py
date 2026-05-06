#!/usr/bin/env python3
"""
State: Drive and Compliment
============================
Fährt zum Ziel UND macht gleichzeitig ein Kompliment.
"""

import rospy
import smach
import threading
from std_msgs.msg import String
from task_control.actions.conversation import Conversation
from task_control.actions.navigation_controller import NavigationController
from task_control.config import WAYPOINTS

class StateDriveAndCompliment(smach.State):
    def __init__(self, guest_manager, target_location, target_role="guest1"):
        smach.State.__init__(self, outcomes=['done', 'failed'])
        self.guest_manager = guest_manager
        self.target_location = target_location
        self.target_role = target_role
        self.target_state = 25  # Fixer State für Compliment
        self.nav = NavigationController()
        #rospy.loginfo(f"[StateDriveAndCompliment] Initialisiert für {self.target_role} -> {target_location}")

    def execute(self, userdata):
        rospy.loginfo(f"🚗✨ STATE: DRIVE & COMPLIMENT - Fahre zu '{self.target_location}' und mache Kompliment")
        
        # 1. Hole Wegpunkt
        target = WAYPOINTS.get(self.target_location)
        if not target:
            rospy.logerr(f"❌ Unbekannter Ort: {self.target_location}")
            return 'failed'
        
        # 2. Visuelle Beschreibung holen
        desc = "No visual info available."
        guest = self.guest_manager.get_guest_by_role(self.target_role)
        
        if guest and guest.get("visual_description"):
            raw_desc = guest["visual_description"]
            if raw_desc:
                desc = raw_desc
                rospy.loginfo(f"   Nutze Beschreibung: {desc}")
        
        # 3. Kontext an Smart Brain senden
        ctx_pub = rospy.Publisher('/smart_brain/scene_context', String, queue_size=1, latch=True)
        ctx_pub.publish(String(desc))
        rospy.sleep(0.5)  # Kurz warten damit Nachricht ankommt
        
        # 4. PARALLEL: Navigation starten (in eigenem Thread)
        nav_done = threading.Event()
        nav_success = [False]  # Liste damit wir im Thread schreiben können
        
        def navigate():
            rospy.loginfo("   🚗 [Thread] Navigation gestartet...")
            success = self.nav.go_to(target)
            nav_success[0] = success
            nav_done.set()
            rospy.loginfo(f"   🚗 [Thread] Navigation beendet: {'✅' if success else '❌'}")
        
        nav_thread = threading.Thread(target=navigate)
        nav_thread.daemon = True
        nav_thread.start()
        
        # 5. Kurz warten bis Navigation läuft
        rospy.sleep(1.0)
        
        # 6. PARALLEL: Conversation starten (während Navigation läuft!)
        conversation = Conversation(
            role=self.target_role,
            state_number=self.target_state,
            guest_manager=self.guest_manager
        )
        
        rospy.loginfo("   ✨ Starte Kompliment (während der Fahrt)...")
        conversation.start()
        
        # 7. Warte auf beide Aktionen
        # Warte bis Conversation fertig ist
        while not conversation.is_done() and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        conversation.stop()
        rospy.loginfo("   ✅ Kompliment erledigt.")
        
        # Warte auch auf Navigation (falls noch nicht fertig)
        if not nav_done.is_set():
            rospy.loginfo("   ⏳ Warte noch auf Navigation...")
            nav_done.wait()
        
        # 8. Prüfe Ergebnis
        if nav_success[0]:
            rospy.loginfo("✅ Drive & Compliment erfolgreich abgeschlossen!")
            return 'done'
        else:
            rospy.logerr("❌ Navigation fehlgeschlagen!")
            return 'failed'
