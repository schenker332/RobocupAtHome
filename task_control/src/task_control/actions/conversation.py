#!/usr/bin/env python3
"""
Conversation Action
===================
Steuert Smart Brain für Gespräche und sammelt Ergebnisse.

AUFGABE:
- Aktiviert Smart Brain über /set_state (Int32)
- Hört auf /guest_data für Name/Drink
- Hört auf /conversation_done für Abschluss
- Updated GuestManager SOFORT wenn Daten ankommen

VERWENDUNG:
    conversation = Conversation(
        role="host",
        state_number=1,  # Smart Brain State (1=Host, 2=Guest, etc.)
        guest_manager=gm
    )
    conversation.start()
    
    while not conversation.is_done():
        time.sleep(0.1)
    
    conversation.stop()
"""

import rospy
import time
from std_msgs.msg import String, Int32


class Conversation:
    """
    Steuert Smart Brain und sammelt Gesprächs-Ergebnisse.
    Updated GuestManager direkt wenn Daten von Smart Brain kommen.
    """
    
    def __init__(self, role, state_number, guest_manager):
        """
        Args:
            role: Gast-Rolle ("host", "guest1", "guest2")
            state_number: State-Nummer für Smart Brain (z.B. 1, 2, 3)
            guest_manager: GuestManager Instanz (für direkte Updates)
        """
        self.role = role
        self.state_number = state_number
        self.guest_manager = guest_manager
        
        # Status
        self.is_running = False
        self.done = False
        
        # Gesammelte Daten (optional, für Debugging)
        self.name = None
        self.drink = None
        
        # Publishers (WICHTIG: /set_state nimmt Int32, nicht String!)
        self.state_pub = rospy.Publisher('/set_state', Int32, queue_size=1, latch=True)
        
        # Subscribers (werden in start() aktiviert)
        self.data_sub = None
        self.done_sub = None
        
        #rospy.loginfo(f"[Conversation] Initialisiert für {role} (State: {state_number})")

    def start(self):
        """Startet Conversation - aktiviert Smart Brain."""
        if self.is_running:
            rospy.logwarn(f"[Conversation] Bereits gestartet!")
            return
        
        self.is_running = True
        self.done = False
        self.name = None
        self.drink = None
        
        # Subscriber für Ergebnisse ZUERST starten (bevor wir Smart Brain aktivieren)
        self.data_sub = rospy.Subscriber('/guest_data', String, self._on_guest_data)
        self.done_sub = rospy.Subscriber('/conversation_done', Int32, self._on_conversation_done)
        
        # Warte bis Publisher Subscriber gefunden hat
        #rospy.loginfo(f"[Conversation] 💬 Starte für {self.role}...")
        time.sleep(0.3)  # Kurz warten
        
        # Warte auf Subscriber
        timeout = 5.0
        start = time.time()
        while self.state_pub.get_num_connections() == 0:
            if time.time() - start > timeout:
                rospy.logwarn("[Conversation] Kein Subscriber für /set_state gefunden! Smart Brain läuft?")
                break
            time.sleep(0.1)
        
        # Aktiviere Smart Brain mit diesem State (als Int32!)
        self.state_pub.publish(Int32(self.state_number))
        #rospy.loginfo(f"[Conversation] Published /set_state: {self.state_number}")
        
        #rospy.loginfo(f"[Conversation] 🎤 Smart Brain aktiviert! Warte auf Daten...")

    def stop(self):
        """Stoppt Conversation - deaktiviert Smart Brain."""
        if not self.is_running:
            return
        
        rospy.loginfo(f"[Conversation] Stoppe...")
        
        # Deaktiviere Smart Brain (zurück zu idle = State 0)
        self.state_pub.publish(Int32(0))
        #rospy.loginfo(f"[Conversation] Published /set_state: 0 (idle)")
        
        # Unsubscribe
        if self.data_sub:
            self.data_sub.unregister()
            self.data_sub = None
        if self.done_sub:
            self.done_sub.unregister()
            self.done_sub = None
        
        self.is_running = False
        rospy.loginfo(f"[Conversation] Gestoppt.")

    def is_done(self):
        """Gibt zurück ob Conversation abgeschlossen ist."""
        return self.done

    def get_name(self):
        """Gibt gesammelten Namen zurück (oder None)."""
        return self.name
    
    def get_drink(self):
        """Gibt gesammeltes Getränk zurück (oder None)."""
        return self.drink

    def _on_guest_data(self, msg):
        """
        Callback für /guest_data Topic.
        Format: "role:field:value" (z.B. "host:name:Charlie" oder "host:drink:Cola")
        """
        if not self.is_running:
            return
        
        try:
            # Parse Message
            parts = msg.data.split(':')
            if len(parts) != 3:
                rospy.logwarn(f"[Conversation] Ungültiges Format: {msg.data}")
                return
            
            role = parts[0]
            field = parts[1]
            value = parts[2]
            
            # Nur für unsere Rolle
            if role != self.role:
                return
            
            rospy.loginfo(f"[Conversation] 📨 Empfangen: {field}={value}")
            
            # Speichere lokal (für get_name/get_drink)
            if field == "name":
                self.name = value
            elif field == "drink":
                self.drink = value
            
            # WICHTIG: Update GuestManager SOFORT!
            update_dict = {field: value}
            self.guest_manager.update_guest(self.role, **update_dict)
            
            rospy.loginfo(f"[Conversation] ✅ GuestManager updated: {self.role}.{field} = {value}")
            
        except Exception as e:
            rospy.logerr(f"[Conversation] Fehler in _on_guest_data: {e}")

    def _on_conversation_done(self, msg):
        """
        Callback für /conversation_done Topic.
        Smart Brain signalisiert dass alle Tasks abgeschlossen sind.
        """
        if not self.is_running or self.done:
            return
        
        rospy.loginfo(f"[Conversation] 🎉 Conversation abgeschlossen!")
        self.done = True

