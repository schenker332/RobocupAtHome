#!/usr/bin/env python3
"""
FaceLearning Action
===================
Sammelt Face Samples von /vision/guest_info (NUR sammeln, NICHT registrieren!).

AUFGABE:
- Wartet auf Person vor der Kamera
- Sammelt N Face Samples
- Speichert Samples und Track-ID
- Gibt alles an den aufrufenden State zurück

"""

import rospy
from messages.msg import GuestList


class FaceLearning:
    """
    Sammelt NUR Face Samples - registriert NICHT im GuestManager!
    Die Registrierung passiert im State.
    """
    
    def __init__(self, samples_required=10, timeout=30.0):
        """
        Args:
            samples_required: Anzahl benötigter Face Samples (default: 10)
            timeout: Max Zeit in Sekunden (default: 30s)
        """
        self.samples_required = samples_required
        self.timeout_duration = rospy.Duration(timeout)
        
        # Status
        self.is_running = False
        self.done = False
        self.success = False
        
        # Gesammelte Daten
        self.collected_samples = []  # Face encodings
        self.track_id = -1           # Track ID der Person
        
        # Tracking
        self.sample_count = 0
        self.start_time = None
        
        # Subscriber (wird in start() aktiviert)
        self.subscriber = None
        
        #rospy.loginfo(f"[FaceLearning] Initialisiert ({samples_required} Samples)")

    def start(self):
        """Startet den Face Learning Prozess."""
        if self.is_running:
            rospy.logwarn("[FaceLearning] Bereits gestartet!")
            return True  # Läuft bereits, ist kein Fehler
        
        self.is_running = True
        self.done = False
        self.success = False
        self.sample_count = 0
        self.track_id = -1
        self.collected_samples = []
        self.start_time = rospy.Time.now()
        
        # Subscriber starten
        self.subscriber = rospy.Subscriber('/vision/guest_info', GuestList, self._callback)
        
        #rospy.loginfo(f"[FaceLearning] 🎥 Gestartet! Warte auf Person...")
        return True  # Erfolgreich gestartet

    def stop(self):
        """Stoppt den Learning Prozess."""
        if self.subscriber:
            self.subscriber.unregister()
            self.subscriber = None
        self.is_running = False
        rospy.loginfo(f"[FaceLearning] Gestoppt.")

    def is_done(self):
        """Gibt zurück ob der Prozess abgeschlossen ist."""
        if self.done:
            rospy.loginfo_once("[FaceLearning] is_done() → True")
        return self.done

    def get_result(self):
        """Gibt Erfolgs-Status zurück. Nur nach is_done() aufrufen!"""
        return self.success

    def get_progress(self):
        """Gibt aktuellen Fortschritt zurück als Dictionary."""
        return {
            'collected': self.sample_count,
            'target': self.samples_required,
            'percentage': float(self.sample_count) / self.samples_required
        }
    
    def get_samples(self):
        """Gibt gesammelte Face Samples zurück. Nur nach is_done() aufrufen!"""
        return self.collected_samples
    
    def get_track_id(self):
        """Gibt Track ID der gelernten Person zurück. Nur nach is_done() aufrufen!"""
        return self.track_id

    def _callback(self, msg):
        """Interner Callback für /vision/guest_info."""
        if not self.is_running or self.done:
            return
        
        # 1. TIMEOUT CHECK
        if rospy.Time.now() - self.start_time > self.timeout_duration:
            rospy.logerr(f"[FaceLearning] ⏰ Timeout! Keine Person gefunden nach {self.timeout_duration.to_sec()}s")
            self._finish(success=False)
            return

        # 2. VERARBEITE ALLE PERSONEN
        for person in msg.guests:
            track_id = person.current_track_id
            
            # Face Encoding Check
            if len(person.face_encoding) == 0:
                if track_id == self.track_id:
                    rospy.logwarn_throttle(1.5, f"[FaceLearning] [ID {track_id}] Bitte schaue mich an!")
                continue
            
            # STRENGER FILTER NUR BEIM LERNEN:
            # Wir prüfen, ob das Gesicht mindestens 85 Pixel breit ist
            if person.face_width < 85:
                # Logge auch für die Person, die wir gerade lernen
                if track_id == self.track_id or self.track_id == -1:
                    rospy.logwarn_throttle(2.0, f"[FaceLearning] ⚠️ IGNORIERT: {person.face_width}x{person.face_height}px (zu klein, brauche >85px)")
                continue

            # FALL 1: Wir lernen bereits diese Person
            if track_id == self.track_id:
                self._add_sample(person)
                continue
            
            # FALL 2: Neue Person gefunden (wir lernen noch niemanden)
            if self.track_id == -1:
                # Neue unbekannte Person → Starte Learning
                rospy.loginfo(f"[FaceLearning] 🆕 Neue Person erkannt [ID {track_id}] ({person.face_width}x{person.face_height}px) → Starte Sammlung")
                self.track_id = track_id
                
                # Füge erstes Sample hinzu
                self._add_sample(person)

    def _add_sample(self, person):
        """Fügt ein Face Sample hinzu."""
        # Speichere Sample lokal
        self.collected_samples.append(list(person.face_encoding))
        self.sample_count += 1
        
        progress = self.get_progress()
        rospy.loginfo(f"[FaceLearning] sample {progress['collected']}/{progress['target']} ({person.face_width}x{person.face_height}px)")
        
        # Fertig?
        if self.sample_count >= self.samples_required:
            rospy.loginfo(f"[FaceLearning] ✅ Alle {self.samples_required} Samples gesammelt!")
            self._finish(success=True)

    def _finish(self, success):
        """Beendet den Learning Prozess."""
        self.stop()  # Erst stoppen
        self.success = success
        self.done = True  # ZULETZT setzen!
        
        if success:
            rospy.loginfo(f"[FaceLearning] 🎉 Erfolgreich! {len(self.collected_samples)} Samples gesammelt (Track ID: {self.track_id})")
        else:
            rospy.logerr(f"[FaceLearning] ❌ Fehlgeschlagen!")
        
        rospy.loginfo(f"[FaceLearning] done={self.done}, success={self.success}")


# === TEST FUNKTION ===
if __name__ == '__main__':
    """Test: Face Learning standalone"""
    rospy.init_node('test_face_learning')
    
    learner = FaceLearning(samples_required=10, timeout=30.0)
    
    rospy.loginfo("=== TEST: Face Learning ===")
    learner.start()
    
    # Warte bis fertig
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown() and not learner.is_done():
        rate.sleep()
    
    if learner.get_result():
        rospy.loginfo("✅ TEST ERFOLGREICH!")
        rospy.loginfo(f"   Samples: {len(learner.get_samples())}")
        rospy.loginfo(f"   Track ID: {learner.get_track_id()}")
    else:
        rospy.logerr("❌ TEST FEHLGESCHLAGEN!")
