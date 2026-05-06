#!/home/niclas/robocup/PR/venv39/bin/python3
import os
import sys
import rospy
import numpy as np
import threading
import re
import time  # NEU: für Zeitmessung
import openai
from messages.msg import AudioData
from pal_interaction_msgs.msg import TtsActionGoal, TtsActionResult
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Header, String, Int32
from point_to.msg import PointToActionGoal

# --- KONFIGURATION ---
API_KEY = ""    

# Pfad zu den Prompt-Dateien
PROMPTS_DIR = os.path.join(os.path.dirname(__file__), 'prompts')

# State-Konfiguration: Mapping von State-Nummer -> Dateiname
STATE_FILES = {
    1: "STATE1_HOST_GREETING.txt",
    2: "STATE2_GUEST_WELCOME.txt",
    8: "STATE8_INTRODUCE_GUEST.txt",
    9: "STATE9_VERIFY_SEATS.txt",
    12: "STATE12_GUEST_WELCOME_2.txt",
    13: "STATE13_VERIFY_CHANGES.txt",
    14: "STATE14_INTRODUCE_GUEST_2.txt",
    15: "STATE15_FINAL_VERIFY.txt",
    25: "STATE25_GUEST_COMPLIMENT.txt"
}

# Fallback Prompt wenn Datei nicht gefunden
DEFAULT_PROMPT = """You are Tiago, a friendly robot. Have a natural conversation."""
# ---------------------

# --- EINSTELLUNGEN ---
# Whisper Speech-to-Text
MODEL_SIZE = "small"           # Whisper Modell: tiny, base, small, medium, large
DEVICE = "cpu"                  # cpu oder cuda (GPU)

# Audio Detection Thresholds (0.0 - 1.0)
START_THRESHOLD = 0.20          # Lautstärke um Recording zu starten
CONTINUE_THRESHOLD = 0.3        # Lautstärke während Recording (höher = weniger false positives)
SILENCE_TIMEOUT = 1             # Sekunden Stille bevor Recording stoppt

# Recording Validation
REQUIRED_VALIDATION = 7         # Anzahl Chunks über START_THRESHOLD für valide Aufnahme
MAX_VALIDATION_FAILS = 1        # Tolerierte Chunks unter Schwelle während Validation

# OpenAI LLM
OPENAI_MODEL = "gpt-4o"    
MAX_TOKENS = 60                 # Max Tokens in Antwort (~45 Wörter bei 60)   

try:
    from faster_whisper import WhisperModel
    WHISPER_AVAILABLE = True
except ImportError:
    WHISPER_AVAILABLE = False

class SmartBrainNode:
    def __init__(self):
        rospy.init_node('smart_brain_node', anonymous=True)
        
        self.is_recording = False
        self.is_processing = False 
        self.is_speaking = False
        self.last_speech_time = time.time()
        self.audio_buffer = []
        
        # === State Management ===
        self.current_state = 0  # Start im IDLE Mode - wartet auf /set_state!
        self.current_config = {} # Speichert geparste Config (Tasks, Role, etc.)
        
        # === Gespeicherte Daten für spätere States ===
        self.stored_data = {
            "host_name": None,
            "guest_name": None,
            "guest_drink": None
        }
        self.scene_context = ""  # New: Stores the scene description (e.g. seat info)
        
        # === Task Tracking (wird pro State initialisiert) ===
        self.tasks = {}
        self.conversation_active = False  # Startet inaktiv!
        self.base_system_prompt = ""
        self.messages = []  # Initialisiere leer - wird in _load_state() gefüllt
        
        # NICHT automatisch State laden - warte auf /set_state!
        # self._load_state(self.current_state)
        
        # Counter for validating recording start
        self.validation_count = 0
        self.required_validation = REQUIRED_VALIDATION
        self.is_validating = False
        self.validation_fails = 0
        self.max_validation_fails = MAX_VALIDATION_FAILS

        # OpenAI
        if len(API_KEY) < 20:
            self.openai_client = None
            rospy.logerr("❌ KEIN API KEY!")
        else:
            self.openai_client = openai.OpenAI(api_key=API_KEY)

        # Whisper
        self.model = None
        if WHISPER_AVAILABLE:
            rospy.loginfo(f"Lade Whisper '{MODEL_SIZE}'...")
            self.model = WhisperModel(MODEL_SIZE, device=DEVICE, compute_type="int8")
            rospy.loginfo("✅ Whisper bereit!")
        else:
            rospy.logerr("❌ Faster-Whisper nicht installiert!")

        # Publishers
        self.tts_pub = rospy.Publisher('/tts/goal', TtsActionGoal, queue_size=10)
        self.guest_data_pub = rospy.Publisher('/guest_data', String, queue_size=10)
        self.conversation_done_pub = rospy.Publisher('/conversation_done', Int32, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/audio', AudioData, self.audio_callback)
        rospy.Subscriber('/tts/result', TtsActionResult, self.tts_result_callback)
        rospy.Subscriber('/set_state', Int32, self.set_state_callback)
        rospy.Subscriber('/smart_brain/scene_context', String, self.scene_context_callback) # NEW
        
        # TTS done flag
        self.tts_done = False
        
        # PointTo Publisher für synchronisiertes Zeigen (einfache Int32 Message mit Stuhlnummer)
        self.point_to_pub = rospy.Publisher('/smart_brain/point_to_seat', Int32, queue_size=1)
        # Echter PointTo Action Publisher für Arm-Bewegung
        self.point_to_action_pub = rospy.Publisher('/point_to_action/goal', PointToActionGoal, queue_size=1)
        
        # Wait for publisher to connect (Wall-clock time!)
        time.sleep(0.5)
        
        self._print_status()

    def _parse_prompt_file(self, content):
        """
        Parst Header (--- CONFIG ---) und Body (--- PROMPT ---) aus der Textdatei.
        Gibt (config_dict, prompt_text) zurück.
        """
        config = {
            "role": None,
            "tasks": [],
            "name_task": None,
            "drink_task": None,
            "start_on_load": False # NEW: Auto-start conversation
        }
        prompt_text = content

        # Checke auf Header
        if "--- CONFIG ---" in content and "--- PROMPT ---" in content:
            parts = content.split("--- PROMPT ---")
            header = parts[0].replace("--- CONFIG ---", "").strip()
            prompt_text = parts[1].strip()
            
            # Header parsen
            for line in header.splitlines():
                if ":" in line:
                    key, val = line.split(":", 1)
                    key = key.strip().upper()
                    val = val.strip()
                    
                    if key == "ROLE":
                        config["role"] = val if val != "NONE" else None
                    elif key == "TASKS":
                        config["tasks"] = [t.strip() for t in val.split(",") if t.strip()]
                    elif key == "NAME_TASK":
                        config["name_task"] = val if val != "NONE" else None
                    elif key == "DRINK_TASK":
                        config["drink_task"] = val if val != "NONE" else None
                    elif key == "START_ON_LOAD": # NEW
                        config["start_on_load"] = (val.upper() == "YES" or val.upper() == "TRUE")
        
        return config, prompt_text

    def _load_state(self, state_num):
        """Lädt Prompt und Tasks für einen State."""
        print(f"\n{'='*60}")
        print(f"🔄 LOADING STATE {state_num}")
        print(f"{'='*60}")
        
        # State 0 = IDLE (kein Prompt, keine Tasks, nicht lauschen)
        if state_num == 0:
            self.current_state = 0
            self.conversation_active = False
            self.tasks = {}
            self.base_system_prompt = ""
            print("💤 IDLE Mode - Warte auf /set_state Command...")
            print(f"{'='*60}\n")
            return
        
        if state_num not in STATE_FILES:
            rospy.logerr(f"Unknown state: {state_num}")
            return
        
        filename = STATE_FILES[state_num]
        self.current_state = state_num
        
        # Reset conversation
        self.conversation_active = True
        
        # Reset stored_data für neuen State
        if state_num == 1:
            # State 1 (Host) - Alles zurücksetzen
            self.stored_data["host_name"] = None
            self.stored_data["guest_name"] = None
            self.stored_data["guest_drink"] = None
        elif state_num == 2:
            # State 2 (Guest 1) - Nur Guest-Daten zurücksetzen
            self.stored_data["guest_name"] = None
            self.stored_data["guest_drink"] = None
        elif state_num == 12:
             # Save Guest 1's data before clearing for Guest 2
             if self.stored_data.get("guest_name"):
                 self.stored_data["first_guest_name"] = self.stored_data["guest_name"]
             if self.stored_data.get("guest_drink"):
                 self.stored_data["first_guest_drink"] = self.stored_data["guest_drink"]
             
             # State 12 (Guest 2) - Wir überschreiben die Guest Daten für den aktuellen Kontext
             self.stored_data["guest_name"] = None
             self.stored_data["guest_drink"] = None
        
        # Lade Prompt aus Datei
        prompt_file = os.path.join(PROMPTS_DIR, filename)
        try:
            with open(prompt_file, 'r') as f:
                content = f.read()
            
            # Parse Config & Prompt
            self.current_config, self.base_system_prompt = self._parse_prompt_file(content)
            print(f"✅ Loaded prompt: {filename}")
            print(f"   Config: {self.current_config}")
            
        except FileNotFoundError:
            rospy.logerr(f"Prompt file not found: {prompt_file}")
            self.base_system_prompt = DEFAULT_PROMPT
            self.current_config = {"tasks": []}
        
        # Initialisiere Tasks
        self.tasks = {}
        for task_name in self.current_config["tasks"]:
            self.tasks[task_name] = {"done": False, "value": None}
        
        # Ersetze Platzhalter in Prompt (für State 8, 12, 14)
        if state_num == 8:
            # Fallback falls kein Context da ist
            ctx = self.scene_context if self.scene_context else "No visual info available. Just ask them to sit anywhere."
            
            self.base_system_prompt = self.base_system_prompt.format(
                host_name=self.stored_data.get("host_name", "Host"),
                guest_name=self.stored_data.get("guest_name", "Guest"),
                guest_drink=self.stored_data.get("guest_drink", "water"),
                scene_context=ctx # NEW: Injected vision context
            )
            
        elif state_num == 9 or state_num == 15 or state_num == 25:
            # State 9, 15, 25: Context Injection
            ctx = self.scene_context if self.scene_context else "No info."
            self.base_system_prompt = self.base_system_prompt.replace("{scene_context}", ctx)
            
        elif state_num == 12:
            # State 12: Guest 2 Welcome
            # Wir injizieren Host Name und First Guest Name
            self.base_system_prompt = self.base_system_prompt.format(
                host_name=self.stored_data.get("host_name", "the Host"),
                first_guest_name=self.stored_data.get("first_guest_name", "the first guest")
            )
            
        elif state_num == 13:
            # State 13: Verify Changes
            ctx = self.scene_context if self.scene_context else "No info."
            self.base_system_prompt = self.base_system_prompt.replace("{scene_context}", ctx)
            self.base_system_prompt += f"\n\nCURRENT SITUATION FROM VISION:\n{ctx}"
            
        elif state_num == 14:
            # State 14: Introduce Guest 2
            ctx = self.scene_context if self.scene_context else "No visual info."
            
            self.base_system_prompt = self.base_system_prompt.format(
                host_name=self.stored_data.get("host_name", "the Host"),
                first_guest_name=self.stored_data.get("first_guest_name", "Guest 1"),
                first_guest_drink=self.stored_data.get("first_guest_drink", "something nice"),
                guest_name=self.stored_data.get("guest_name", "Guest 2"),
                guest_drink=self.stored_data.get("guest_drink", "water"), 
                scene_context=ctx
            )

        elif state_num == 15:
            # State 15: Final Verify (Alle Infos)
            ctx = self.scene_context if self.scene_context else "No visual info."
            
            self.base_system_prompt = self.base_system_prompt.format(
                host_name=self.stored_data.get("host_name", "the Host"),
                first_guest_name=self.stored_data.get("first_guest_name", "Guest 1"),
                guest_name=self.stored_data.get("guest_name", "Guest 2"),
                scene_context=ctx
            )
        
        # Reset messages mit neuem System Prompt
        self.messages = [{"role": "system", "content": self._build_system_prompt()}]
        
        print(f"   Tasks: {list(self.tasks.keys())}")
        print(f"{'='*60}\n")
        
        # Check Auto-Start
        if self.current_config.get("start_on_load"):
            # print("🚀 AUTO-STARTING CONVERSATION...") <-- Zu viel Noise
            # Vermeide Doppelt-Start wenn schon am Processen
            if not self.is_processing and not self.is_speaking:
                self.is_processing = True
                threading.Thread(
                    target=self.process_pipeline, 
                    args=(None, "Please start the introduction now.")
                ).start()

    def set_state_callback(self, msg):
        """Callback für /set_state Topic."""
        new_state = msg.data
        print(f"\n📨 Received /set_state: {new_state}")
        self._load_state(new_state)
        self._print_status()

    def scene_context_callback(self, msg):
        """NEW: Callback for scene context updates (e.g. seat info)."""
        self.scene_context = msg.data
        # print(f"🖼️ Scene Context received: {self.scene_context}") <-- Zu viel Noise
        
        # Reload state to inject new context into the prompt
        # Relevant for states that use {scene_context} or append it
        if self.current_state in [8, 9, 13, 14, 15]:
            # print(f"   🔄 Reloading State {self.current_state} to apply context...")
            self._load_state(self.current_state)

    def _build_system_prompt(self):
        """Baut System Prompt mit aktuellem Task-Status."""
        done_count = sum(1 for t in self.tasks.values() if t["done"])
        total = len(self.tasks)
        pending = [name for name, t in self.tasks.items() if not t["done"]]
        
        status = f"\n\n[STATUS: {done_count}/{total} tasks done."
        if pending:
            status += f" TODO: {', '.join(pending)}]"
        else:
            status += " ALL COMPLETE! Wrap up the conversation.]"
        
        return self.base_system_prompt + status

    def _print_status(self):
        """Zeigt aktuellen Task-Status."""
        print("\n" + "="*60, flush=True)
        print(f"🤖 TIAGO SMART BRAIN - STATE {self.current_state}", flush=True)
        print("="*60, flush=True)
        
        # IDLE Mode?
        if self.current_state == 0:
            print("   💤 IDLE - Warte auf /set_state Command...", flush=True)
            print("="*60 + "\n", flush=True)
            return
        
        for name, task in self.tasks.items():
            icon = "✅" if task["done"] else "⬜"
            value = f" = {task['value']}" if task["value"] else ""
            print(f"   {icon} {name}{value}", flush=True)
        print("-"*60, flush=True)
        print(f"   📁 Stored: host={self.stored_data['host_name']}, guest={self.stored_data['guest_name']}, drink={self.stored_data['guest_drink']}", flush=True)
        if self.scene_context:
             print(f"   🖼️ Context: {self.scene_context}", flush=True)
        print("="*60, flush=True)
        if self.conversation_active:
            print("   🎤 Waiting for speech...", flush=True)
        else:
            print("   🏁 State COMPLETE!", flush=True)
        print("="*60 + "\n", flush=True)

    def _parse_done_tags(self, answer):
        """
        Parst [DONE:TASK] und [DONE:TASK:VALUE] Tags aus der Antwort.
        Gibt bereinigte Antwort (ohne Tags) zurück.
        """
        # Pattern: [DONE:TASK] oder [DONE:TASK:VALUE]
        pattern = r'\[DONE:(\w+)(?::([^\]]+))?\]'
        matches = re.findall(pattern, answer)
        
        name_task = self.current_config.get("name_task")
        drink_task = self.current_config.get("drink_task")
        role = self.current_config.get("role")
        
        for task_name, value in matches:
            task_name = task_name.upper()
            if task_name in self.tasks:
                self.tasks[task_name]["done"] = True
                if value:
                    self.tasks[task_name]["value"] = value.strip()
                    
                    # Speichere extrahierte Daten
                    if task_name == name_task and value:
                        clean_value = value.strip()
                        # Je nach State unterschiedlich speichern
                        if self.current_state == 1:  # Host
                            self.stored_data["host_name"] = clean_value
                            self._publish_guest_data("name", clean_value, "host")
                        else:  # Guest
                            self.stored_data["guest_name"] = clean_value
                            self._publish_guest_data("name", clean_value, role)
                    
                    elif task_name == drink_task and value:
                        clean_value = value.strip()
                        self.stored_data["guest_drink"] = clean_value
                        self._publish_guest_data("drink", clean_value, role)
                
                print(f"✅ DONE: {task_name}" + (f" = {value}" if value else ""))
        
        # Entferne Tags aus der Antwort für TTS
        clean_answer = re.sub(pattern, '', answer).strip()
        
        # Check ob alle Tasks erledigt
        if all(t["done"] for t in self.tasks.values()):
            self.conversation_active = False
            print(f"\n🎉 STATE {self.current_state} COMPLETE!")
            # Publish completion
            self.conversation_done_pub.publish(Int32(self.current_state))
        
        return clean_answer

    def _publish_guest_data(self, field, value, role=None):
        """Published Daten auf /guest_data im Format 'role:field:value' oder 'field:value'."""
        msg = String()
        if role:
            msg.data = f"{role}:{field}:{value}"
        else:
            msg.data = f"{field}:{value}"
        self.guest_data_pub.publish(msg)
        print(f"📤 Update: {msg.data}")

    def _parse_intro_seat_parts(self, answer):
        """
        Parst INTRO: und SEAT: Teile aus der Antwort (für State 8/14).
        Gibt (intro_text, seat_text, chair_number) zurück.
        """
        intro_text = None
        seat_text = None
        chair_number = None
        
        # Parse INTRO: Teil
        intro_match = re.search(r'INTRO:\s*(.+?)(?=SEAT:|$)', answer, re.DOTALL)
        if intro_match:
            intro_text = intro_match.group(1).strip()
        
        # Parse SEAT: Teil
        seat_match = re.search(r'SEAT:\s*(.+?)(?=\[DONE|$)', answer, re.DOTALL)
        if seat_match:
            seat_text = seat_match.group(1).strip()
            
            # Extrahiere Stuhlnummer aus SEAT Teil
            chair_match = re.search(r'[Cc]hair\s*(\d+)', seat_text)
            if chair_match:
                chair_number = int(chair_match.group(1))
        
        return intro_text, seat_text, chair_number
    
    def _point_at_seat(self, seat_id):
        """Sendet PointTo Befehl für einen Stuhl via /goal Topic."""
        rospy.loginfo(f"👉 Zeige auf Platz {seat_id}...")
        
        # 1. Einfache Int32 Message für Logging/Debugging
        msg = Int32()
        msg.data = seat_id
        self.point_to_pub.publish(msg)
        
        # 2. Echte PointToActionGoal an /goal senden
        action_msg = PointToActionGoal()
        action_msg.header = Header()
        action_msg.header.stamp = rospy.Time.now()
        action_msg.header.frame_id = ''
        
        action_msg.goal_id = GoalID()
        action_msg.goal_id.stamp = rospy.Time(0)
        action_msg.goal_id.id = ''
        
        # pose_id entspricht dem Stuhl
        action_msg.goal.pose_id = seat_id
        
        self.point_to_action_pub.publish(action_msg)
        rospy.loginfo(f"   📤 PointTo Action gesendet (pose_id={seat_id})")
        
        # Kurz warten bis Arm anfängt sich zu bewegen
        time.sleep(0.5)

    def audio_callback(self, msg):
        # WICHTIG: Nicht lauschen wenn IDLE oder Conversation nicht aktiv!
        if not self.conversation_active:
            return
        
        if self.is_processing or self.model is None or self.is_speaking: 
            return

        raw_bytes = bytes(msg.data)
        audio_float = np.frombuffer(raw_bytes, dtype=np.int16).astype(np.float32) / 32768.0
        current_volume = np.max(np.abs(audio_float))
        now = time.time()  # Geändert: time.time() statt rospy.Time.now()
        
        if self.is_recording:
            self.audio_buffer.extend(msg.data)
            
            # If we're still validating (first 5 chunks)
            if self.is_validating:
                if current_volume > START_THRESHOLD:
                    self.validation_count += 1
                    print(f"🔄 Validiere+Rec: Vol {current_volume:.3f}/{START_THRESHOLD:.2f} ✅ [{self.validation_count}/{self.required_validation}]")
                    
                    if self.validation_count >= self.required_validation:
                        print("✅ VALIDIERT! Recording läuft nahtlos weiter...")
                        self.is_validating = False
                        self.validation_fails = 0
                        self.last_speech_time = now
                else:
                    # Below threshold during validation - tolerate up to max_validation_fails
                    self.validation_fails += 1
                    if self.validation_fails <= self.max_validation_fails:
                        print(f"⚠️ Vol {current_volume:.3f}/{START_THRESHOLD:.2f} - Kurze Pause toleriert ({self.validation_fails}/{self.max_validation_fails})")
                    else:
                        # Too many fails - cancel recording
                        print(f"❌ Vol {current_volume:.3f}/{START_THRESHOLD:.2f} - Validation fehlgeschlagen! ({self.validation_count}/{self.required_validation}) Aufnahme verworfen!")
                        self.is_recording = False
                        self.is_validating = False
                        self.validation_count = 0
                        self.validation_fails = 0
                        self.audio_buffer = []
                        return
            else:
                # Normal recording (after validation passed)
                if current_volume > CONTINUE_THRESHOLD:
                    self.last_speech_time = now
                    print(f"🔴 Rec: Vol {current_volume:.3f}/{CONTINUE_THRESHOLD:.2f} | hört zu ✅")
                else:
                    # Below threshold - show silence countdown
                    silence = now - self.last_speech_time  # Geändert: einfache Subtraktion
                    warning = "⏳" if silence > 1.0 else ""
                    print(f"🔴 Rec: Vol {current_volume:.3f}/{CONTINUE_THRESHOLD:.2f} | Stille: {silence:.1f}/{SILENCE_TIMEOUT:.1f}s {warning}")

                # Check if silence timeout reached
                silence = now - self.last_speech_time  # Geändert: einfache Subtraktion
                if silence > SILENCE_TIMEOUT:
                    print("⏹️ STOPP! Verarbeite...")
                    self.stop_and_process()
                
        else:
            # Idle - waiting for first chunk over threshold
            if current_volume > START_THRESHOLD:
                print(f"🎤 Lauscht: Vol {current_volume:.3f}/{START_THRESHOLD:.2f} 💥 Start Aufnahme!")
                self.is_recording = True
                self.is_validating = True
                self.validation_count = 1  # This chunk counts as first
                self.audio_buffer = [] 
                self.audio_buffer.extend(msg.data)
            else:
                print(f"🎤 Lauscht: Vol {current_volume:.3f}/{START_THRESHOLD:.2f}")

    def stop_and_process(self):
        data = list(self.audio_buffer)
        self.audio_buffer = []
        self.is_recording = False
        self.is_processing = True
        threading.Thread(target=self.process_pipeline, args=(data,)).start()

    def process_pipeline(self, audio_data, text_input=None):
        """
        Verarbeitet Audio ODER Text-Input.
        Wenn text_input gesetzt ist, wird Whisper übersprungen.
        """
        text = text_input
        
        # A) WHISPER (nur wenn kein Text vorgegeben)
        if text is None:
            try:
                audio_np = np.frombuffer(bytes(audio_data), dtype=np.int16).astype(np.float32) / 32768.0
                segments, _ = self.model.transcribe(audio_np, language="en", beam_size=5)
                text = " ".join([s.text for s in segments]).strip()
            except Exception as e:
                print(f"Whisper Error: {e}")
                self.is_processing = False
                return

        if not text:
            print("❌ Nichts verstanden.")
            self.is_processing = False
            return

        if text_input:
            pass # Auto-Start: Kein Log nötig
        else:
            print(f"\n👂 USER: {text}")

        # B) OPENAI (mit Task-Tracking)
        answer = self.ask_openai(text)
        
        # C) Parse [DONE:...] Tags (Tags entfernen für Sprechen)
        pattern = r'\[DONE:(\w+)(?::([^\]]+))?\]'
        clean_answer = re.sub(pattern, '', answer).strip()
        
        print(f"🤖 TIAGO: {clean_answer}")

        # D) SPRECHEN - Spezialfall für State 8/14 (Intro mit synchronisiertem Pointing)
        if self.current_state in [8, 14]:
            intro_text, seat_text, chair_number = self._parse_intro_seat_parts(clean_answer)
            
            if intro_text and seat_text:
                # 1. Erst INTRO sprechen
                print(f"   📢 [INTRO] {intro_text}")
                self.speak(intro_text)
                
                # 2. Dann auf Stuhl zeigen (während Arm sich bewegt)
                if chair_number:
                    self._point_at_seat(chair_number)
                else:
                    print("   ⚠️ Keine Stuhlnummer erkannt, zeige auf Stuhl 2 als Default")
                    self._point_at_seat(2)
                
                # 3. Dann SEAT-Anweisung sprechen
                print(f"   📢 [SEAT] {seat_text}")
                self.speak(seat_text)
            else:
                # Fallback: Normale Ausgabe wenn Format nicht passt
                print("   ⚠️ INTRO:/SEAT: Format nicht erkannt, normale Ausgabe")
                self.speak(clean_answer)
        else:
            # Normale Ausgabe für alle anderen States
            self.speak(clean_answer)

        # E) LOGS erst NACH dem Sprechen anzeigen
        self._parse_done_tags(answer)
        
        # F) Status anzeigen (NUR wenn Conversation noch läuft!)
        if self.conversation_active:
             self._print_status()

        self.is_processing = False
        
        if self.conversation_active:
            print("\n👂 Höre wieder zu...")
        else:
            # Gespräch beendet - kein weiterer Log nötig, da _parse_done_tags schon "COMPLETE" meldet
            pass

    def ask_openai(self, text):
        if not self.openai_client: 
            return "I have no API key."
        try:
            # Update system prompt mit aktuellem Status
            self.messages[0] = {"role": "system", "content": self._build_system_prompt()}
            
            self.messages.append({"role": "user", "content": text})
            resp = self.openai_client.chat.completions.create(
                model=OPENAI_MODEL, 
                messages=self.messages, 
                max_completion_tokens=MAX_TOKENS
            )
            ans = resp.choices[0].message.content
            self.messages.append({"role": "assistant", "content": ans})
            return ans
        except Exception as e:
            return f"Error: {e}"

    def speak(self, text):
        """
        Publishes TTS message directly to /tts/goal topic.
        Waits for TTS to finish via /tts/result topic.
        """
        self.is_speaking = True
        self.tts_done = False
        print("📢 [TTS] Speaking...", end=" ", flush=True)
        
        try:
            # Create TtsActionGoal message
            msg = TtsActionGoal()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = ''
            
            msg.goal_id = GoalID()
            msg.goal_id.stamp = rospy.Time(0)
            msg.goal_id.id = ''
            
            msg.goal.rawtext.text = text
            msg.goal.rawtext.lang_id = 'en_GB'
            msg.goal.wait_before_speaking = 0.0
            
            # Publish
            self.tts_pub.publish(msg)
            
            # Wait for TTS to finish (with shorter timeout) - using time.time()
            start_time = time.time()
            timeout_sec = 15.0  # 15s max (kürzer!)
            
            while not self.tts_done and (time.time() - start_time) < timeout_sec and not rospy.is_shutdown():
                time.sleep(0.1)  # Einfaches sleep statt rospy.Rate
            
            if self.tts_done:
                print("✅ Done.")
            else:
                print("⚠️ Timeout.")
            
        except Exception as e:
            print(f"\n❌ TTS Fehler: {e}", flush=True)
        finally:
            self.is_speaking = False
    
    def tts_result_callback(self, msg):
        """Called when TTS finishes speaking."""
        # Status 3 = SUCCEEDED
        if msg.status.status == 3:
            self.tts_done = True

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = SmartBrainNode()
    node.run()