#!/usr/bin/env python3
import rospy
import numpy as np
import csv
import resampy
from std_msgs.msg import String
from messages.msg import AudioData
import os
import rospkg

# Import TFLite
try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    try:
        import tensorflow.lite as tflite
    except ImportError:
        print("ERROR: Please run 'pip3 install tflite-runtime'!")
        exit(1)

class DualModelNode:
    def __init__(self):
        rospy.init_node('dual_audio_detector', anonymous=True)
        
        # --- SETTINGS ---
        self.DEBUG_RAW = True
        self.GAIN_FACTOR = 1.0 
        
        # TRIGGER LOGIC
        # If Index 0 OR Index 1 rise above this value, it's considered an event.
        self.CUSTOM_TRIGGER_THRESHOLD = 0.80 
        
        # YAMNet Memory
        self.YAMNET_MEM_THRESHOLD = 0.10 
        self.MEMORY_DURATION = 0.6  # Short memory for synchronization
        self.REQUIRED_HITS = 1      # 1 hit is enough since we double-check
        
        self.hits_knock = 0
        self.hits_bell = 0
        self.last_seen_knock = 0
        self.last_seen_bell = 0

        # --- PATHS ---
        try:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('audio_detection') 
            script_path = os.path.join(pkg_path, "scripts")
            
            self.PATH_YAMNET = os.path.join(script_path, "yamnet.tflite")
            self.PATH_YAMNET_LABELS = os.path.join(script_path, "yamnet_class_map.csv")
            self.PATH_CUSTOM = os.path.join(script_path, "doorbell_detector_model.tflite")
            
        except Exception as e:
            rospy.logerr(f"Path error: {e}")
            exit(1)

        self.init_yamnet()
        self.init_custom()
        
        self.RATE_IN = 16000 
        self.audio_buffer = np.array([], dtype=np.float32)
        
        self.pub = rospy.Publisher('/audio/event', String, queue_size=1)
        self.sub = rospy.Subscriber(rospy.get_param('~audio_topic', '/audio'), AudioData, self.audio_callback)
        
        self.last_event_time = 0
        self.last_debug_time = 0

    def init_yamnet(self):
        rospy.loginfo("Loading YAMNet...")
        self.interpreter_y = tflite.Interpreter(model_path=self.PATH_YAMNET)
        self.interpreter_y.allocate_tensors()
        self.in_idx_y = self.interpreter_y.get_input_details()[0]['index']
        self.out_idx_y = self.interpreter_y.get_output_details()[0]['index']
        
        self.labels_y = []
        with open(self.PATH_YAMNET_LABELS, 'r') as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                self.labels_y.append(row[2])
                
        self.yamnet_groups = {'knock': [], 'bell': []}
        knock_terms = ['knock', 'door', 'tap', 'wood', 'thump'] 
        bell_terms = ['doorbell', 'ding', 'chime', 'bell', 'buzzer']
        
        for i, name in enumerate(self.labels_y):
            n = name.lower()
            if any(t in n for t in knock_terms):
                if i not in self.yamnet_groups['knock']: self.yamnet_groups['knock'].append(i)
            if any(t in n for t in bell_terms):
                if i not in self.yamnet_groups['bell']: self.yamnet_groups['bell'].append(i)

    def init_custom(self):
        rospy.loginfo("Loading Custom Model...")
        self.interpreter_c = tflite.Interpreter(model_path=self.PATH_CUSTOM)
        self.interpreter_c.allocate_tensors()
        self.in_details_c = self.interpreter_c.get_input_details()
        self.out_idx_c = self.interpreter_c.get_output_details()[0]['index']
        self.custom_input_size = self.in_details_c[0]['shape'][1]
        
        # Index 0 and 1 are the events (Doorbell/Knocking mixed)
        # Index 2 (background) is ignored
        print("Mapping: Checking only Index 0 and 1 for activity.")

    def predict_yamnet(self, audio_16k):
        if len(audio_16k) < 15600: return {'knock': 0.0, 'bell': 0.0}
        input_data = audio_16k[:15600]
        max_val = np.max(np.abs(input_data))
        if max_val > 0.0001: 
            input_data = input_data / max_val * self.GAIN_FACTOR
        
        self.interpreter_y.set_tensor(self.in_idx_y, input_data.astype(np.float32))
        self.interpreter_y.invoke()
        scores = self.interpreter_y.get_tensor(self.out_idx_y)[0]
        
        k_score = sum([scores[i] for i in self.yamnet_groups['knock']])
        b_score = sum([scores[i] for i in self.yamnet_groups['bell']])
        return {'knock': k_score, 'bell': b_score}

    def predict_custom(self, audio_16k):
        target_sr = 44100
        if self.RATE_IN != target_sr:
            resampled = resampy.resample(audio_16k, self.RATE_IN, target_sr)
        else:
            resampled = audio_16k

        if len(resampled) < self.custom_input_size:
            resampled = np.pad(resampled, (0, self.custom_input_size - len(resampled)))
        else:
            resampled = resampled[:self.custom_input_size]
            
        input_data = np.expand_dims(resampled, axis=0).astype(np.float32)
        self.interpreter_c.set_tensor(self.in_details_c[0]['index'], input_data)
        self.interpreter_c.invoke()
        scores = self.interpreter_c.get_tensor(self.out_idx_c)[0]
        
        return {'idx0': scores[0], 'idx1': scores[1], 'idx2': scores[2]}

    def audio_callback(self, msg):
        try:
            if msg.sample_rate > 0: self.RATE_IN = msg.sample_rate
            data = np.frombuffer(msg.data, dtype=np.int16)
            if msg.channels == 2: data = data[::2]
            data = data.astype(np.float32) / 32768.0
            
            self.audio_buffer = np.concatenate((self.audio_buffer, data))
            
            if len(self.audio_buffer) > 48000: self.audio_buffer = self.audio_buffer[-16000:]

            if len(self.audio_buffer) >= 16000:
                chunk = self.audio_buffer[:16000]
                res_y = self.predict_yamnet(chunk)
                res_c = self.predict_custom(chunk)
                self.check_fusion(res_y, res_c)
                self.audio_buffer = self.audio_buffer[8000:]
                
        except Exception as e:
            rospy.logerr(f"Callback Err: {e}")

    def check_fusion(self, y, c):
        event_out = None
        info = ""
        now = rospy.Time.now().to_sec()
        
        # --- 1. YAMNET MEMORY ---
        if y['knock'] > self.YAMNET_MEM_THRESHOLD: self.last_seen_knock = now
        if y['bell'] > self.YAMNET_MEM_THRESHOLD: self.last_seen_bell = now
            
        # --- 2. CUSTOM TRIGGER CHECK ---
        # We only check if Index 0 OR Index 1 is loud.
        # "max" takes the higher of the two.
        activity_level = max(c['idx0'], c['idx1'])
        
        is_triggered = activity_level > self.CUSTOM_TRIGGER_THRESHOLD
        
        # --- 3. DECISION (YAMNet) ---
        yamnet_says_knock = (now - self.last_seen_knock) < self.MEMORY_DURATION
        yamnet_says_bell = (now - self.last_seen_bell) < self.MEMORY_DURATION

        if is_triggered:
            # Custom Model triggers: "THERE IS SOMETHING!"
            # Now we check YAMNet memory to identify what it was.
            
            # Case A: Knocking
            if yamnet_says_knock and not yamnet_says_bell:
                self.hits_knock += 1
                self.hits_bell = 0
            
            # Case B: Doorbell
            elif yamnet_says_bell and not yamnet_says_knock:
                self.hits_bell += 1
                self.hits_knock = 0
            
            # Case C: Conflict (Both? Take the stronger one at the current moment)
            elif yamnet_says_knock and yamnet_says_bell:
                if y['knock'] > y['bell']:
                    self.hits_knock += 1
                    self.hits_bell = 0
                else:
                    self.hits_bell += 1
                    self.hits_knock = 0
            else:
                # Custom triggers, but YAMNet heard nothing (Noise/False alarm)
                self.hits_knock = 0
                self.hits_bell = 0
        else:
            self.hits_knock = 0
            self.hits_bell = 0

        # --- DEBUG OUTPUT ---
        if (now - self.last_debug_time) > 1.0:
            trig_str = "LOUD" if is_triggered else "."
            mem_k = "yes" if yamnet_says_knock else "."
            mem_b = "yes" if yamnet_says_bell else "."
            
            print(f"DEBUG | Event(0/1):{activity_level:.2f} ({trig_str}) | MEM: K={mem_k} B={mem_b}")
            self.last_debug_time = now

        # --- ALARM ---
        if self.hits_knock >= self.REQUIRED_HITS:
            event_out = "KNOCK"
            info = f"Val:{activity_level:.2f}"
            
        elif self.hits_bell >= self.REQUIRED_HITS:
            event_out = "DOORBELL"
            info = f"Val:{activity_level:.2f}"

        if event_out:
            if (now - self.last_event_time) > 2.0:
                print(f"\n>>> ALARM: {event_out} [{info}] <<<
")
                self.pub.publish(event_out)
                self.last_event_time = now
                self.hits_knock = 0
                self.hits_bell = 0

    def run(self):
        print("=== DUAL AI DETECTOR (EVENT MODE) ===")
        print(f"Trigger at Index 0 or 1 > {self.CUSTOM_TRIGGER_THRESHOLD}")
        rospy.spin()

if __name__ == '__main__':
    DualModelNode().run()
