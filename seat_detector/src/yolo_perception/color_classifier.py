#!/usr/bin/env python3
import cv2
import numpy as np

class ColorClassifier:
    """
    #TODO: make more refined later, maybe adjust for the yellow camera
    Robust Color Classifier optimized for cameras with heavy yellow tint/warm white balance.
    Based on log data:
    - Brown/Dark objects appear as Dark Orange (H=25, S=200, V=50)
    - White objects appear as Bright Yellow (H=33, S=120, V=160)
    """
    def __init__(self):
        pass

    def get_color_name(self, h, s, v):
        """
        Converts HSV values to a string name.
        """
        # --- 1. SPECIAL RULES FOR "BROKEN" WHITE BALANCE ---
        
        # RULE A: BROWN / DARK GREY
        # If hue is Orange/Yellow (10-40) BUT Value is low (< 90), it's Brown or Dark Grey.
        # Log data showed V=45-55 for brown pants.
        if (10 <= h <= 45) and v < 90:
            return "Brown"

        # RULE B: WHITE / LIGHT GREY
        # Standard white has S < 30. Your camera gives S=120 for white!
        # We increase tolerance but require High Value (Brightness) to distinguish from real Yellow.
        # Log data showed S=115-125 and V=160+ for white shirt.
        if s < 135: 
            if v > 140:
                return "White"
            elif v > 40: # If it's not bright enough for white, but low saturation
                return "Grey"
        
        # RULE C: BLACK
        # Very low value is always black
        if v < 40:
            return "Black"

        # --- 2. STANDARD COLOR RANGES (Adjusted) ---
        
        # Red spans 0-10 and 170-180
        if (0 <= h <= 10) or (170 <= h <= 180):
            return "Red"
        
        # Orange usually 11-25
        elif 11 <= h <= 25:
            return "Orange"
        
        # Yellow usually 26-35
        # Since White is filtered above (S<135), real yellow must have S > 135
        elif 26 <= h <= 35:
            return "Yellow"
        
        # Green
        elif 36 <= h <= 85:
            return "Green"
        
        # Blue
        elif 86 <= h <= 130:
            return "Blue"
        
        # Purple
        elif 131 <= h <= 165:
            return "Purple"
            
        # Pink
        elif 166 <= h <= 169:
            return "Pink"
        
        return "Unknown"

    def analyze_region(self, image_crop, mask_crop):
        if image_crop.size == 0 or mask_crop is None:
            return "Unknown"

        if cv2.countNonZero(mask_crop) < 10: 
            return "Unknown"

        hsv_img = cv2.cvtColor(image_crop, cv2.COLOR_BGR2HSV)
        mask_flat = mask_crop.flatten()
        valid_indices = mask_flat > 0
        
        h_vals = hsv_img[:,:,0].flatten()[valid_indices]
        s_vals = hsv_img[:,:,1].flatten()[valid_indices]
        v_vals = hsv_img[:,:,2].flatten()[valid_indices]

        if len(h_vals) == 0:
            return "Unknown"

        h_median = np.median(h_vals)
        s_median = np.median(s_vals)
        v_median = np.median(v_vals)
        
        # DEBUG PRINT - KEEP THIS until calibration is perfect
        result = self.get_color_name(h_median, s_median, v_median)
        # print(f"HSV: {h_median:.0f}, {s_median:.0f}, {v_median:.0f} -> {result}")
        
        return result