"""
This file will contain the throttle setting
"""

class Throttle:
    def __init__(self):
        
        #intilize default throttle
        self.initial_throttle = 1.0 # 100 % at the beginning
        self.throttle = self.initial_throttle
        self.min_throttle =0.0
        self.max_throttle = 1.0

    
    def set_throttle(self, throttle):
        self.throttle = throttle

    def get_throttle(self):
        return self.throttle
    
    def reset(self):
        self.throttle = self.initial_throttle

