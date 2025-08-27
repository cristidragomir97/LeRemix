#!/usr/bin/env python3
from sensor_msgs.msg import Joy

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def normalize_axis(raw):
    if abs(raw) > 1.5:
        return clamp(raw / 32767.0, -1.0, 1.0)
    return clamp(raw, -1.0, 1.0)

class JoystickDriver:
    def __init__(self, trigger_threshold=0.3):
        self.trigger_threshold = trigger_threshold
        
        # Xbox controller mapping
        self.axis_ls_x = 0
        self.axis_ls_y = 1
        self.axis_lt   = 2
        self.axis_rs_x = 3
        self.axis_rs_y = 4
        self.axis_rt   = 5
        
        self.btn_a    = 0
        self.btn_b    = 1
        self.btn_x    = 2
        self.btn_y    = 3
        self.btn_lb   = 4
        self.btn_rb   = 5
        self.btn_back = 6
        self.btn_start= 7
        self.btn_lsb  = 9
        self.btn_rsb  = 10
        
        # Button state tracking for edge detection
        self.prev_rt_pressed = False
        self.prev_lt_pressed = False
        self.prev_rb_pressed = False
        self.prev_lb_pressed = False
        
        # Previous button states for all buttons
        self.prev_buttons = {}

    def _axis(self, axes, idx, invert=False):
        if idx < 0 or idx >= len(axes):
            return 0.0
        val = normalize_axis(axes[idx])
        return -val if invert else val

    def _btn(self, btns, idx):
        return (idx < len(btns)) and (btns[idx] == 1)

    def process_joy_message(self, msg: Joy):
        """Process a Joy message and return a dictionary of events"""
        axes = msg.axes
        btns = msg.buttons
        
        events = {}
        
        # Handle analog sticks (always active, no button press required)
        ls_x = self._axis(axes, self.axis_ls_x)
        ls_y = self._axis(axes, self.axis_ls_y, invert=True)
        rs_x = self._axis(axes, self.axis_rs_x)
        rs_y = self._axis(axes, self.axis_rs_y, invert=True)
        
        events['left_stick'] = {'x': ls_x, 'y': ls_y}
        events['right_stick'] = {'x': rs_x, 'y': rs_y}
        
        # Handle face buttons with press/release events
        face_buttons = {
            'a': self.btn_a,
            'b': self.btn_b,
            'x': self.btn_x,
            'y': self.btn_y,
            'back': self.btn_back,
            'start': self.btn_start,
            'lsb': self.btn_lsb,
            'rsb': self.btn_rsb
        }
        
        for name, idx in face_buttons.items():
            current_pressed = self._btn(btns, idx)
            prev_pressed = self.prev_buttons.get(name, False)
            
            if current_pressed and not prev_pressed:
                events[f'{name}_press'] = True
            elif not current_pressed and prev_pressed:
                events[f'{name}_release'] = True
                
            events[name] = current_pressed
            self.prev_buttons[name] = current_pressed
        
        # Convert triggers to button presses based on threshold
        rt_val = self._axis(axes, self.axis_rt)
        lt_val = self._axis(axes, self.axis_lt)
        
        rt_pressed = abs(rt_val) > self.trigger_threshold
        lt_pressed = abs(lt_val) > self.trigger_threshold
        
        # RT button events
        if rt_pressed and not self.prev_rt_pressed:
            events['rt_press'] = True
            events['rt_direction'] = 1 if rt_val > 0 else -1
        elif not rt_pressed and self.prev_rt_pressed:
            events['rt_release'] = True
            
        events['rt'] = rt_pressed
        events['rt_value'] = rt_val
        self.prev_rt_pressed = rt_pressed
        
        # LT button events
        if lt_pressed and not self.prev_lt_pressed:
            events['lt_press'] = True
            events['lt_direction'] = 1 if lt_val > 0 else -1
        elif not lt_pressed and self.prev_lt_pressed:
            events['lt_release'] = True
            
        events['lt'] = lt_pressed
        events['lt_value'] = lt_val
        self.prev_lt_pressed = lt_pressed
        
        # Handle shoulder buttons with press/release events
        rb_pressed = self._btn(btns, self.btn_rb)
        lb_pressed = self._btn(btns, self.btn_lb)
        
        if rb_pressed and not self.prev_rb_pressed:
            events['rb_press'] = True
        elif not rb_pressed and self.prev_rb_pressed:
            events['rb_release'] = True
            
        events['rb'] = rb_pressed
        self.prev_rb_pressed = rb_pressed
        
        if lb_pressed and not self.prev_lb_pressed:
            events['lb_press'] = True
        elif not lb_pressed and self.prev_lb_pressed:
            events['lb_release'] = True
            
        events['lb'] = lb_pressed
        self.prev_lb_pressed = lb_pressed
        
        return events