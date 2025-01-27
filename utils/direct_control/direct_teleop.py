#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sd_msgs.msg import DirectControl, SDControl
import sys, select, termios, tty
import time


class DirectTeleop(Node):
    def __init__(self):
        super().__init__('direct_teleop')

        self.declare_parameter('torque_increase_rate', 10.0)
        self.torque_increase_rate = self.get_parameter('torque_increase_rate').get_parameter_value().double_value

        self.declare_parameter('steering_increase_rate', 5.0)
        self.steering_increase_rate = self.get_parameter('steering_increase_rate').get_parameter_value().double_value

        self.declare_parameter('max_torque', 100.0)
        self.max_torque = self.get_parameter('max_torque').get_parameter_value().double_value

        self.declare_parameter('max_steering_value', 100.0)
        self.max_steering_value = self.get_parameter('max_steering_value').get_parameter_value().double_value

        self.publisher = self.create_publisher(DirectControl, 'direct_control_cmd', 10)
        self.subscriber = self.create_subscription(SDControl, 'sd_control', self.vel_callback, 10)

        self.control = DirectControl()
        self.current_torque = 0.0
        self.current_steering = 0.0
        self.previous_steering = 0.0
        self.current_vel = 0
        self.reference_time = 0
        self.steering_ramp_intersection = 0
        self.torque_ramp_intersection = 0
        self.last_key = ''
        self.key = ''
        self.settings = termios.tcgetattr(sys.stdin)

        self.dict = {'w':self.foward, 's':self.brake, 'a':self.left, 'd':self.right, 
                     'x':self.stop, '1':self.step_torque, '8':self.step_steer_left, '9':self.step_steer_right, 
                     'i':self.steer_ramp, 'o':self.steer_ramp, 'r':self.torque_ramp, 'z':self.center_steer}

        print("""
Reading from the keyboard and Publishing to TwistStamped!
Uses "w, a, s, d, x" keys
---------------------------
Move forward:
   'w'
Move backward:
    's'
Turn left:
    'a'
Turn right:
    'd'
Stop:
    'x'
    
CTRL-C to quit
""")
    
    def vel_callback(self, msg):
        self.current_vel = msg.current_velocity 

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def foward(self):
        self.current_torque += self.torque_increase_rate
        self.last_key = self.key

    def brake(self):
        self.current_torque -= self.torque_increase_rate
        self.last_key = self.key

    def left(self):
        self.current_steering += self.steering_increase_rate
        self.last_key = self.key

    def right(self):
        self.current_steering-= self.steering_increase_rate
        self.last_key = self.key

    def step_torque(self):
        self.current_torque = 40.0 #torque step
        self.last_key = self.key

    def stop(self):
        self.current_torque = -50.0
        self.current_steering = self.previous_steering
        self.last_key = self.key

    def step_steer_left(self):
        if self.current_vel < 4:
            self.current_steering = 50.0 # step setpoint (left)
            self.last_key = self.key
    
    def step_steer_right(self):
        if self.current_vel < 4:
            self.current_steering = -50.0 # step setpoint (left)
        self.last_key = self.key
    
    def steer_ramp(self):
        self.reference_time = time.time() # steer ramp setpoint 
        self.steering_ramp_intersection = self.current_steering
        self.last_key = self.key  

    def torque_ramp(self):
        self.reference_time = time.time()
        self.torque_ramp_intersection = self.current_torque
        self.last_key = self.key

    def center_steer(self):
        self.current_steering = 0.0
        self.last_key = self.key

    def run(self):
        try:
            while True:
                self.key = self.get_key().lower()
                if self.key == '\x03':  # CTRL-C
                    break
                
                func = self.dict.get(self.key, None)
                if func != None: func()
            
                elapsed_time = time.time() - self.reference_time
                if self.last_key == 'i':
                    self.current_steering = 2*elapsed_time + self.steering_ramp_intersection
                elif self.last_key == 'o':
                    self.current_steering = -2*elapsed_time + self.steering_ramp_intersection
                elif self.last_key == 'r':
                    self.current_torque = 3*elapsed_time + self.torque_ramp_intersection

                self.current_torque = min(max(self.current_torque, -self.max_torque), self.max_torque)
                self.current_steering = min(max(self.current_steering, -self.max_steering_value), self.max_steering_value)

                self.control.torque_setpoint = self.current_torque
                self.control.steer_setpoint = self.current_steering
                self.previous_steering = self.current_steering
                self.publisher.publish(self.control)

                print(f"Torque Setpoint: {self.current_torque}, Steering Value: {self.current_steering}")

        except Exception as e:
            print(e)
        finally:
            self.control.torque_setpoint  = 0.0
            self.control.steer_setpoint = 0.0
            self.publisher.publish(self.control)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = DirectTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

