#!/usr/bin/env python3
# direct control using joystick

import rclpy
import threading
import rclpy.callback_groups
import rclpy.executors
from rclpy.node import Node
import rclpy.qos
from sensor_msgs.msg import Joy
from sd_msgs.msg import DirectControl, SDControl
from math import ceil

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')

        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        
        # subscribers        
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.vel_sub = self.create_subscription(
            SDControl, 
            "sd_control",
            self.vel_callback,
            10,
            callback_group=self.callback_group
        )
        
        # publisher

        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
        )

        self.control_pub = self.create_publisher(
            DirectControl, 
            "direct_control_cmd",
            qos,
            callback_group=self.callback_group
        )

        #timer
        self.timer = self.create_timer(0.05, self.control_callback, self.callback_group)

        # Vel table (m/s)    0    1    2   3   4   5    6   7   8   9   10  11  12  13
        self.vel_table  =  (100, 100, 100, 90, 80, 70, 65, 60, 55, 45,  35, 25, 15, 10 )

        # TODO: Ajustar os mapeamentos antes de usar o codigo
        # controller mapping

        self.controller_map = {
            "xbox" : { 
            # Botões
            "A": 0, "B": 1, "X": 2, "Y": 3,
            "LB": 4, "RB": 5,
            "View": 6, "Menu": 7,"Xbox": 8,
            "L3": 9,"R3": 10,
            
            # Eixos
            "LX": 0, "LY": 1,
            "RX": 3,"RY": 4,
            "LT": 2,"RT": 5,
            "D-Pad_H": 6,"D-Pad_V": 7},
            
            "ps4" : {
            # Botões
            "X": 0, "Círculo": 1, "Triângulo": 2, "Quadrado": 3,
            "L1": 4,"R1": 5,
            "Share": 8, "Options": 9,"PS": 10,
            "L3": 11,"R3": 12,

            # Eixos
            "LX": 0,"LY": 1,
            "RX": 2,"RY": 3,
            "L2": 4,"R2": 5,
            "D-Pad_H": 6, "D-Pad_V": 7} 
            }
        
        self.lock = threading.Lock()

        self.trigger_left = 0.0             # brake trigger
        self.trigger_right = 0.0            # acceleration trigger
        self.lx = 0.0                       # LX
        self.buttons = [0,0,0,0,0,0,0,0,0,0,0]

        self.current_vel = 0.0

        
        self.message = DirectControl()
        self.message.torque_setpoint = 0.0
        self.message.steer_setpoint = 0.0

        self.max_throttle = 30
        self.max_steer = 100
        self.max_brake = -40
        self.additional_brake = 0

        self.steering_rate = 1
        self.center_steer = 0

        self.data = [self.trigger_left, self.trigger_right, self.lx, 
                         self.max_throttle, self.max_brake, self.additional_brake, self.max_steer, self.center_steer]


    def joy_callback(self, msg):
        
        self.trigger_left =  (1 - msg.axes[(self.controller_map["xbox"])["LT"]])*0.5
        self.trigger_right = (1 - msg.axes[(self.controller_map["xbox"])["RT"]])*0.5

        self.lx = msg.axes[(self.controller_map["xbox"])["LX"]]
        self.center_steer = msg.buttons[(self.controller_map['xbox'])["L3"]]

        if 1 in msg.buttons[:4]:
            if msg.buttons[(self.controller_map["xbox"])["Y"]] == 1 and self.buttons[(self.controller_map["xbox"])["Y"]] == 0:
                self.max_throttle = min((self.max_throttle + 30), 100)
            elif msg.buttons[(self.controller_map["xbox"])["B"]] == 1 and self.buttons[(self.controller_map["xbox"])["B"]] == 0:
                self.max_throttle = max((self.max_throttle - 30), 30)
            if msg.buttons[(self.controller_map["xbox"])["X"]] == 1 and self.buttons[(self.controller_map["xbox"])["X"]] == 0: 
                self.additional_brake -= 20 if self.additional_brake > -60 else 0
            elif msg.buttons[(self.controller_map["xbox"])["A"]] == 1 and self.buttons[(self.controller_map["xbox"])["A"]] == 0:    
                self.additional_brake += 20 if self.additional_brake < 0 else 0

        self.buttons = msg.buttons

        with self.lock:
            self.data[0] = self.trigger_left
            self.data[1] = self.trigger_right
            self.data[2] = self.lx 
            self.data[3] = self.max_throttle
            self.data[4] = self.max_brake 
            self.data[5] = self.additional_brake
            self.data[7] = self.center_steer

    def vel_callback(self, msg):
        self.current_vel = msg.current_velocity
        vel_index = ceil(self.current_vel)

        max_steering = self.vel_table[vel_index] if vel_index <= 13 else self.vel_table[-1]

        if max_steering != self.max_steer:
            self.max_steer = max_steering
            with self.lock:
                self.data[6] = max_steering
                


    def control_callback(self):
        # calculating torque
        with self.lock:
            if self.data[0] > 0:
                self.message.torque_setpoint = self.data[0] * self.data[4]
            else:
                self.message.torque_setpoint = self.data[1] * self.data[3]
            
            self.message.torque_setpoint += self.data[5]

            #calculating steer - the controller imput works as a steering rate
            if self.data[7] == 1:
                self.message.steer_setpoint = 0.0
            else:
                self.message.steer_setpoint = max(min((self.message.steer_setpoint + self.steering_rate * self.data[2]), float(self.data[6])), -1*float(self.data[6]))

        self.control_pub.publish(self.message)
        print(f"Torque Setpoint: {self.message.torque_setpoint} ; Steering Setpoint: {self.message.steer_setpoint}")


def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    
    # Executor multithreaded
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando nó...")
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()


# TODO: Aumentar a deadzone
#       -> ros2 run joy joy_node --ros-args -p deadzone:=0.15
# TODO: Corrigir valores dos gatilhos
#       -> ok
# TODO: SATURAR STEERING
#       -> ok
# TODO: SATURAR TORQUE
#       -> ok
# TODO: ADICIONAR FUNCIONALIDADE DO R3
#       -> ok
