#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys, select, termios, tty
import time


class RampInputNode(Node):
    def __init__(self):
        super().__init__('ramp_input_generator')

        self.publisher = self.create_publisher(TwistStamped, 'twist_cmd', 10)
        self.setpoint = 0.0
        self.input = TwistStamped()
        
        self.a = 0.0
        self.b = 0.0
        self.reference_time = 0.0

        self.settings = termios.tcgetattr(sys.stdin)

    def input_function(self):
        elapsed_time = time.time() - self.reference_time
        self.setpoint = self.a*elapsed_time + self.b
        if self.setpoint < 0:
            self.setpoint = 0.0


    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while True:
                key = self.get_key()
                if key == 'g':
                    self.a = 0.2
                    self.b = self.setpoint
                    self.reference_time = time.time()
                elif key == 'x':
                    self.a = -1.5
                    self.b = self.setpoint
                    self.reference_time = time.time()
                elif key == '\x03':  # CTRL-C
                    break

                  
                self.input_function()
                self.input.twist.linear.x = self.setpoint
                self.publisher.publish(self.input)

                print(f"Velocity (ramp setpoint): {self.setpoint}")

        except Exception as e:
            print(e)
        finally:
            self.input.twist.linear.x = 0.0
            self.publisher.publish(self.input)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = RampInputNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
