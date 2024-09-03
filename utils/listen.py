#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import can
import cantools
from std_msgs.msg import String

class StreetDroneCANListener(Node):
    def __init__(self):
        super().__init__('streetdrone_can_listener')

        # Load the DBC file
        dbc_file_path = '/root/can/streetdrone_can.dbc'  # Update this path as needed
        self.db = cantools.database.load_file(dbc_file_path)

        # Initialize CAN interface
        self.can_bus = can.interface.Bus(channel='can2', interface='socketcan')

        # Create publishers for each specific message type
        self._publishers = {
            'BMC_Acceleration': self.create_publisher(String, 'BMC_Acceleration', 10),
            'BMC_MagneticField': self.create_publisher(String, 'BMC_MagneticField', 10),
            'Customer_Control_1': self.create_publisher(String, 'Customer_Control_1', 10),
            'GPS_CourseSpeed': self.create_publisher(String, 'GPS_CourseSpeed', 10),
            'GPS_DateTime': self.create_publisher(String, 'GPS_DateTime', 10),
            'GPS_Delusions_A': self.create_publisher(String, 'GPS_Delusions_A', 10),
            'GPS_Delusions_B': self.create_publisher(String, 'GPS_Delusions_B', 10),
            'GPS_PositionAltitude': self.create_publisher(String, 'GPS_PositionAltitude', 10),
            'GPS_PositionLatitude': self.create_publisher(String, 'GPS_PositionLatitude', 10),
            'GPS_PositionLongitude': self.create_publisher(String, 'GPS_PositionLongitude', 10),
            'GPS_Status': self.create_publisher(String, 'GPS_Status', 10),
            'IO': self.create_publisher(String, 'IO', 10),
            'L3GD20_Rotation_A': self.create_publisher(String, 'L3GD20_Rotation_A', 10),
            'L3GD20_Rotation_B': self.create_publisher(String, 'L3GD20_Rotation_B', 10),
            'RTC_DateTime': self.create_publisher(String, 'RTC_DateTime', 10),
            'StreetDrone_Control_1': self.create_publisher(String, 'StreetDrone_Control_1', 10),
            'StreetDrone_Data_1': self.create_publisher(String, 'StreetDrone_Data_1', 10),
            'StreetDrone_Data_2': self.create_publisher(String, 'StreetDrone_Data_2', 10),
            'StreetDrone_Data_3': self.create_publisher(String, 'StreetDrone_Data_3', 10),
            'StreetDrone_Diag_CAN': self.create_publisher(String, 'StreetDrone_Diag_CAN', 10),
            'StreetDrone_Diag_Error_Code': self.create_publisher(String, 'StreetDrone_Diag_Error_Code', 10),
            'StreetDrone_Diag_Error_Raw_A': self.create_publisher(String, 'StreetDrone_Diag_Error_Raw_A', 10),
            'StreetDrone_Diag_Error_Raw_B': self.create_publisher(String, 'StreetDrone_Diag_Error_Raw_B', 10),
            'StreetDrone_Diag_Error_Raw_Delta': self.create_publisher(String, 'StreetDrone_Diag_Error_Raw_Delta', 10),
            'StreetDrone_Diag_XCU_State': self.create_publisher(String, 'StreetDrone_Diag_XCU_State', 10),
            'StreetDrone_Diag_XCU_Time': self.create_publisher(String, 'StreetDrone_Diag_XCU_Time', 10),
        }

        # Create a timer to check for CAN messages
        self.timer = self.create_timer(0.01, self.read_can_messages)

    def read_can_messages(self):
        message = self.can_bus.recv(0.1)
        if message:
            try:
                decoded_message = self.db.decode_message(message.arbitration_id, message.data)
                message_name = self.db.get_message_by_frame_id(message.arbitration_id).name

                # Publish the decoded message to the corresponding topic
                if message_name in self._publishers:
                    self._publishers[message_name].publish(String(data=str(decoded_message)))
                    self.get_logger().info(f"Published {message_name}: {decoded_message}")
                else:
                    self.get_logger().warn(f"Unknown message: {message_name}")

            except Exception as e:
                self.get_logger().error(f"Failed to decode message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = StreetDroneCANListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
