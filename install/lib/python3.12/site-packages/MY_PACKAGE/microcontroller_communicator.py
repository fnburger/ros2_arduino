#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import time
import os

class MicrocontrollerCommunicator(Node):
    def __init__(self):
        super().__init__('microcontroller_communicator')
        self.subscription = self.create_subscription(
            Int32, '/dominant_color_code', self.code_callback, 10)
        
        # Serial setup: Check if port exists before opening
        self.ser = None
        self.port = '/dev/ttyACM0'  # Change this if your Arduino uses /dev/ttyUSB0, etc.
        baudrate = 9600
        if os.path.exists(self.port):
            try:
                self.ser = serial.Serial(self.port, baudrate, timeout=1)
                time.sleep(2)  # Wait for Arduino reset
                self.get_logger().info(f'MicrocontrollerCommunicator started. Serial connected to {self.port}')
            except Exception as e:
                self.get_logger().warn(f'Could not open serial port {self.port}: {e}. Running in simulation mode (logging only).')
        else:
            self.get_logger().warn(f'Serial port {self.port} does not exist. Running in simulation mode (logging only).')

    def code_callback(self, msg):
        code = msg.data
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(bytes([code]))  # Send as single byte (0,1,2)
                self.get_logger().info(f'Sent color code {code} over serial')
            except Exception as e:
                self.get_logger().error(f'Serial send error: {e}')
        else:
            # Simulate: Just log the code (as if sent)
            self.get_logger().info(f'SIMULATION: Would send color code {code} to Arduino (Red=0, Green=1, Blue=2)')

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MicrocontrollerCommunicator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()