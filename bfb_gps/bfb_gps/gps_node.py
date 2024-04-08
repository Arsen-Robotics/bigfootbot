import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import pynmea2
import serial

class GpsNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        self.serial = serial.Serial('/dev/ttyACM0', 9600)

        self.publisher = self.create_publisher(
            NavSatFix,
            'gps_fix',
            10)
        
        self.timer = self.create_timer(0.6, self.publish_callback)
        
    def publish_callback(self):
        while True:
            line = self.serial.readline().decode('utf-8')
            if line.startswith('$GPGGA'):
                self.serial.flushInput()
                break

        line = line.strip()
        parsed_nmea = pynmea2.parse(line)

        fix = NavSatFix()
        fix.latitude = float(parsed_nmea.lat)
        fix.longitude = float(parsed_nmea.lon)

        self.publisher.publish(fix)
        
def main():
    rclpy.init()
    gps_node = GpsNode()
    rclpy.spin(gps_node)
    gps_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
