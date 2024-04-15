import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial

class GpsNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        # Open serial connection
        self.serial = serial.Serial('/dev/gps-module', 9600)

        # Create publisher that will publish GPS fixes as NavSatFix message
        self.publisher = self.create_publisher(
            NavSatFix,
            'gps_fix',
            10)
        
        # Timer will call publish_gps_fix function every 0.5 sec
        # Function reads data from GPS module and publishes GPS fixes to ROS2 topic
        self.timer = self.create_timer(0.5, self.publish_gps_fix)
        
    def publish_gps_fix(self):
        self.serial.flushInput()

        # Read lines from serial until line that starts with $GPGGA (GPS fix) is reached
        while True:
            line = self.serial.readline().decode('utf-8')
            if line.startswith('$GPGGA'):
                break
        
        # Parse GPGGA string
        nmea_lat, lat_direction, nmea_lon, lon_direction = self.parse_gpgga(line)

        # Convert coordinates in NMEA format to decimals
        decimal_lat, decimal_lon = self.nmea_to_decimal(nmea_lat, lat_direction, nmea_lon, lon_direction)

        # Publish decimal latitude and longitude to ROS2 topic
        fix = NavSatFix()
        fix.latitude = decimal_lat
        fix.longitude = decimal_lon

        self.publisher.publish(fix)

    def parse_gpgga(self, line):
        # Split the GPGGA string into its components
        parts = line.split(',')
        
        # Extract NMEA latitude, longitude, and their direction indicators (N,S,E,W)
        nmea_lat = parts[2]
        lat_direction = parts[3]
        nmea_lon = parts[4]
        lon_direction = parts[5]

        return nmea_lat, lat_direction, nmea_lon, lon_direction
    
    def nmea_to_decimal(self, nmea_lat, lat_direction, nmea_lon, lon_direction):
        # Extract degrees and minutes from NMEA format
        lat_degrees = int(nmea_lat[:2])
        lat_minutes = float(nmea_lat[2:])

        lon_degrees = int(nmea_lon[:3])
        lon_minutes = float(nmea_lon[3:])

        # Convert degrees and minutes to decimal
        lat_decimal = lat_degrees + lat_minutes / 60.0
        lon_decimal = lon_degrees + lon_minutes / 60.0

        # Adjust decimal coordinates for direction
        # If latitude is south, add - to latitude
        # If longitude is west, add - to longitude
        if lat_direction == 'S':
            lat_decimal *= -1
        if lon_direction == 'W':
            lon_decimal *= -1

        return lat_decimal, lon_decimal
        
def main():
    rclpy.init()
    gps_node = GpsNode()
    rclpy.spin(gps_node)
    gps_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
