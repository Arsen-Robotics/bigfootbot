import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import pynmea2
import serial

class GpsNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        self.serial = serial.Serial('/dev/gps-module', 9600)

        self.publisher = self.create_publisher(
            NavSatFix,
            'gps_fix',
            10)
        
        self.timer = self.create_timer(0.6, self.publish_callback)
        
    def publish_callback(self):
        self.serial.flushInput()
        while True:
            line = self.serial.readline().decode('utf-8')
            if line.startswith('$GPGGA'):
                break
        
        # Parse GPGGA string
        nmea_lat, lat_direction, nmea_lon, lon_direction = self.parse_gpgga(line)

        # Convert to decimal
        decimal_lat, decimal_lon = self.nmea_to_decimal(nmea_lat, lat_direction, nmea_lon, lon_direction)

        #line = line.strip()
        #parsed_nmea = pynmea2.parse('$GPGGA,115739.00,4158.8441367,N,09147.4416929,W,4,13,0.9,255.747,M,-32.00,M,01,0000*6E')

        fix = NavSatFix()
        fix.latitude = decimal_lat
        fix.longitude = decimal_lon

        self.publisher.publish(fix)

    def parse_gpgga(self, gpgga_string):
        # Split the GPGGA string into its components
        parts = gpgga_string.split(',')
        
        # Extract NMEA latitude, longitude, and their direction indicators
        nmea_lat = parts[2]
        lat_direction = parts[3]
        nmea_lon = parts[4]
        lon_direction = parts[5]

        return nmea_lat, lat_direction, nmea_lon, lon_direction
    
    def nmea_to_decimal(self, nmea_lat, lat_direction, nmea_lon, lon_direction):
        lat_degrees = int(nmea_lat[:2])
        lat_minutes = float(nmea_lat[2:])

        lon_degrees = int(nmea_lon[:3])
        lon_minutes = float(nmea_lon[3:])

        # Conversion from minutes to decimal
        lat_decimal = lat_degrees + lat_minutes / 60.0
        lon_decimal = lon_degrees + lon_minutes / 60.0

        # Adjust for direction
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
