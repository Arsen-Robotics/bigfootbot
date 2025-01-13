import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import serial
import gpxpy.gpx

class GpsNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        self.comport = "/dev/gps-module"
        self.baudrate = 9600

        # Initially these flags are set to None, because it is not known if the GPS module is connected or not
        # and if it is receiving GPS data or not
        self.gps_module_connected = None
        self.gps_module_receiving_gps_data = None

        # Create publisher that will publish GPS fixes as NavSatFix message
        self.gps_fix_publisher = self.create_publisher(
            NavSatFix,
            'gps_fix',
            10)

        # Create publisher that will publish GPS ground speed as Float32 message
        self.ground_speed_publisher = self.create_publisher(
            Float32,
            'ground_speed',
            10)
        
        # Timer will call publish_gps_data function every 0.5 sec
        # Function reads data from GPS module and publishes GPS data to ROS2 topics
        self.timer = self.create_timer(0.5, self.publish_gps_data)

        # GPX initialization
        self.gpx = gpxpy.gpx.GPX()
        self.track = gpxpy.gpx.GPXTrack()
        self.gpx.tracks.append(self.track)
        self.track_segment = gpxpy.gpx.GPXTrackSegment()
        self.track.segments.append(self.track_segment)

    # This function tries to open a serial connection to the GPS module
    # If the connection succeeds or fails, it prints a message to the console
    # The flags self.gps_module_connected and self.gps_module_receiving_data are only for the reason
    # that the message is printed only once when the connection is established or lost
    # The function returns the value of self.gps_module_connected
    def connect_to_gps_module(self):
        try:
            self.serial = serial.Serial(self.comport, self.baudrate, timeout=0.2)

        except Exception as e:
            if self.gps_module_connected == True or self.gps_module_connected == None:
                self.gps_module_connected = False
                self.gps_module_receiving_gps_data = None
                self.get_logger().error(f"Failed to open GPS module, retrying...1 e: {e}")

        else:
            if self.gps_module_connected == False or self.gps_module_connected == None:
                self.gps_module_connected = True
                self.get_logger().warning("GPS module connected")

        return self.gps_module_connected

    # def destroy_node(self):
    #     try:
    #         self.get_logger().info("Closing GPS serial port.")
    #         self.serial.close()
    #     except Exception as e:
    #         self.get_logger().warning(f"Failed to close GPS serial port: {e}")
    #     finally:
    #         super().destroy_node()
        
    def publish_gps_data(self):
        try:
            # If the GPS module is not connected, exit the function to avoid errors when calling GPS module
            if not self.connect_to_gps_module():
                return

            got_vtg = False
            got_gga = False

            # Read lines from serial until both, a line that starts with $GPGGA
            # and a line that starts with $GPVTG have been reached
            while not (got_vtg and got_gga):
                line = self.serial.readline().decode('utf-8')

                if line.startswith('$GPVTG') and not got_vtg:
                    # Parse GPVTG string
                    ground_speed_val = self.parse_gpvtg(line)

                    got_vtg = True


                if line.startswith('$GPGGA') and not got_gga:
                    # Parse GPGGA string
                    nmea_lat, lat_direction, nmea_lon, lon_direction = self.parse_gpgga(line)

                    # Convert coordinates in NMEA format to decimals
                    decimal_lat, decimal_lon = self.nmea_to_decimal(nmea_lat, lat_direction, nmea_lon, lon_direction)

                    got_gga = True

        # Even though the connection is checked in the connect_to_gps_module function,
        # the connection can be lost while program is communicating with the GPS module,
        # so SerialException is caught here
        except serial.SerialException:
            self.gps_module_connected = False
            self.gps_module_receiving_gps_data = None
            self.get_logger().error("Failed to open GPS module, retrying...2")

        # This exception is thrown when the GPS module is not sending any GPS fix data,
        # because most likely the module is not receiving GPS signal
        except ValueError:
            if self.gps_module_receiving_gps_data == True or self.gps_module_receiving_gps_data == None:
                self.gps_module_receiving_gps_data = False
                self.get_logger().error("No GPS data received from module. Waiting...")

        # All known exceptions are caught, but if some unknown exception occurs,
        # it is caught here and printed to the console
        except Exception as e:
            self.get_logger().error(f"Exception: {e}")

        # If no exception is thrown, it means that the GPS module is receiving GPS data
        else:
            if self.gps_module_receiving_gps_data == False or self.gps_module_receiving_gps_data == None:
                self.gps_module_receiving_gps_data = True
                self.get_logger().info("Started receiving GPS data")
            
            # Publish ground speed to ROS2 topic
            ground_speed = Float32()
            ground_speed.data = ground_speed_val
            self.ground_speed_publisher.publish(ground_speed)

            # Publish decimal latitude and longitude to ROS2 topic
            fix = NavSatFix()
            fix.latitude = decimal_lat
            fix.longitude = decimal_lon
            self.gps_fix_publisher.publish(fix)

            # Append coordinates to GPX
            self.track_segment.points.append(gpxpy.gpx.GPXTrackPoint(decimal_lat, decimal_lon))

            # Write to GPX file every time a new point is added
            with open("/ros2_ws/src/bfb_gps/gpx/output.gpx", "w") as f:
                f.write(self.gpx.to_xml())

    def parse_gpgga(self, line):
        # Split the GPGGA string into its components
        parts = line.split(',')
        
        # Extract NMEA latitude, longitude, and their direction indicators (N,S,E,W)
        nmea_lat = parts[2]
        lat_direction = parts[3]
        nmea_lon = parts[4]
        lon_direction = parts[5]

        return nmea_lat, lat_direction, nmea_lon, lon_direction

    def parse_gpvtg(self, line):
        # Split the GPGGA string into its components
        parts = line.split(',')

        # Extract ground speed (in km/h)
        ground_speed = float(parts[7])

        return ground_speed
    
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
