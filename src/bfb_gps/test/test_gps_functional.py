#!/usr/bin/env python3
"""
Functional tests for the GPS node module.

This module contains comprehensive tests for the GPS functionality,
including NMEA parsing, coordinate conversion, and error handling.

For beginners: This file demonstrates how to write unit tests step by step.
Each test function follows the Arrange-Act-Assert pattern:
- Arrange: Set up the test data and conditions
- Act: Execute the function being tested
- Assert: Verify the results are correct
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the parent directory to the path so we can import the module under test
# This is necessary because the test file is in a different directory than the source code
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Import the class we want to test
from bfb_gps.gps_node import GpsNode


class TestGpsNodeNmeaParsing(unittest.TestCase):
    """
    Test cases for NMEA sentence parsing functionality.
    
    NMEA (National Marine Electronics Association) is a standard format
    for GPS data. GPS devices send text sentences that contain location,
    speed, and other information.
    
    This test class focuses on testing the parsing of these NMEA sentences.
    """

    def setUp(self):
        """
        Set up test fixtures before each test method.
        
        This method runs before every test function. It's used to:
        - Create objects that will be used in multiple tests
        - Set up common test data
        - Mock dependencies that aren't being tested
        
        Think of it as "preparing the test environment"
        """
        # Mock ROS2 components because we don't want to test ROS2 itself
        # We only want to test our GPS parsing logic
        with patch('rclpy.init'), \
             patch('rclpy.node.Node.__init__', return_value=None), \
             patch('rclpy.node.Node.declare_parameter'), \
             patch('rclpy.node.Node.get_parameter'), \
             patch('rclpy.node.Node.create_publisher'), \
             patch('rclpy.node.Node.create_timer'), \
             patch('rclpy.node.Node.get_logger'):
            
            # Create an instance of our GPS node for testing
            self.gps_node = GpsNode()
            
            # Set up some test data that we'll use in multiple tests
            self.test_comport = "/dev/test_gps"
            self.test_baudrate = 9600

    def tearDown(self):
        """
        Clean up after each test method.
        
        This method runs after every test function. It's used to:
        - Close files or connections
        - Clean up resources
        - Reset any changes made during the test
        
        Think of it as "cleaning up after the test"
        """
        # Close any serial connections that might have been opened
        if hasattr(self.gps_node, 'serial') and self.gps_node.serial:
            self.gps_node.serial.close()

    def test_parse_gpgga_valid_data(self):
        """
        Test parsing a valid GPGGA NMEA sentence.
        
        GPGGA (Global Positioning System Fix Data) is a NMEA sentence that contains:
        - Latitude and longitude
        - GPS quality indicator
        - Number of satellites
        - Altitude
        - And more...
        
        This test verifies that our parsing function correctly extracts
        the latitude, longitude, and their directions from a valid GPGGA sentence.
        """
        # ARRANGE: Set up the test data
        # This is a real GPGGA sentence from a GPS device
        gpgga_line = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
        
        # ACT: Call the function we want to test
        # We're testing the parse_gpgga method
        nmea_lat, lat_direction, nmea_lon, lon_direction = self.gps_node.parse_gpgga(gpgga_line)
        
        # ASSERT: Verify the results are correct
        # Check that we extracted the correct latitude
        self.assertEqual(nmea_lat, "4807.038", 
                        "Latitude should be extracted correctly from GPGGA sentence")
        
        # Check that we extracted the correct latitude direction
        self.assertEqual(lat_direction, "N", 
                        "Latitude direction should be 'N' for North")
        
        # Check that we extracted the correct longitude
        self.assertEqual(nmea_lon, "01131.000", 
                        "Longitude should be extracted correctly from GPGGA sentence")
        
        # Check that we extracted the correct longitude direction
        self.assertEqual(lon_direction, "E", 
                        "Longitude direction should be 'E' for East")

    def test_parse_gpgga_south_west_coordinates(self):
        """
        Test parsing GPGGA sentence with South-West coordinates.
        
        This test ensures our parser works with different coordinate directions.
        GPS coordinates can be:
        - North (N) or South (S) for latitude
        - East (E) or West (W) for longitude
        """
        # ARRANGE: Set up test data with South-West coordinates
        gpgga_line = "$GPGGA,123519,4807.038,S,01131.000,W,1,08,0.9,545.4,M,46.9,M,,*47"
        
        # ACT: Parse the GPGGA sentence
        nmea_lat, lat_direction, nmea_lon, lon_direction = self.gps_node.parse_gpgga(gpgga_line)
        
        # ASSERT: Verify South-West coordinates are parsed correctly
        self.assertEqual(lat_direction, "S", "Latitude direction should be 'S' for South")
        self.assertEqual(lon_direction, "W", "Longitude direction should be 'W' for West")

    def test_parse_gpgga_invalid_data(self):
        """
        Test parsing an invalid GPGGA sentence.
        
        This test verifies that our parser handles invalid data gracefully.
        In real-world scenarios, GPS devices might send malformed data,
        and our code should handle this without crashing.
        """
        # ARRANGE: Set up invalid test data (incomplete sentence)
        invalid_line = "$GPGGA,123519,4807.038,N,01131.000"  # Missing parts
        
        # ACT & ASSERT: Verify that invalid data raises an exception
        # We expect an IndexError because the sentence doesn't have enough parts
        with self.assertRaises(IndexError, 
                              msg="Invalid GPGGA sentence should raise IndexError"):
            self.gps_node.parse_gpgga(invalid_line)

    def test_parse_gpgga_empty_string(self):
        """
        Test parsing an empty GPGGA sentence.
        
        This test ensures our parser handles edge cases like empty strings.
        """
        # ARRANGE: Set up empty test data
        empty_line = ""
        
        # ACT & ASSERT: Verify that empty string raises an exception
        with self.assertRaises(IndexError, 
                              msg="Empty GPGGA sentence should raise IndexError"):
            self.gps_node.parse_gpgga(empty_line)

    def test_parse_gpvtg_valid_data(self):
        """
        Test parsing a valid GPVTG NMEA sentence.
        
        GPVTG (Track Made Good and Ground Speed) is a NMEA sentence that contains:
        - Course over ground (true and magnetic)
        - Ground speed (in knots and km/h)
        
        This test verifies that our parsing function correctly extracts
        the ground speed from a valid GPVTG sentence.
        """
        # ARRANGE: Set up test data with a valid GPVTG sentence
        gpvtg_line = "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48"
        
        # ACT: Parse the GPVTG sentence
        ground_speed = self.gps_node.parse_gpvtg(gpvtg_line)
        
        # ASSERT: Verify the ground speed is extracted correctly
        # The speed should be 10.2 km/h (from the 'K' field)
        self.assertEqual(ground_speed, 10.2, 
                        "Ground speed should be extracted correctly from GPVTG sentence")

    def test_parse_gpvtg_different_speeds(self):
        """
        Test parsing GPVTG sentences with different speed values.
        
        This test ensures our parser works with various speed values.
        """
        # ARRANGE: Set up test data with different speeds
        test_cases = [
            ("$GPVTG,054.7,T,034.4,M,005.5,N,000.0,K*48", 0.0),    # Zero speed
            ("$GPVTG,054.7,T,034.4,M,005.5,N,025.5,K*48", 25.5),   # Medium speed
            ("$GPVTG,054.7,T,034.4,M,005.5,N,100.0,K*48", 100.0),  # High speed
        ]
        
        # ACT & ASSERT: Test each case
        for gpvtg_line, expected_speed in test_cases:
            with self.subTest(gpvtg_line=gpvtg_line):
                # Parse the GPVTG sentence
                actual_speed = self.gps_node.parse_gpvtg(gpvtg_line)
                
                # Verify the speed is correct
                self.assertEqual(actual_speed, expected_speed,
                               f"Speed should be {expected_speed} for line: {gpvtg_line}")

    def test_parse_gpvtg_invalid_data(self):
        """
        Test parsing an invalid GPVTG sentence.
        
        This test verifies that our parser handles invalid GPVTG data gracefully.
        """
        # ARRANGE: Set up invalid test data
        invalid_line = "$GPVTG,054.7,T,034.4,M"  # Incomplete sentence
        
        # ACT & ASSERT: Verify that invalid data raises an exception
        with self.assertRaises(IndexError, 
                              msg="Invalid GPVTG sentence should raise IndexError"):
            self.gps_node.parse_gpvtg(invalid_line)

    def test_parse_gpvtg_non_numeric_speed(self):
        """
        Test parsing GPVTG sentence with non-numeric speed.
        
        This test ensures our parser handles malformed speed data.
        """
        # ARRANGE: Set up test data with non-numeric speed
        invalid_line = "$GPVTG,054.7,T,034.4,M,005.5,N,invalid,K*48"
        
        # ACT & ASSERT: Verify that non-numeric speed raises an exception
        with self.assertRaises(ValueError, 
                              msg="Non-numeric speed should raise ValueError"):
            self.gps_node.parse_gpvtg(invalid_line)


class TestGpsNodeCoordinateConversion(unittest.TestCase):
    """
    Test cases for coordinate conversion functionality.
    
    GPS coordinates come in NMEA format (degrees and minutes),
    but most applications need decimal format (decimal degrees).
    
    This test class focuses on testing the conversion between these formats.
    """

    def setUp(self):
        """
        Set up test fixtures for coordinate conversion tests.
        """
        # Mock ROS2 components
        with patch('rclpy.init'), \
             patch('rclpy.node.Node.__init__', return_value=None), \
             patch('rclpy.node.Node.declare_parameter'), \
             patch('rclpy.node.Node.get_parameter'), \
             patch('rclpy.node.Node.create_publisher'), \
             patch('rclpy.node.Node.create_timer'), \
             patch('rclpy.node.Node.get_logger'):
            
            self.gps_node = GpsNode()

    def test_nmea_to_decimal_north_east(self):
        """
        Test converting NMEA coordinates to decimal format for North-East location.
        
        NMEA format: DDMM.MMMM (Degrees Minutes.Minutes)
        Decimal format: DD.DDDD (Decimal Degrees)
        
        Example: 4807.038 N = 48 degrees, 7.038 minutes North
        Conversion: 48 + (7.038 / 60) = 48.1173 degrees
        """
        # ARRANGE: Set up NMEA coordinate data
        nmea_lat = "4807.038"    # 48 degrees, 7.038 minutes
        lat_direction = "N"      # North
        nmea_lon = "01131.000"   # 11 degrees, 31.000 minutes
        lon_direction = "E"      # East
        
        # ACT: Convert NMEA to decimal coordinates
        decimal_lat, decimal_lon = self.gps_node.nmea_to_decimal(
            nmea_lat, lat_direction, nmea_lon, lon_direction)
        
        # ASSERT: Verify the conversion is correct
        # Expected latitude: 48 + (7.038 / 60) = 48.1173
        expected_lat = 48 + (7.038 / 60.0)
        self.assertAlmostEqual(decimal_lat, expected_lat, places=4,
                             msg="Latitude conversion should be accurate to 4 decimal places")
        
        # Expected longitude: 11 + (31.000 / 60) = 11.5167
        expected_lon = 11 + (31.000 / 60.0)
        self.assertAlmostEqual(decimal_lon, expected_lon, places=4,
                             msg="Longitude conversion should be accurate to 4 decimal places")

    def test_nmea_to_decimal_south_west(self):
        """
        Test converting NMEA coordinates to decimal format for South-West location.
        
        South and West coordinates should be negative in decimal format.
        """
        # ARRANGE: Set up NMEA coordinate data for South-West
        nmea_lat = "4807.038"    # 48 degrees, 7.038 minutes
        lat_direction = "S"      # South (should be negative)
        nmea_lon = "01131.000"   # 11 degrees, 31.000 minutes
        lon_direction = "W"      # West (should be negative)
        
        # ACT: Convert NMEA to decimal coordinates
        decimal_lat, decimal_lon = self.gps_node.nmea_to_decimal(
            nmea_lat, lat_direction, nmea_lon, lon_direction)
        
        # ASSERT: Verify South-West coordinates are negative
        expected_lat = -(48 + (7.038 / 60.0))  # Negative for South
        self.assertAlmostEqual(decimal_lat, expected_lat, places=4,
                             msg="South latitude should be negative")
        
        expected_lon = -(11 + (31.000 / 60.0))  # Negative for West
        self.assertAlmostEqual(decimal_lon, expected_lon, places=4,
                             msg="West longitude should be negative")

    def test_nmea_to_decimal_edge_cases(self):
        """
        Test coordinate conversion with edge cases.
        
        Edge cases are unusual or extreme values that might cause problems.
        Testing these helps ensure our code is robust.
        """
        # ARRANGE: Set up edge case test data
        test_cases = [
            # Zero coordinates
            ("0000.000", "N", "00000.000", "E", 0.0, 0.0),
            # Maximum coordinates
            ("9000.000", "N", "18000.000", "E", 90.0, 180.0),
            # Minimum coordinates
            ("9000.000", "S", "18000.000", "W", -90.0, -180.0),
        ]
        
        # ACT & ASSERT: Test each edge case
        for nmea_lat, lat_dir, nmea_lon, lon_dir, expected_lat, expected_lon in test_cases:
            with self.subTest(nmea_lat=nmea_lat, nmea_lon=nmea_lon):
                # Convert coordinates
                actual_lat, actual_lon = self.gps_node.nmea_to_decimal(
                    nmea_lat, lat_dir, nmea_lon, lon_dir)
                
                # Verify results
                self.assertAlmostEqual(actual_lat, expected_lat, places=4,
                                     msg=f"Edge case latitude conversion failed for {nmea_lat}")
                self.assertAlmostEqual(actual_lon, expected_lon, places=4,
                                     msg=f"Edge case longitude conversion failed for {nmea_lon}")

    def test_nmea_to_decimal_invalid_format(self):
        """
        Test coordinate conversion with invalid NMEA format.
        
        This test ensures our conversion function handles invalid input gracefully.
        """
        # ARRANGE: Set up invalid test data
        invalid_cases = [
            ("invalid", "N", "01131.000", "E"),  # Invalid latitude
            ("4807.038", "N", "invalid", "E"),   # Invalid longitude
            ("", "N", "01131.000", "E"),         # Empty latitude
            ("4807.038", "N", "", "E"),          # Empty longitude
        ]
        
        # ACT & ASSERT: Test each invalid case
        for nmea_lat, lat_dir, nmea_lon, lon_dir in invalid_cases:
            with self.subTest(nmea_lat=nmea_lat, nmea_lon=nmea_lon):
                # Verify that invalid input raises an exception
                with self.assertRaises((ValueError, IndexError),
                                      msg=f"Invalid NMEA format should raise exception: {nmea_lat}, {nmea_lon}"):
                    self.gps_node.nmea_to_decimal(nmea_lat, lat_dir, nmea_lon, lon_dir)


# This is the main entry point for running the tests
if __name__ == '__main__':
    # Create a test suite and run all tests
    unittest.main() 