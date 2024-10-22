import serial
import struct
from .commands import Cmd
import time

class Roboclaw:
    def __init__(self, port, baudrate, tries=2):
        """
        Initializes the Roboclaw class with the specified serial port and baud rate.
        
        :param port: The serial port where the RoboClaw is connected (e.g., "/dev/roboclaw").
        :param baudrate: The baud rate for serial communication (e.g., 38400).
        """
        self.tries = tries  # Number of attempts to send a command
        self.port = port
        self.baudrate = baudrate
        self.ser = None  # Serial connection object

    def open(self):
        """
        Opens the serial port for communication with RoboClaw.

        :return: True if the port was opened successfully, False otherwise.
        """
        try:
            self.ser = serial.Serial(self.port, baudrate=self.baudrate, timeout=0.01)
            return True
        except:
            # print(f"Failed to open port {self.port}")
            return False

    def close(self):
        """
        Closes the serial port if it is open.
        """
        if self.ser and self.ser.is_open:
            self.ser.close()

    def forward_m1(self, address, value):
        """
        Drive motor 1 forward.

        :param address: The address of the RoboClaw controller.
        :param value: Speed value (0-127).
        :return: True if successful, False otherwise.
        """
        return self.send_drive_command(address, Cmd.M1_FORWARD, value)

    def backward_m1(self, address, value):
        """
        Drive motor 1 backward.

        :param address: The address of the RoboClaw controller.
        :param value: Speed value (0-127).
        :return: True if successful, False otherwise.
        """
        return self.send_drive_command(address, Cmd.M1_BACKWARD, value)

    def forward_m2(self, address, value):
        """
        Drive motor 2 forward.

        :param address: The address of the RoboClaw controller.
        :param value: Speed value (0-127).
        :return: True if successful, False otherwise.
        """
        return self.send_drive_command(address, Cmd.M2_FORWARD, value)

    def backward_m2(self, address, value):
        """
        Drive motor 2 backward.

        :param address: The address of the RoboClaw controller.
        :param value: Speed value (0-127).
        :return: True if successful, False otherwise.
        """
        return self.send_drive_command(address, Cmd.M2_BACKWARD, value)

    def read_version(self, address):
        """
        Reads the firmware version of the RoboClaw.

        :param address: The address of the RoboClaw controller.
        :return: A string representing the firmware version or None if an error occurred.
        """
        try:
            self.ser.flushInput()  # Clear the input buffer
            self.send_command(address, 21, [])  # Send the command to read the version
            version = self.ser.read(32).decode('utf-8')  # Read the response (up to 32 bytes)
            crc = self.read_crc()  # Read the CRC from the response
            if self._validate_crc(version.encode('utf-8'), crc):
                return version.strip()
            else:
                return None
        except Exception as e:
            # print(f"Error reading version: {e}")
            return None

    def read_main_battery_voltage(self, address):
        """
        Reads the main battery voltage.

        :param address: The address of the RoboClaw controller.
        :return: The main battery voltage in volts or None if an error occurred.
        """
        return self.read_2bytes_command(address, Cmd.READ_MAIN_BATTERY_VOLTAGE)

    def read_logic_battery_voltage(self, address):
        """
        Reads the logic battery voltage.

        :param address: The address of the RoboClaw controller.
        :return: The logic battery voltage in volts or None if an error occurred.
        """
        return self.read_2bytes_command(address, Cmd.READ_LOGIC_BATTERY_VOLTAGE)

    def read_motor_currents(self, address):
        """
        Reads the current consumption of both motors (M1 and M2).

        :param address: The address of the RoboClaw controller.
        :return: A tuple (M1 current, M2 current) in amps or None if an error occurred.
        """
        return self.read_4bytes_command(address, Cmd.READ_MOTOR_CURRENTS)

    def read_temperature(self, address):
        """
        Reads the temperature of the RoboClaw controller.

        :param address: The address of the RoboClaw controller.
        :return: The temperature in degrees Celsius or None if an error occurred.
        """
        return self.read_2bytes_command(address, Cmd.READ_TEMPERATURE)

    # def send_command(self, address, command, data):
    #     """
    #     Sends a command to the RoboClaw controller.

    #     :param address: The address of the RoboClaw controller.
    #     :param command: The command to be sent (byte value).
    #     :param data: A list of data bytes to be sent with the command.
    #     :return: True if the command was acknowledged, False otherwise.
    #     """
    #     try:
    #         checksum = address
    #         self.ser.write(bytes([address, command]))  # Send address and command
    #         checksum += command

    #         for byte in data:
    #             self.ser.write(bytes([byte]))  # Send each data byte
    #             checksum += byte

    #         self.ser.write(bytes([checksum & 0x7F]))  # Send the checksum (only the lower 7 bits)
    #         ack = self.ser.read(1)  # Read the acknowledgment byte
    #         return len(ack) == 1  # True if acknowledgment received
    #     except Exception as e:
    #         print(f"Error sending command: {e}")
    #         return False

    def send_read_command(self, address, command):
        """
        Sends a read command to the RoboClaw controller without checksum.

        :param address: The address of the RoboClaw controller.
        :param command: The command to be sent.
        :return: True if the command was sent successfully, False otherwise.
        """
        try:
            self.ser.write(bytes([address, command]))  # Send address and command
            return True
        except Exception as e:
            # print(f"Error sending read command: {e}")
            return False
        
    def send_drive_command(self, address, command, value):
        """
        Sends a drive command to the RoboClaw controller.
        This includes the command for driving M1 or M2 forwards or backwards.
        
        :param address: The address of the RoboClaw controller.
        :param command: The drive command (0 for M1 forward, 1 for M1 backward, 4 for M2 forward, 5 for M2 backward).
        :param value: The speed value (0-127).
        :return: True if the command was acknowledged (0xFF), False otherwise.
        """
        try:
            start_time = time.time()
            self.ser.flushInput()  # Clear the input buffer

            # Prepare the data to send
            data = bytes([address, command, value])
            crc = self._crc16(data)  # Use the existing _crc16 method
            crc_bytes = struct.pack('>H', crc)  # Convert CRC to 2-byte array
            
            # tries = self.tries
            # while tries:
            
            # Send the data + CRC
            self.ser.write(data + crc_bytes)
            
            # Read the acknowledgment
            ack = self.ser.read(1)
            with open('/ros2_ws/src/motor_control/motor_control/log.txt', 'a') as log_file:
                if ack == b'\xFF':
                    # elapsed_time = time.time() - start_time
                    # log_file.write(f"True, time: {elapsed_time}\n")
                    # print("Drive command acknowledged (0xFF).")
                    # log_file.write(f"OK\n")
                    return True
                else:
                    # tries -= 1
                    elapsed_time = time.time() - start_time
                    log_file.write(f"Received byte: {ack}; left tries: {tries}. Time: {elapsed_time}\n")
                    # logger.info(f"Unexpected response: {ack}. Expected 0xFF. Remaining tries: {tries}")
                    return False

            # with open('/ros2_ws/src/motor_control/motor_control/log.txt', 'a') as log_file:
            #     elapsed_time = time.time() - start_time
            #     log_file.write(f"Failed after multiple attempts, time: {elapsed_time}\n")        
            # # print("Failed to send drive command after multiple attempts.")
            # return False
        
        except Exception as e:
            # print(f"Error sending drive command: {e}")
            return False
        
    # def read_2bytes_command(self, address, command):
    #     """
    #     Reads a 2-byte response from a command.

    #     :param address: The address of the RoboClaw controller.
    #     :param command: The command to be sent.
    #     :return: The interpreted value from the 2-byte response or None if an error occurred.
    #     """
    #     try:
    #         print(f"Sending command: {command} to address: {address}")  # Debugging
    #         self.send_command(address, command, [])
    #         data = self.ser.read(2)  # Read 2 bytes of data
    #         print(f"Raw data received: {data}")  # Debugging
    #         crc = self.read_crc()  # Read the CRC
    #         print(f"CRC received: {crc}")  # Debugging

    #         if len(data) == 2 and self._validate_crc(data, crc):
    #             voltage = struct.unpack('>H', data)[0] / 10.0  # Convert to a value in volts
    #             print(f"Voltage read: {voltage} V")  # Debugging
    #             return voltage
    #         else:
    #             print("CRC validation failed or data length incorrect.")  # Debugging
    #             return None
    #     except Exception as e:
    #         print(f"Error reading 2-byte command: {e}")
    #         return None


    def read_2bytes_command(self, address, command):
        """
        Sends read command and reads a 2-byte response and validates the response using CRC.

        :param address: The address of the RoboClaw controller.
        :param command: The command to be sent.
        :return: The interpreted value from the 2-byte response or None if an error occurred.
        """
        try:
            # tries = self.tries
            # while tries:
            self.ser.flushInput()  # Clear the input buffer
            self.send_read_command(address, command)  # Send the address and command

            data = self.ser.read(2)  # Read 2 bytes of data
            crc_received = self.read_crc()  # Read the CRC
            crc_input = bytes([address, command]) + data  # Combine address, command, and data for CRC validation

            if len(data) == 2 and self._validate_crc(crc_input, crc_received):
                return struct.unpack('>H', data)[0] # Unpack as an unsigned short
            else:
                # tries -= 1
                # print(f"CRC validation failed or data length incorrect: {len(data)}. Expected: 2. Remaining tries: {tries}")  # Debugging
                return None

            # print("Failed to read 2-byte command after multiple attempts.")
            # return None
        
        except Exception as e:
            # print(f"Error reading 2-byte command: {e}")
            return None

    def read_4bytes_command(self, address, command):
        """
        Sends read command and reads a 4-byte response and validates the response using CRC.

        :param address: The address of the RoboClaw controller.
        :param command: The command to be sent.
        :return: A tuple of interpreted values from the 4-byte response or None if an error occurred.
        """

        try:
            # tries = self.tries
            # while tries:
            self.ser.flushInput()  # Clear the input buffer
            self.send_read_command(address, command)  # Send the address and command

            data = self.ser.read(4)  # Read 4 bytes of data
            crc_received = self.read_crc()  # Read the CRC
            crc_input = bytes([address, command]) + data  # Combine address, command, and data for CRC validation

            if len(data) == 4 and self._validate_crc(crc_input, crc_received):
                return struct.unpack('>hh', data) # Unpack as two signed shorts
            else:
                # tries -= 1
                # print(f"CRC validation failed or data length incorrect: {len(data)}. Expected: 4. Remaining tries: {tries}")  # Debugging
                return None

            # print("Failed to read 4-byte command after multiple attempts.")
            # return None

        except Exception as e:
            # print(f"Error reading 4-byte command: {e}")
            return None

    def read_crc(self):
        """
        Reads a 2-byte CRC value from the serial port.

        :return: The CRC value as an unsigned short or None if an error occurred.
        """
        try:
            crc_bytes = self.ser.read(2)  # Read 2 bytes for CRC
            if len(crc_bytes) == 2:
                crc = struct.unpack('>H', crc_bytes)[0]  # Unpack as an unsigned short
                return crc
            else:
                # print("Failed to read 2-byte CRC.")  # Debugging
                return None
        except Exception as e:
            # print(f"Error reading CRC: {e}")  # Debugging
            return None

    def _validate_crc(self, data, crc):
        """
        Validates the CRC16 checksum against received data.

        :param data: The data bytes that were received.
        :param crc: The CRC value received.
        :return: True if the CRC is valid, False otherwise.
        """
        calculated_crc = self._crc16(data)  # Calculate CRC for the data
        return calculated_crc == crc

    def _crc16(self, data):
        """
        Calculates the CRC16 checksum for the given data.

        :param data: The data bytes to calculate the checksum for.
        :return: The calculated CRC16 checksum as an unsigned short.
        """
        crc = 0
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if (crc & 0x8000) != 0:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
        return crc & 0xFFFF

# Usage example:
# if __name__ == "__main__":
    # roboclaw = Roboclaw("/dev/roboclaw", 38400)
    # if roboclaw.open():
    #     address = 0x80

    #     # Read and print RoboClaw firmware version
    #     version = roboclaw.read_version(address)
    #     if version:
    #         # print(f"RoboClaw version: {version}")
