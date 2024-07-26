from zaber_motion import Library, DeviceDbSourceType, Units, MotionLibException
from zaber_motion.ascii import Connection
from zaber_motion.exceptions.connection_failed_exception import ConnectionFailedException

import serial
import os
from pathlib import Path

from ..helper.Logger import Logger
from ..helper.utils import get_proper_port_for_device, SupportedDevices

class ZaberRail():
    def __init__(self, port):
        Library.enable_device_db_store("./device-db-store")
        self.device = None
        self.axis = None
        self.connection = None
        self.logger = Logger("Zaber-X-LRT1500BL-E08C", Path(os.getcwd())/"logs", "zaber.log")
        self.port = port

    def __del__(self):
        self.disconnect()
    
    def disconnect(self):
        if self.connection is not None:
            self.connection.close()
            self.connection = None

    def connect(self):
        try:
            self.connection = Connection.open_serial_port(self.port)
            self.connection.enable_alerts()
        except ConnectionFailedException as e:
            print(f"Zaber Rail Connectioned Failed: {e}")
            self.logger.error(f"Zaber Rail Connectioned Failed: {e}")
            return False
        device_list = self.connection.detect_devices()
        print(f"Found {len(device_list)} devices")
        self.device = device_list[0]
        self.axis = self.device.get_axis(1)
        print(f"axis obtained: {self.axis}")
        if not self.axis.is_homed():
            self.axis.home()
        return True
    
    def print_io_info(self):
        if self.device is None:
            ok = self.connect()
            if not ok:
                print("zaber rail cannot be connected!")
                exit()
        io_info = self.device.io.get_channels_info()
        print("Number of analog outputs:", io_info.number_analog_outputs)
        print("Number of analog inputs:", io_info.number_analog_inputs)
        print("Number of digital outputs:", io_info.number_digital_outputs)
        print("Number of digital inputs:", io_info.number_digital_inputs)

    def basic_move(self):
        """for debugging rail movement only, don't use this function in production"""
        try:
            # Move to 50cm
            self.axis.move_absolute(10, Units.LENGTH_CENTIMETRES)
            # Move by an additional cm
            self.axis.move_relative(5, Units.LENGTH_CENTIMETRES)
            # Move back to 0
            self.axis.move_absolute(5, Units.LENGTH_CENTIMETRES)

        except MotionLibException as e:
            print(f"Failed to move Zaber rail with error: {e}")

    def move_home(self):
        self.axis.move_absolute(0, Units.LENGTH_CENTIMETRES)

    def move(self, pos_abs): 
        if self.axis is None:
            ok = self.connect()
            if not ok:
                print("zaber rail cannot be connected!")
                exit()
        pos_cur = self.axis.get_position(Units.LENGTH_CENTIMETRES)
        self.logger.debug(f"[REL_MOVE] current pos: {pos_cur} cm, moving to the absolute position: {pos_abs}")
        self.axis.move_absolute(pos_abs, Units.LENGTH_CENTIMETRES)

    def rel_move(self, pos_rel):
        pos_cur = self.axis.get_position(Units.LENGTH_CENTIMETRES)
        self.logger.debug(f"[REL_MOVE] current pos: {pos_cur} cm, relatively moving {pos_rel}")
        self.axis.move_relative(pos_rel)


# below are for testing the zaber rail standalone.

def drive_rail(rail):
    while True:
        pos = input("Type in Abs position, Press [Enter] to quit: ")
        if pos == '':
            break
        try:
            pos = float(pos)
            rail.move(pos)
        except ValueError:
            print("Invalid input. Please enter a valid position value.")

def zaber_cli_app():
    selected_port = get_proper_port_for_device(SupportedDevices.ZaberLinearRail)
    zaber_rail = ZaberRail(port=selected_port)
    ok = zaber_rail.connect()
    if not ok:
        print("zaber rail cannot be connected!")
        exit()
    try:
        while True:
            input_str = input("Press [Enter] to quit, [0] to home the rail, [M] to drive rail: ").strip().lower()
            if input_str == '':
                break
            elif input_str == '0':
                zaber_rail.axis.home()
                break
            elif input_str == 'm':
                drive_rail(zaber_rail)
            else:
                print("Invalid input. Please enter a valid option.")
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        zaber_rail.disconnect()
        print("Zaber Rail disconnected safely.")