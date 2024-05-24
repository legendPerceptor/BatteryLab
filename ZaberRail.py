from zaber_motion import Library, DeviceDbSourceType, Units, MotionLibException
from zaber_motion.ascii import Connection

from Logger import Logger
import argparse

class ZaberRail():
    def __init__(self):
        Library.enable_device_db_store("./device-db-store")
        self.device = None
        self.axis = None
        self.connection = None
        self.logger = Logger("Zaber-X-LRT1500BL-E08C", "./zaber.log")

    def __del__(self):
        self.disconnect()
    
    def disconnect(self):
        if self.connection is not None:
            self.connection.close()
            self.connection = None

    def connect(self, port="/dev/tty.usbserial-A10NH07T"):
        self.connection = Connection.open_serial_port(port)
        self.connection.enable_alerts()

        device_list = self.connection.detect_devices()
        print(f"Found {len(device_list)} devices")
        self.device = device_list[0]
        self.axis = self.device.get_axis(1)
        if not self.axis.is_homed():
            self.axis.home()
    
    def print_io_info(self):
        if self.device is None:
            self.connect()
        io_info = self.device.io.get_channels_info()
        print("Number of analog outputs:", io_info.number_analog_outputs)
        print("Number of analog inputs:", io_info.number_analog_inputs)
        print("Number of digital outputs:", io_info.number_digital_outputs)
        print("Number of digital inputs:", io_info.number_digital_inputs)

    def basic_move(self):
        try:
            # Move to 50cm
            self.axis.move_absolute(10, Units.LENGTH_CENTIMETRES)
            # Move by an additional cm
            self.axis.move_relative(5, Units.LENGTH_CENTIMETRES)
            # Move back to 0
            self.axis.move_absolute(5, Units.LENGTH_CENTIMETRES)

        except MotionLibException as e:
            print(f"Failed to move Zaber rail with error: {e}")

    def move(self, pos_abs): 
        if self.axis is None:
            self.connect()
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

def main():

    parser = argparse.ArgumentParser(
        description="zaber rail testing program",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument(
        "-p",
        "--port",
        type=str,
        default="/dev/tty.usbserial-A10NH07T",
        help="The serial port to connect to the Zaber rail"
    )

    args = parser.parse_args()
    port = args.port

    zaber_rail = ZaberRail()
    zaber_rail.connect(port)
    zaber_rail.basic_move()
    try:
        while True:
            input_str = input("Press [Enter] to reset the rail, [0] to home the rail, [M] to drive rail: ").strip().lower()
            if input_str == '':
                zaber_rail.axis.home()
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

if __name__ == "__main__":
    main()