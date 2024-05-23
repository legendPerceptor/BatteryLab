import zaber_motion

from zaber_motion import Library, DeviceDbSourceType, Units
from zaber_motion.ascii import Connection

class ZaberRail():
    def __init__(self):
        pass

    def connect(port="COM5"):
        with Connection.open_serial_port("COM5") as connection:
            device_list = connection.detect_devices()
            print(f"Found {len(device_list)} devices")
            device = device_list[0]
            axis = device.get_axis(1)
            axis.home()
            axis.wait_until_idle()
            print("Axis Homed")
            axis.move_absolute(150, Units.LENGTH_MILLIMETRES)
            axis.wait_until_idle()