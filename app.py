# TODO: This will be a Qt-based GUI app
# This is not part of the BatteryLab package when using pip install

from BatteryLab.robots.ZaberRail import zaber_cli_app
from BatteryLab.robots.SuctionPump import suction_cli_app
from BatteryLab.robots.RailMeca500 import rail_meca500_example_app
from BatteryLab.robots.BreadboardMeca500 import breadboard_meca500_example_app
from BatteryLab.robots.Meca500 import test_config_file_location
from BatteryLab.camera.camera_utility import send_image, receive_image

def main():
    help_str = """Choose which app to use in the Battery Lab CLI program.
Options: 1. suction, 2. zaber_rail, 3. rail_meca500 4. breadboard_meca500
5. camera_master, 6. camera_slave, 7. file_location_test
"""
    print(help_str)
    mode = input("Please select which app to use: ")

    if mode == 'suction' or mode == '1':
        suction_cli_app()
    elif mode == 'zaber' or mode == '2':
        zaber_cli_app()
    elif mode == 'meca500' or mode == '3':
        rail_meca500_example_app()
    elif mode == 'breadboard_mec500' or mode =='4':
        breadboard_meca500_example_app()
    elif mode == 'camera_master' or mode =='5':
        receive_image(port=9999)
    elif mode == 'camera_slave' or mode == '6':
        ip_address = input("type in the master computer's ip address:")
        send_image(ip=ip_address, port=9999)
    elif mode == 'file_location_test' or mode == '7':
        test_config_file_location()
    else:
        print("Your options are not valid! Program will exit!")

if __name__ == '__main__':
    main()