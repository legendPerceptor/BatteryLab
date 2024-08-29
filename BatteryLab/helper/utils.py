import os
import platform
import pyudev # Linux only
from pathlib import Path
from serial.tools import list_ports
from enum import Enum
import numpy as np
from typing import List
from ..robots.Constants import AssemblyRobotConstants, ComponentProperty
from matplotlib import pyplot as plt

class SupportedDevices(Enum):
    ZaberLinearRail = 1
    SuctionPump = 2
    SartoriusRLine = 3

DeviceToSerialDict = {
    SupportedDevices.ZaberLinearRail : "FTDI_FT232R_USB_UART_A10NH07T",
    SupportedDevices.SuctionPump: "1a86_USB_Serial",
    SupportedDevices.SartoriusRLine: "TODO"
}

def get_proper_port_for_device(device_name: SupportedDevices):
    usb_ports = list_ports.comports()
    print("please select the correct port by typing the index number:")
    port_index = -1
    # list available devices
    for i, port in enumerate(usb_ports):
        print(f'{i}> name: {port.name}, device: {port.device}')
    
    selected_port = ""
    if platform.system() == 'Linux':
        context = pyudev.Context()
        tty_devices = [device for device in context.list_devices(subsystem='tty') if 'ttyUSB' in device.device_node]
        for tty in tty_devices:
            if tty.get('ID_SERIAL') == DeviceToSerialDict[device_name]:
                selected_port = tty.device_node
    elif platform.system() == 'Darwin': # MacBook
        for port in usb_ports:
            if 'usbserial' in port.device:
                if device_name == SupportedDevices.ZaberLinearRail and port.serial_number == 'A10NH07T':
                    selected_port = port.device
                if device_name == SupportedDevices.SuctionPump and port.serial_number != 'A10NH07T':
                    selected_port = port.device

    while True:
        if platform.system() == 'Linux' and selected_port != "":
            break 
        port_index_str = input(f"[default is {selected_port}]: ").strip().lower()
        flag = True
        if port_index_str != '':
            try:
                port_index = int(port_index_str)
            except ValueError as e:
                print("Please provide a proper serial port to proceed!")
                flag = False
        if flag:
            break
    print(f"selected port index: {port_index}")
    if port_index != -1:
        selected_port = usb_ports[port_index].device
    print("selected port: ", selected_port)
    return selected_port

def draw_calculated_points(pos_dict):
    x_coords = []
    y_coords = []
    manual_x_coords = []
    manual_y_coords = []
    offset = 0.04
    plt.figure(figsize=(8, 6))
    for i in range(64):
        x = int(i // 8)
        y = int(i % 8)
        if (x == 0 and y == 0) or (x == 7 and y == 0) or (x == 0 and y == 7) or (x == 7 and y == 7):
            # Add index labels near each point
            plt.text(pos_dict[i][0] + offset, pos_dict[i][1] + offset, str(i), fontsize=8, color='red')
            manual_x_coords.append(pos_dict[i][0])
            manual_y_coords.append(pos_dict[i][1])
        else:
            plt.text(pos_dict[i][0] + offset, pos_dict[i][1] + offset, str(i), fontsize=8, color='blue')
            x_coords.append(pos_dict[i][0])
            y_coords.append(pos_dict[i][1])
    plt.scatter(x_coords, y_coords, color='blue', s=14)  # Plot the points
    plt.scatter(manual_x_coords, manual_y_coords, color='red', s=14)
    for i in range(4):
        if i == 0 or i == 2:
            points_x = [manual_x_coords[i], manual_x_coords[(i+1) % 4]]
            points_y = [manual_y_coords[i], manual_y_coords[(i+1) % 4]]
            plt.plot(points_x, points_y, color='r', linewidth=1)
        if i == 0 or i == 1:
            points_x = [manual_x_coords[i], manual_x_coords[(i+2) % 4]]
            points_y = [manual_y_coords[i], manual_y_coords[(i+2) % 4]]
            plt.plot(points_x, points_y, color='r', linewidth=1)

    # Add labels and title
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Scatter Plot of Points')
    # Display the grid and axes
    plt.grid(True)
    project_dir = "/home/yuanjian/Research/BatteryLab/logs/"
    plt.savefig(project_dir + 'generated_points.png')

def get_8_8_well_pos(bottom_left_coordinates, bottom_right_coordinates, top_left_coordinates, top_right_coordinates):
    avg_height = np.average([bottom_left_coordinates[2], bottom_right_coordinates[2], top_left_coordinates[2], top_right_coordinates[2]])
    pos_dict: dict[int, List[float]] = {}
    delta_y = np.average([bottom_right_coordinates[1] - bottom_left_coordinates[1], top_right_coordinates[1] - top_left_coordinates[1]]) / 7
    delta_x = np.average([bottom_right_coordinates[0] - top_right_coordinates[0], bottom_left_coordinates[0] - top_left_coordinates[0]]) / 7
    for i in range(64):
        x = int(i // 8)
        y = int(i % 8)
        if x == 0 and y == 0:
            pos_dict[i] = top_left_coordinates
        elif x == 7 and y == 0:
            pos_dict[i] = bottom_left_coordinates
        elif x == 0 and y == 7:
            pos_dict[i] = top_right_coordinates
        elif x == 7 and y == 7:
            pos_dict[i] = bottom_right_coordinates
        else:
            pos_dict[i] = [float(top_left_coordinates[0] + x * delta_x), float(top_left_coordinates[1] + y * delta_y), float(avg_height)] + bottom_left_coordinates[3:]
    draw_calculated_points(pos_dict)
    return pos_dict

def create_robot_constants_from_manual_positions(manual_positions) -> AssemblyRobotConstants:
    """
    Obtain the assembly robot constants from manually aligned positions.

    Parameters:
      manual_positions: the YAML file content parsed by yaml.safe_load()

    Returns:
      AssemblyRobotConstants: generate all the well positions and 

    There are 7 fields for different components, namely Cathode Case, Cathode, Separator, Anode, Washer, Spacer, Anode Case.
    Each component tray will have 4 manually posed positions at 4 corners (x, y, z, alpha, beta, gama). There are 64 wells on each tray.
    The location for each well will be bilinearly interpolated by the 4 positions.

    An example YAML file looks like the following.

AessemblePost:
  rail_pos: 110
  cartesian: [-193.99646, -152.00122, 126.08627, 163.49303, -50.50488, -21.00815]
  joints: [-135.12872, 42.12607, 7.25795, -105.43295, 25.59059, 127.64982]
Separator:
  rail_pos: 14.5
  bottom_left:
    cartesian: [235.97647, -87.38851, 40.11804, 173.25027, 42.36252, -166.87105]
    joints: [-23.29881, 67.05935, 0.30938, -30.22345, -21.77718, 19.73194]
  bottom_right:
    cartesian: [235.16858, 72.84175, 39.81935, 173.25028, 42.36252, -166.87105]
    joints: [22.77213, 66.685, 1.47282, 51.36269, -25.87523, -22.1584]
  top_left:
    cartesian: [75.24227, -87.65825, 41.29566, 173.25028, 42.36252, -166.87105]
    joints: [-71.00944, 52.73619, 64.5154, -45.23412, -59.04767, -25.42198]
  top_right:
    cartesian: [74.08437, 72.84175, 41.03251, 173.25028, 42.36252, -166.87105]
    joints: [71.16736, 53.23022, 65.68386, 54.59531, -54.76851, 38.62172]
Cathode:
    ...
    """
    constants = AssemblyRobotConstants()

    cartesian_coord_prop = "cartesian"
    rail_pos_prop = "rail_pos"
    bottom_left_prop = "bottom_left"
    bottom_right_prop = "bottom_right"
    top_left_prop = "top_left"
    top_right_prop = "top_right"

    assemble_post = manual_positions["AssemblePost"]
    constants.POST_C_SK_PO = assemble_post[cartesian_coord_prop]
    constants.POST_RAIL_LOCATION = assemble_post[rail_pos_prop]

    components = ["CathodeCase", "Cathode", "Separator", "Anode", "Washer", "Spacer", "AnodeCase"]

    component_name = components[2]
    component = manual_positions[component_name]
    component_property = ComponentProperty()
    component_property.railPo = component[rail_pos_prop]
    component_property.dropPo = assemble_post[cartesian_coord_prop]
    bottom_left_coordinates = component[bottom_left_prop][cartesian_coord_prop]
    bottom_right_coordinates = component[bottom_right_prop][cartesian_coord_prop]
    top_left_coordinates = component[top_left_prop][cartesian_coord_prop]
    top_right_coordinates = component[top_right_prop][cartesian_coord_prop]
    component_property.grabPo = get_8_8_well_pos(bottom_left_coordinates, bottom_right_coordinates, top_left_coordinates, top_right_coordinates)
    setattr(constants, component_name, component_property)

    return constants
