import os
import platform
import pyudev # Linux only
from pathlib import Path
from serial.tools import list_ports
from enum import Enum
import numpy as np
from typing import List
from ..robots.Constants import AssemblyRobotConstants, ComponentProperty, AssemblyRobotCameraConstants
from matplotlib import pyplot as plt

class SupportedDevices(Enum):
    ZaberLinearRail = 1
    SuctionPump = 2
    SartoriusRLine = 3

DeviceToSerialDict = {
    SupportedDevices.ZaberLinearRail : "FTDI_FT232R_USB_UART_A10NH07T",
    SupportedDevices.SuctionPump: "1a86_USB_Serial",
    SupportedDevices.SartoriusRLine: "FTDI_USB-RS232_Cable_FT4WM2HM"
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

def draw_calculated_points(pos_list, m, n, file_name:str = "generated_points"):
    x_coords = []
    y_coords = []
    manual_x_coords = []
    manual_y_coords = []
    offset = 0.04
    plt.figure(figsize=(n, m))
    for i in range(m * n):
        x = int(i // n)
        y = int(i % n)
        if (x == 0 and y == 0) or (x == m-1 and y == 0) or (x == 0 and y == n-1) or (x == m-1 and y == n-1):
            # Add index labels near each point
            plt.text(pos_list[i][0] + offset, pos_list[i][1] + offset, str(i), fontsize=8, color='red')
            manual_x_coords.append(pos_list[i][0])
            manual_y_coords.append(pos_list[i][1])
        else:
            plt.text(pos_list[i][0] + offset, pos_list[i][1] + offset, str(i), fontsize=8, color='blue')
            x_coords.append(pos_list[i][0])
            y_coords.append(pos_list[i][1])
    plt.scatter(y_coords, x_coords, color='blue', s=14)  # Plot the points
    plt.scatter(manual_y_coords,manual_x_coords, color='red', s=14)
    for i in range(4):
        if i == 0 or i == 2:
            points_x = [manual_x_coords[i], manual_x_coords[(i+1) % 4]]
            points_y = [manual_y_coords[i], manual_y_coords[(i+1) % 4]]
            plt.plot(points_y,points_x, color='r', linewidth=1)
        if i == 0 or i == 1:
            points_x = [manual_x_coords[i], manual_x_coords[(i+2) % 4]]
            points_y = [manual_y_coords[i], manual_y_coords[(i+2) % 4]]
            plt.plot(points_y,points_x, color='r', linewidth=1)

    # Add labels and title
    plt.xlabel('Y-axis')
    plt.ylabel('X-axis')
    plt.title(file_name)
    # Display the grid and axes
    plt.grid(True)
    project_dir = Path(__file__).parent.parent.parent / "images"
    os.makedirs(str(project_dir), exist_ok=True)
    plt.savefig(str(project_dir / (file_name + ".png")))

def get_m_n_well_pos(bottom_left_coordinates, bottom_right_coordinates, top_left_coordinates, top_right_coordinates, m, n, name:str="default"):
    avg_height = np.average([bottom_left_coordinates[2], bottom_right_coordinates[2], top_left_coordinates[2], top_right_coordinates[2]])
    # pos_dict: dict[int, List[float]] = {}
    # delta_y = np.average([bottom_right_coordinates[1] - bottom_left_coordinates[1], top_right_coordinates[1] - top_left_coordinates[1]]) / (m-1)
    # delta_x = np.average([bottom_right_coordinates[0] - top_right_coordinates[0], bottom_left_coordinates[0] - top_left_coordinates[0]]) / (n-1)
    pos_list: List[List[float]] = []
    top_edge_x = np.linspace(top_left_coordinates[0], top_right_coordinates[0], n)
    top_edge_y = np.linspace(top_left_coordinates[1], top_right_coordinates[1], n)
    bottom_edge_x = np.linspace(bottom_left_coordinates[0], bottom_right_coordinates[0], n)
    bottom_edge_y = np.linspace(bottom_left_coordinates[1], bottom_right_coordinates[1], n)
    
    grid_x = np.zeros((m, n))
    grid_y = np.zeros((m, n))
    
    for i in range(n):
        grid_x[:, i] = np.linspace(top_edge_x[i], bottom_edge_x[i], m)
        grid_y[:, i] = np.linspace(top_edge_y[i], bottom_edge_y[i], m)

    for i in range(n * m):
        x = int(i // n)
        y = int(i % n)
        if x == 0 and y == 0:
            pos_list.append(top_left_coordinates)
        elif x == m-1 and y == 0:
            pos_list.append(bottom_left_coordinates)
        elif x == 0 and y == n-1:
            pos_list.append(top_right_coordinates)
        elif x == m-1 and y == n-1:
            pos_list.append(bottom_right_coordinates)
        else:
            # pos_dict[i] = [float(top_left_coordinates[0] + x * delta_x), float(top_left_coordinates[1] + y * delta_y), float(avg_height)] + bottom_left_coordinates[3:]
            pos_list.append([grid_x[x, y], grid_y[x, y], avg_height] + bottom_left_coordinates[3:])
    draw_calculated_points(pos_list, m, n, name)
    return pos_list

def create_assembly_robot_camera_constants_from_manual_positions(camera_manual_positions) -> AssemblyRobotCameraConstants:
    """
    Obtain the assembly robot camera constants from manually aligned positions.

    Parameters:
        camera_manual_positions: the YAML file content parsed by yaml.safe_load()
    
    Returns:
        AssemblyRobotCameraConstants: record all the camera positions and robot poses
    
    An example YAML file will look like this:

TRF: [0, 55, 70, 0, 30, 0]
Home: [0, 0, 0, 0, 60 ,0]
RobotPose:
  cartesian: [177.40324, 55, 208.15256, 180, 0, 180]
  joints: [0, -1.50672, -9.1021, 0, 70.60881, 0]
CathodeCase:
  rail_pos: 0.0
Cathode:
  rail_pos: 19.0
Separator:
  rail_pos: 42.0
Anode:
  rail_pos: 65.0
Washer:
  rail_pos: 91.0
Spacer:
  rail_pos: 114.0
AnodeCase:
  rail_pos: 137.0
    """
    constants = AssemblyRobotCameraConstants()
    constants.HOME_J = camera_manual_positions["Home"]
    constants.TRF = camera_manual_positions["TRF"]
    components = ["CathodeCase", "Cathode", "Separator", "Anode", "Washer", "Spacer", "AnodeCase"]
    constants.RobotPose = camera_manual_positions["RobotPose"]["cartesian"]
    for component in components:
        setattr(constants, component, camera_manual_positions[component]["rail_pos"])
    return constants

def create_assembly_robot_constants_from_manual_positions(manual_positions) -> AssemblyRobotConstants:
    """
    Obtain the assembly robot constants from manually aligned positions.

    Parameters:
        manual_positions: the YAML file content parsed by yaml.safe_load()

    Returns:
        AssemblyRobotConstants: generate all the well positions and record other location constants.

    There are 7 fields for different components, namely Cathode Case, Cathode, Separator, Anode, Washer, Spacer, Anode Case.
    Each component tray will have 4 manually posed positions at 4 corners (x, y, z, alpha, beta, gama). There are 64 wells on each tray.
    The location for each well will be bilinearly interpolated by the 4 positions.

    An example YAML file looks like the following.

TRF: [43, 12.0, 75.734, 0, 45, 0]
Home: [0, 0, 0, 0, 45, 0] 
AssemblePost:
  rail_pos: 110.0
  cartesian: [-224.18615, -168.52829, 49.29558, 179.99999, -0.00001, -22.02103]
  joints: [-141.10427, 43.06009, -1.64403, -68.06679, 12.7839, 79.67016]
LookupCamera:
  rail_pos: 105.0
  cartesian: [-222.98894, -102.23394, 122.5639, 179.99999, 0, -22.02105]
  joints: [-158.25595, 20.22285, -0.07483, 0.46605, 24.85303, -0.61875]
Spacer:
  bottom_right_box:
    rail_pos: 0.0
    top_left:
      cartesian: [187.94318, -128.6655, -43.60079, -180, 0, -179.99999]
      joints: [-50.65956, 61.76791, 15.80108, -62.43019, -38.09191, 15.65597]
    top_right:
      cartesian: [187.94313, -59.88992, -43.87432, -179.99999, -0.00001, -180]
      joints: [-31.94384, 54.79421, 41.05701, -29.21546, -50.04, -4.03524]
    bottom_left:
      cartesian: [256.62244, -128.66552, -43.83429, -180, 0, -179.99999]
      joints: [-37.40065, 76.78961, -21.94534, -83.08163, -25.63496, 53.93757]
    bottom_right:
      cartesian: [256.62243, -59.75853, -43.85109, 180, 0, -180]
      joints: [-21.30765, 65.50667, 5.70367, -32.98368, -28.16281, 14.35802]
  bottom_left_box:
    rail_pos: 8.0
    top_left:
      cartesian: [188.2683, -140.14922, -43.21727, -179.99999, 0.00001, -179.99997]
      joints: [-52.76732, 63.7041, 10.04234, -69.29185, -37.00412, 21.73102]
    top_right:
      cartesian: [187.76203, -71.62817, -43.95334, -179.99999, 0.00001, -179.99996]
      joints: [-35.99673, 55.48907, 37.80336, -34.19027, -47.6945, -2.61597]
    bottom_left:
      cartesian: [256.66829, -140.14921, -44.12214, -179.99998, 0.00001, -179.99997]
      joints: [-39.58355, 80.32561, -29.86375, -93.79165, -26.84373, 63.93629]
    bottom_right:
      cartesian: [256.66829, -71.53552, -44.08656, -179.99998, 0.00001, -179.99997]
      joints: [-24.41507, 66.81093, 2.46611, -39.70348, -27.22797, 18.6433]
  top_left_box:
    rail_pos: 0.0
    top_left:
      cartesian: [96.17218, -220.46755, -43.40975, 180, 0.00001, -179.99997]
      joints: [-84.22072, 77.57258, -23.98963, -117.25954, -52.31706, 48.2725]
    top_right:
      cartesian: [96.17218, -151.32057, -43.63123, -180, 0.00001, -179.99997]
      joints: [-81.80227, 58.56065, 25.69223, -87.58548, -44.46742, 8.13413]
    bottom_left:
      cartesian: [164.74315, -220.46754, -43.689, -180, 0, -179.99997]
      joints: [-68.38746, 87.63256, -46.25383, -118.59964, -48.48243, 68.69939]
    bottom_right:
      cartesian: [164.7535, -151.32057, -43.85983, -180, 0.00002, -179.99996]
      joints: [-60.57786, 63.09773, 12.2525, -75.67902, -39.46895, 20.27669]
  top_right_box:
    rail_pos: -1
    top_left:
      cartesian: []
      joints: []
    top_right:
      cartesian: []
      joints: []
    bottom_left:
      cartesian: []
      joints: []
    bottom_right:
      cartesian: []
      joints: []
    ...
    """
    constants = AssemblyRobotConstants()

    constants.HOME_J = manual_positions["Home"]
    constants.TRF = manual_positions["TRF"]

    cartesian_coord_prop = "cartesian"
    rail_pos_prop = "rail_pos"
    bottom_left_prop = "bottom_left"
    bottom_right_prop = "bottom_right"
    top_left_prop = "top_left"
    top_right_prop = "top_right"

    assemble_post = manual_positions["AssemblePost"]
    constants.POST_C_SK_PO = assemble_post[cartesian_coord_prop]
    constants.POST_RAIL_LOCATION = assemble_post[rail_pos_prop]

    lookup_camera = manual_positions["LookupCamera"]
    constants.LOOKUP_CAM_SK_PO = lookup_camera[cartesian_coord_prop]
    constants.LOOKUP_CAM_RAIL_LOCATION = lookup_camera[rail_pos_prop]

    components = ["CathodeCase", "Cathode", "Separator", "Anode", "Washer", "Spacer", "AnodeCase"]

    for component_name in components:
        print(f"Dealing with component <{component_name}>")
        if component_name not in manual_positions:
            print(f"The component <{component_name}> is not in the well positions YAML file")
            continue
        component = manual_positions[component_name]

        sub_locations = ["bottom_right_box", "bottom_left_box", "top_left_box", "top_right_box"]
        component_property_dict = {}
        for sub_location in sub_locations:
            component_sub_loc = component[sub_location]
            if component_sub_loc[rail_pos_prop] == -1: # ignore the sub locations that is not reachable
                print(f"Ignoring the sublocation <{sub_location}> in component <{component_name}>")
                continue
            component_property = ComponentProperty()
            component_property.railPo = component_sub_loc[rail_pos_prop]
            component_property.dropPo = assemble_post[cartesian_coord_prop]
            bottom_left_coordinates = component_sub_loc[bottom_left_prop][cartesian_coord_prop]
            bottom_right_coordinates = component_sub_loc[bottom_right_prop][cartesian_coord_prop]
            top_left_coordinates = component_sub_loc[top_left_prop][cartesian_coord_prop]
            top_right_coordinates = component_sub_loc[top_right_prop][cartesian_coord_prop]
            component_property.grabPo = get_m_n_well_pos(bottom_left_coordinates, bottom_right_coordinates, top_left_coordinates, top_right_coordinates, 4, 4)
            component_property_dict[sub_location] = component_property
        setattr(constants, component_name, component_property_dict)
        print(f"Finished Dealing with component <{component_name}>")

    return constants
