import os
import platform
import pyudev  # Linux only
from pathlib import Path
from serial.tools import list_ports
from enum import Enum
import numpy as np
from typing import List
from ..robots.Constants import (
    AssemblyRobotConstants,
    ComponentProperty,
    AssemblyRobotCameraConstants,
    Components,
)
from matplotlib import pyplot as plt
import matplotlib


class SupportedDevices(Enum):
    ZaberLinearRail = 1
    SuctionPump = 2
    SartoriusRLine = 3


DeviceToSerialDict = {
    SupportedDevices.ZaberLinearRail: "FTDI_FT232R_USB_UART_A10NH07T",
    SupportedDevices.SuctionPump: "1a86_USB_Serial",
    SupportedDevices.SartoriusRLine: "FTDI_USB-RS232_Cable_FT4WM2HM",
}


def get_proper_port_for_device(device_name: SupportedDevices):
    usb_ports = list_ports.comports()
    print("please select the correct port by typing the index number:")
    port_index = -1
    # list available devices
    for i, port in enumerate(usb_ports):
        print(f"{i}> name: {port.name}, device: {port.device}")

    selected_port = ""
    if platform.system() == "Linux":
        context = pyudev.Context()
        tty_devices = [
            device
            for device in context.list_devices(subsystem="tty")
            if "ttyUSB" in device.device_node
        ]
        for tty in tty_devices:
            if tty.get("ID_SERIAL") == DeviceToSerialDict[device_name]:
                selected_port = tty.device_node
    elif platform.system() == "Darwin":  # MacBook
        for port in usb_ports:
            if "usbserial" in port.device:
                if (
                    device_name == SupportedDevices.ZaberLinearRail
                    and port.serial_number == "A10NH07T"
                ):
                    selected_port = port.device
                if (
                    device_name == SupportedDevices.SuctionPump
                    and port.serial_number != "A10NH07T"
                ):
                    selected_port = port.device

    while True:
        if platform.system() == "Linux" and selected_port != "":
            break
        port_index_str = input(f"[default is {selected_port}]: ").strip().lower()
        flag = True
        if port_index_str != "":
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


def draw_calculated_points(pos_list, m, n, file_name: str = "generated_points"):
    x_coords = []
    y_coords = []
    manual_x_coords = []
    manual_y_coords = []
    offset = 0.04
    matplotlib.use("Agg")
    fig, ax = plt.subplots(figsize=(m, n))
    for i in range(m * n):
        x = int(i // n)
        y = int(i % n)
        if (
            (x == 0 and y == 0)
            or (x == m - 1 and y == 0)
            or (x == 0 and y == n - 1)
            or (x == m - 1 and y == n - 1)
        ):
            # Add index labels near each point
            ax.text(
                pos_list[i][1] + offset,
                pos_list[i][0] + offset,
                str(i),
                fontsize=8,
                color="red",
            )
            manual_x_coords.append(pos_list[i][0])
            manual_y_coords.append(pos_list[i][1])
        else:
            ax.text(
                pos_list[i][1] + offset,
                pos_list[i][0] + offset,
                str(i),
                fontsize=8,
                color="blue",
            )
            x_coords.append(pos_list[i][0])
            y_coords.append(pos_list[i][1])
    ax.scatter(y_coords, x_coords, color="blue", s=14)  # Plot the points
    ax.scatter(manual_y_coords, manual_x_coords, color="red", s=14)
    for i in range(4):
        if i == 0 or i == 2:
            points_x = [manual_x_coords[i], manual_x_coords[(i + 1) % 4]]
            points_y = [manual_y_coords[i], manual_y_coords[(i + 1) % 4]]
            ax.plot(points_y, points_x, color="r", linewidth=1)
        if i == 0 or i == 1:
            points_x = [manual_x_coords[i], manual_x_coords[(i + 2) % 4]]
            points_y = [manual_y_coords[i], manual_y_coords[(i + 2) % 4]]
            ax.plot(points_y, points_x, color="r", linewidth=1)

    ax.invert_yaxis()  # reverse our x-axis (plotted as y-axis) to make it the same as real scene
    # Add labels and title
    ax.set_xlabel("Y-axis")
    ax.set_ylabel("X-axis")
    ax.set_title(file_name)
    # Display the grid and axes
    ax.grid()
    ax.set_aspect("equal")
    project_dir = Path(__file__).parent.parent.parent / "images"
    os.makedirs(str(project_dir), exist_ok=True)
    plt.savefig(str(project_dir / (file_name + ".png")))
    plt.close(fig)


import numpy as np
from scipy.spatial.transform import Rotation as R


def slerp(start, end, t):
    """Perform spherical linear interpolation (slerp) between two rotations."""
    start_quat = start.as_quat()
    end_quat = end.as_quat()
    dot_product = np.dot(start_quat, end_quat)

    # Adjust for shortest path and ensure continuity
    if dot_product < 0.0:
        end_quat = -end_quat
        dot_product = -dot_product

    # If the two quaternions are very close, we use linear interpolation
    if dot_product > 0.9995:
        interp_quat = (1.0 - t) * start_quat + t * end_quat
        interp_quat /= np.linalg.norm(interp_quat)
    else:
        theta_0 = np.arccos(dot_product)
        sin_theta_0 = np.sin(theta_0)

        theta = theta_0 * t
        sin_theta = np.sin(theta)

        s1 = np.sin(theta_0 - theta) / sin_theta_0
        s2 = sin_theta / sin_theta_0

        interp_quat = s1 * start_quat + s2 * end_quat

    return R.from_quat(interp_quat)


# Integrate slerp in the original function
def get_m_n_well_pos(
    bottom_left_coordinates,
    bottom_right_coordinates,
    top_left_coordinates,
    top_right_coordinates,
    m,
    n,
    num_of_joints=6,
    name: str = "default",
):
    pos_TL, rot_TL = np.array(top_left_coordinates[:3]), R.from_euler(
        "xyz", top_left_coordinates[3:]
    )
    pos_TR, rot_TR = np.array(top_right_coordinates[:3]), R.from_euler(
        "xyz", top_right_coordinates[3:]
    )
    pos_BL, rot_BL = np.array(bottom_left_coordinates[:3]), R.from_euler(
        "xyz", bottom_left_coordinates[3:]
    )
    pos_BR, rot_BR = np.array(bottom_right_coordinates[:3]), R.from_euler(
        "xyz", bottom_right_coordinates[3:]
    )

    well_positions = np.zeros((m, n, num_of_joints))

    pos_list = []
    for i in range(m):
        for j in range(n):
            scaled_i = i / (m - 1)
            scaled_j = j / (n - 1)

            interpolated_position = (
                (1 - scaled_i) * (1 - scaled_j) * pos_TL
                + scaled_i * (1 - scaled_j) * pos_TR
                + (1 - scaled_i) * scaled_j * pos_BL
                + scaled_i * scaled_j * pos_BR
            )

            interp_rot_top = slerp(rot_TL, rot_TR, scaled_i)
            interp_rot_bottom = slerp(rot_BL, rot_BR, scaled_i)
            interpolated_rotation = slerp(interp_rot_top, interp_rot_bottom, scaled_j)

            euler_angles = interpolated_rotation.as_euler("xyz")
            well_positions[i, j] = np.concatenate((interpolated_position, euler_angles))
            pos_list.append(list(well_positions[i, j]))

    draw_calculated_points(pos_list, m, n, name)
    return pos_list


def create_assembly_robot_camera_constants_from_manual_positions(
    camera_manual_positions,
) -> AssemblyRobotCameraConstants:
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
    components = [
        "CathodeCase",
        "Cathode",
        "Spacer",
        "SpacerExtra",
        "Anode",
        "Washer",
        "Separator",
        "AnodeCase",
    ]
    constants.RobotPose = camera_manual_positions["RobotPose"]["cartesian"]
    for component in components:
        setattr(constants, component, camera_manual_positions[component])
    return constants


def create_assembly_robot_constants_from_manual_positions(
    manual_positions,
) -> AssemblyRobotConstants:
    """
    Obtain the assembly robot constants from manually aligned positions.

    Parameters:
        manual_positions: the YAML file content parsed by yaml.safe_load()

    Returns:
        AssemblyRobotConstants: generate all the well positions and record other location constants.

    There are 8 fields for different components, namely Cathode Case, Cathode, Spacer, SpacerExtra, Anode, Washer, Separator, Anode Case.
    Each component tray will have 16 manually posed positions at 4 corners (x, y, z, alpha, beta, gama). There are 64 wells on each tray.
    The location for each well will be bilinearly interpolated by the 4 positions.
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
    constants.POST_C_SK_PO = assemble_post["suction"][cartesian_coord_prop]
    constants.POST_RAIL_LOCATION = assemble_post[rail_pos_prop]
    constants.POST_C_GRIPPER_PO = assemble_post["gripper"][cartesian_coord_prop]

    lookup_camera = manual_positions["LookupCamera"]
    constants.LOOKUP_CAM_SK_PO = lookup_camera["suction"][cartesian_coord_prop]
    constants.LOOKUP_CAM_RAIL_LOCATION = lookup_camera[rail_pos_prop]
    constants.LOOKUP_CAM_GRIPPER_PO = lookup_camera["gripper"][cartesian_coord_prop]

    components = [
        Components.CathodeCase.name,
        Components.Cathode.name,
        Components.Spacer.name,
        Components.SpacerExtra.name,
        Components.Anode.name,
        Components.Washer.name,
        Components.Separator.name,
        Components.AnodeCase.name,
    ]

    for component_name in components:
        print(f"Dealing with component <{component_name}>")
        if component_name not in manual_positions:
            print(
                f"The component <{component_name}> is not in the well positions YAML file"
            )
            continue
        component = manual_positions[component_name]
        if component_name == "Washer":
            tool = "gripper"
        else:
            tool = "suction"
        sub_locations = [
            "bottom_right_box",
            "bottom_left_box",
            "top_left_box",
            "top_right_box",
        ]
        component_property_dict = {}
        for sub_location in sub_locations:
            component_sub_loc = component[sub_location]
            if (
                component_sub_loc[rail_pos_prop] == -1
            ):  # ignore the sub locations that is not reachable
                print(
                    f"Ignoring the sublocation <{sub_location}> in component <{component_name}>"
                )
                continue
            component_property = ComponentProperty()
            component_property.railPo = component_sub_loc[rail_pos_prop]
            if tool == "suction":
                component_property.dropPo = assemble_post["suction"][
                    cartesian_coord_prop
                ]
            else:
                component_property.dropPo = assemble_post["gripper"][
                    cartesian_coord_prop
                ]
            m = 4
            n = 4
            if "shape" in component_sub_loc:
                m, n = list(component_sub_loc["shape"])
            component_property.shape = [m, n]
            bottom_left_coordinates = component_sub_loc[bottom_left_prop][
                cartesian_coord_prop
            ]
            bottom_right_coordinates = component_sub_loc[bottom_right_prop][
                cartesian_coord_prop
            ]
            top_left_coordinates = component_sub_loc[top_left_prop][
                cartesian_coord_prop
            ]
            top_right_coordinates = component_sub_loc[top_right_prop][
                cartesian_coord_prop
            ]
            component_property.grabPo = get_m_n_well_pos(
                bottom_left_coordinates,
                bottom_right_coordinates,
                top_left_coordinates,
                top_right_coordinates,
                m,
                n,
                num_of_joints=6,
                name=f"{component_name}-{sub_location}",
            )
            component_property_dict[sub_location] = component_property
        setattr(constants, component_name, component_property_dict)
        print(f"Finished Dealing with component <{component_name}>")

    return constants
