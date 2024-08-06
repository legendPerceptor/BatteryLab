import os
import platform
import pyudev # Linux only
from pathlib import Path
from serial.tools import list_ports
from enum import Enum

class SupportedDevices(Enum):
    ZaberLinearRail = 1
    SuctionPump = 2

DeviceToSerialDict = {
    SupportedDevices.ZaberLinearRail : "FTDI_FT232R_USB_UART_A10NH07T",
    SupportedDevices.SuctionPump: "1a86_USB_Serial"
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