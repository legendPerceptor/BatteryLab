import serial
from serial.tools import list_ports

from ..helper.Logger import Logger
from ..helper.utils import get_proper_port_for_device, SupportedDevices

class SuctionPump():

    def __init__(self, logger, status, vacuum_port, baudrate=9600, timeout=1):
        self.vacuum_port = vacuum_port
        self.baudrate = baudrate
        self.timeout = timeout
        self.logger = logger
        self.serialcomm = None
        self.status = status

    def connect_pump(self):
        try:    
            self.serialcomm = serial.Serial(port=self.vacuum_port, baudrate=self.baudrate, timeout=self.timeout)
        except serial.SerialException as e:
            self.logger.error(f"initializing Pump controller on PORT: {self.vacuum_port}...")
            print(f"serial error: {e}")
            return False
        else:
            self.logger.info(f"Pump Controller on PORT: {self.vacuum_port} connected!")
            return True
    
    def check_connection(self):
        return self.serialcomm.isOpen()
    
    def pick(self):
        try:
            self.serialcomm.write(b'P')
            self.status['Vacuumed'] = True
        except serial.SerialException as e:
            self.logger.error(f"Error in pick operation: {e}")
    
    def drop(self):
        try:
            self.serialcomm.write(b'D')
            self.status['Vacuumed'] = False
        except serial.SerialException as e:
            self.logger.error(f"Error in drop operation: {e}")
    
    def off(self):
        try:
            self.serialcomm.write(b'O')
            self.status['Vacuumed'] = False
        except serial.SerialException as e:
            self.logger.error(f"Error in off operation: {e}")

    def disconnect_pump(self):
        if self.serialcomm.isOpen():
            self.serialcomm.close()
        self.logger.info('Pump Controller disconnected!')
        return True
    
def suction_cli_app():
    logger = Logger("suction_pump_test", "logs", "suction_pump_test.log")
    selected_port = get_proper_port_for_device(SupportedDevices.SuctionPump)
    suctionPump = SuctionPump(logger, {}, vacuum_port=selected_port)
    
    if not suctionPump.connect_pump():
        print("Failed to connect to the pump.")
        return
    
    try:
        while True:
            input_str = input("Press [Enter] to reset the pump, [P] to pick up an item, [D] to drop an item, [C] to check connection, [exit] to exit the program: ").strip().lower()
            if input_str == '':
                suctionPump.off()
            elif input_str == 'p':
                suctionPump.pick()
            elif input_str == 'd':
                suctionPump.drop()
            elif input_str == 'c':
                print(f"The connection status: {suctionPump.check_connection()}")
            elif input_str == 'exit':
                break
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        suctionPump.disconnect_pump()
        print("Suction pump disconnected safely.")
