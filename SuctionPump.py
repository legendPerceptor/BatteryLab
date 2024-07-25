import serial
from serial.tools import list_ports
from Logger import Logger
from utils import get_proper_port_for_device, SupportedDevices

class SuctionPump():

    def __init__(self, logger, status, vacuum_port, baudrate=9600, timeout=100):
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
            self.logger.error(f"Pump Controller on PORT: {self.vacuum_port} connected!")
            return True
    
    def check_connection(self):
        return self.serialcomm.isOpen()
    
    def pick(self):
        self.serialcomm.write(b'P')
        self.status['Vacuumed'] = True
    
    def drop(self):
        self.serialcomm.write(b'D')
        self.status['Vacuumed'] = False
    
    def off(self):
        self.serialcomm.write(b'O')
        self.status['Vacuumed'] = False

    def disconnect_pump(self):
        self.serialcomm.close()
        self.logger.info('Pump Controller disconnected!')
        return True
    
def main():
    logger = Logger("suction_pump_test", "logs", "suction_pump_test.log")
    selected_port = get_proper_port_for_device(SupportedDevices.SuctionPump)
    suctionPump = SuctionPump(logger, {}, vacuum_port=selected_port)
    suctionPump.connect_pump()
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

if __name__ == '__main__':
    main()