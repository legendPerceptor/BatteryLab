import serial
from serial.tools import list_ports
from Logger import Logger

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
    logger = Logger("suction_pump_test", "logs/suction_pump_test.log")
    usb_ports = list_ports.comports()
    print("please select the correct port by typing the index number:")
    for i, port in enumerate(usb_ports):
        print(f'{i}: {port.name}')
        if 'usbserial' in port.name or 'COM' in port.name:
            port_index = i
    
    while True:
        port_index_str = input(f"[default is {port_index}]: ").strip().lower()
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
    suctionPump = SuctionPump(logger, {}, vacuum_port=usb_ports[port_index].device)
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