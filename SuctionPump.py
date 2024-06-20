import serial

class SuctionPump():

    def __init__(self, logger, status, vacuum_port, baudrate=110, timeout=1):
        self.vacuum_port = vacuum_port
        self.baudrate = baudrate
        self.timeout = timeout
        self.logger = logger
        self.serialcomm = None
        self.status = status

    def connect_pump(self):
        try:    
            self.serialcomm = serial.Serial(port=self.vacuum_port, baudrate=self.baudrate, timeout=1)
        except:
            self.logger.error(f"initializing Pump controller on PORT: {self.vacuum_port}...")
            return False
        else:
            self.logger.error(f"Pump Controller on PORT: {self.vacuum_port} connected!")
            return True
    
    def check_connection(self):
        return self.serialcomm.isOpen()
    
    def suction_on(self):
        self.serialcomm.write(b'H')
        self.status['Vacuumed'] = True
    
    def suction_off(self):
        self.serialcomm.write(b'L')
        self.status['Vacuumed'] = False

    def disconnect_pump(self):
        self.serialcomm.close()
        self.logger.info('Pump Controller disconnected!')
        return True