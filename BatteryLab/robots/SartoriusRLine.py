import serial
import time
from ..helper.Logger import Logger
from ..helper.utils import get_proper_port_for_device, SupportedDevices

PRE = bytes([0x01])
POST = bytes([0x0d]) # or POST = str.encode('\r') 
ADR = '1'
BAUD = '5'

ERROR_CODES = {
    "err_codes" :	[str.encode("er1"), str.encode("er2"), str.encode("er3"), str.encode("er4")]
}

REGULAR_COMMANDS = {
    'SET_ADR'  	  	 		: [str.encode("A")  + str.encode(ADR),  str.encode("ok")],
    'BAUD_9600'   	 		: [str.encode("B")  + str.encode('0'),  str.encode("ok")],
    'BAUD_19200'  	 		: [str.encode("B")  + str.encode('1'),  str.encode("ok")],
    'BAUD_115200' 	 		: [str.encode("B")  + str.encode('5'),  str.encode("ok")],
    'DISABLE_LRC' 	 		: [str.encode("C")  + str.encode('0'),  str.encode("ok")],
    'ENABLE_LRC'  	 		: [str.encode("C")  + str.encode('1'),  str.encode("ok")],
    'INITIALIZE'  	 		: [str.encode("RZ") + str.encode('1'),  str.encode("ok")],
    'RESET'  	 		    : [str.encode("RZ") + str.encode('30'), str.encode("ok")],
    'INWARD_SPEED_1' 		: [str.encode("SI") + str.encode('1'),  str.encode("ok")],
    'INWARD_SPEED_2' 		: [str.encode("SI") + str.encode('2'),  str.encode("ok")],
    'INWARD_SPEED_3' 		: [str.encode("SI") + str.encode('3'),  str.encode("ok")],
    'INWARD_SPEED_4' 		: [str.encode("SI") + str.encode('4'),  str.encode("ok")],
    'INWARD_SPEED_5' 		: [str.encode("SI") + str.encode('5'),  str.encode("ok")],
    'INWARD_SPEED_6' 		: [str.encode("SI") + str.encode('6'),  str.encode("ok")],
    'OUTWARD_SPEED_1'		: [str.encode("SO") + str.encode('1'),  str.encode("ok")],
    'OUTWARD_SPEED_2'		: [str.encode("SO") + str.encode('2'),  str.encode("ok")],
    'OUTWARD_SPEED_3'		: [str.encode("SO") + str.encode('3'),  str.encode("ok")],
    'OUTWARD_SPEED_4'		: [str.encode("SO") + str.encode('4'),  str.encode("ok")],
    'OUTWARD_SPEED_5'		: [str.encode("SO") + str.encode('5'),  str.encode("ok")],
    'OUTWARD_SPEED_6'		: [str.encode("SO") + str.encode('6'),  str.encode("ok")],
    'EJECT_AND_HOME'    	: [str.encode("RE") + str.encode('30'),	str.encode("ok")],
    'RUN_TIP_EJECT_CYCLE'	: [str.encode("RE"),  					str.encode("ok")],
    'RUN_BLOWOUT'			: [str.encode("RB"),  					str.encode("ok")],
}

DISPLAY_COMMANDS = {
    'DISPLAY_VERSION'		: [str.encode("DV"),  					str.encode("dv")],
    'DISPLAY_MODEL'			: [str.encode("DV"),  					str.encode("dm")],
    'DISPLAY_CYCLES'		: [str.encode("DX"),  					str.encode("dx")],
    'DISPLAY_IN_SPEED'		: [str.encode("DI"),  					str.encode("di")],
    'DISPLAY_OUT_SPEED'		: [str.encode("DO"),  					str.encode("do")],
    'DISPLAY_RESOLUTION'	: [str.encode("DR"),  					str.encode("dr")],
    'DISPLAY_STATUS'		: [str.encode("DS"),  					str.encode("ds")],
    'DISPLAY_ERRORS'		: [str.encode("DE"),  					str.encode("de")],
    'DISPLAY_POSITION'		: [str.encode("DP"),  					str.encode("dp")],
    'DISPLAY_LIQUID_LEVEL'	: [str.encode("DN"),  					str.encode("dn")],
}

POSITIONAL_COMMANDS = {
    'RUN_TO_POSITION'		: [str.encode("RP"),  str.encode("ok")], #position: nnn, without leading zeros
    'RUN_INWARDS'			: [str.encode("RI"),  str.encode("ok")], #position: nnn, without leading zeros
    'RUN_OUTWARDS'			: [str.encode("RO"),  str.encode("ok")], #position: nnn, without leading zeros
    'TIP_EJECT_AND_MOVE'	: [str.encode("RE"),  str.encode("ok")], #position: nnn, without leading zeros
    'BLOWOUT_AND_MOVE'		: [str.encode("RB"),  str.encode("ok")], #position: nnn, without leading zeros
}

class SartoriusRLine():

    def __init__(self, port, logger = None, log_path="logs", log_file="sartorius_rline.log"):
        self.serial = serial.Serial()
        self.serial.port = port
        self.serial.baudrate = 9600
        self.parity=serial.PARITY_NONE
        self.serial.stopbits=serial.STOPBITS_ONE
        self.serial.bytesize=serial.EIGHTBITS
        self.serial.timeout=1
        self.logger = logger if logger is not None else Logger(logger_name="Sartorius RLine Logger", log_path=log_path, logger_filename=log_file)

    def parseError(self, answer):
        if ERROR_CODES["err_codes"][0] in answer:
            print("Command has not been understood by the module")
        elif ERROR_CODES["err_codes"][1] in answer:
            print("Command has been understood but would result in out-of-bounds state")
        elif ERROR_CODES["err_codes"][2] in answer:
            print("LRC is configured to be used and the checksum does not matc")
        elif ERROR_CODES["err_codes"][3] in answer:
            print("The drive is on and the command or query cannot be answered.")
        else:
            print("Error code unrecognized!")
    
    def waitAck(self, ans):
        char = self.port.read(1)
        answer = str(char)
        while(char):
            char = self.port.read(1)
            answer += str(char)
        self.logger.debug(answer)
        if ans in answer:
            self.logger.debug("Acknowledge received")
            return (True, len(answer))
        else:
            self.logger.debug("nack received")
            self.parseError(answer)
            return (False, len(answer))
        
    def readFeeback(self):
        content = self.serial.read_util(b'\r')
        return content

    def check_connection(self):
        return self.serial.is_open()
    
    def sendCmd(self, cmd, pos = 1):
        if cmd in REGULAR_COMMANDS.keys():
            msg = PRE + str.encode(ADR) +  REGULAR_COMMANDS[cmd][0] + POST
            self.serial.write(msg)
            #time.sleep(0.05)
            res = self.serial.read_until(b'\r')
            if b'er' in res:
                self.parseError(res)
            elif REGULAR_COMMANDS[cmd][1] in res:
                return (True, REGULAR_COMMANDS[cmd][0])
        elif cmd in DISPLAY_COMMANDS.keys():
            msg = PRE + str.encode(ADR) +  DISPLAY_COMMANDS[cmd][0] + POST
            self.serial.write(msg)
            #time.sleep(0.05)
            res = self.serial.read_until(b'\r')
            while not DISPLAY_COMMANDS[cmd][1] in res:
                res = self.serial.read_until(b'\r')
            return int(res[4:-2])
        elif cmd in POSITIONAL_COMMANDS.keys():
            msg = PRE + str.encode(ADR) +  POSITIONAL_COMMANDS[cmd][0] + str.encode(str(pos)) + POST
            self.serial.write(msg)
            #time.sleep(0.05)
            res = self.serial.read_until(b'\r')
            if b'er' in res:
                self.parseError(res)
            elif POSITIONAL_COMMANDS[cmd][1] in res:
                return (True, POSITIONAL_COMMANDS[cmd][0])
        
    def tellPosition(self):
        return self.sendCmd("DISPLAY_POSITION")
        
    def tellLevel(self):
        return self.sendCmd("DISPLAY_LIQUID_LEVEL")

    def initiate_rline(self):
        con_res = self.check_connection()
        if not con_res:
            self.serial.open()
        time.sleep(0.2)
        home_res = self.sendCmd("RESET")
        print("feedback:", self.readFeedback())
        time.sleep(5)
        speed1_res = self.sendCmd("INWARD_SPEED_3")
        print("feedback:", self.readFeedback())
        time.sleep(1)
        speed2_res = self.sendCmd("OUTWARD_SPEED_3")
        print("feedback:", self.readFeedback())
        return all([con_res, home_res[0], speed1_res[0], speed2_res[0]])
    
    def aspirate(self, volume):
        return self.sendCmd("RUN_INWARDS", round(volume))

    def dispense(self, volume):
        return self.sendCmd("RUN_OUTWARDS", round(volume))

    def clear_and_reset(self):
        return self.sendCmd("BLOWOUT_AND_MOVE", 30)

    def reset(self):
        return self.sendCmd("RESET")

    def blowout(self):
        return self.sendCmd('RUN_BLOWOUT')

    def eject(self):
        return self.sendCmd("RUN_TIP_EJECT_CYCLE")

    def eject_and_home(self):
        return self.sendCmd("EJECT_AND_HOME")

    def disconnect(self):
        self.serial.close()
        self.logger.info('Sartorius rLine disconnected!')
    
def sartorius_example_app():
    sartorius_rline = SartoriusRLine(port=get_proper_port_for_device(device_name=SupportedDevices.SartoriusRLine))
    ok = sartorius_rline.initiate_rline()
    if not ok:
        print("Cannot connect to Sartorius rLine.")
        exit()
    try:
        while True:
            input_str = input("Press [Enter] to quit, [P] to pick up pipette, [E] to eject pipette,\n[A] to aspirate liquid, [D] to dispense liquid").strip().lower()
            if input_str == '':
                break
            elif input_str == 'P':
                sartorius_rline.blowout()
            elif input_str == 'E':
                sartorius_rline.eject_and_home()
            elif input_str == 'A':
                volume = float(input("Please input the volume to aspirate: ").strip())
                sartorius_rline.aspirate(volume)
            elif input_str == 'D':
                volume = float(input("Please input the volume to dispense: ").strip())
                sartorius_rline.dispense(volume)
            else:
                print("Invalid input. Please enter a valid option.")
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    finally:
        sartorius_rline.disconnect()
        print("Sartorius rLine disconnected safely.")