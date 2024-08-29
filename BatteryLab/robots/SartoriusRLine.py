import serial

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

    def __init__(self, port):
        self.serial = serial.Serial()
        self.serial.port = port
        self.serial.baudrate = 9600
        self.parity=serial.PARITY_NONE
        self.serial.stopbits=serial.STOPBITS_ONE
        self.serial.bytesize=serial.EIGHTBITS
        self.serial.timeout=1

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