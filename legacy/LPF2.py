from machine import UART, Pin
import math, utime
import ustruct as struct

CMD_MODES   = 0x51
CMD_SPEED   = 0x52
CMD_SELECT  = 0x43
CMD_WRITE   = 0x44
CMD_VERSION = 0x5f
CMD_Data    = 0xC0
CMD_TYPE    = 0x40
CMD_MSG     = 0x46
CMD_LLL_SHIFT = 3

INFO_NAME    = 0x00
INFO_RAW     = 0x01
INFO_PCT     = 0x02
INFO_SI      = 0x03
INFO_UNITS   = 0x04
INFO_MAPPING = 0x05
INFO_MODE_COMBOS = 0x06
INFO_FORMAT  = 0x80

DATA8, DATA16, DATA32, DATAF = 0x00, 0x01, 0x02, 0x03

length = {'Int8' : 1, 'uInt8' : 1, 'Int16' : 2, 'uInt16' : 2, 'Int32' : 4, 'uInt32' : 4, 'float' : 4}
format = {'Int8' : '<b', 'uInt8' : '<B', 'Int16' : '<h', 'uInt16' : '<H',
     'Int32' : '<l', 'uInt32' : '<L', 'float' : '<f'}

class LPF2:
    def __init__(self, modes, txPin = 1, rxPin = 0, baud=115200, type = 65):
        self.modes = modes
        self.type = type
        self.txPin = txPin
        self.rxPin = rxPin
        self.tx = Pin(txPin, Pin.OUT)
        self.rx = Pin(rxPin, Pin.IN)
        self.baud = baud
        self.ser = UART(1, baud, tx=self.tx, rx=self.rx)
        self.connected = False

    def sendData(self, type, data, mode):
        if isinstance(data, list):
            bit = math.floor(math.log2(length[type] * len(data)))
            bit = 4 if bit > 4 else bit  # max 16 bytes total (4 floats)
            array = data[:math.floor((2 ** bit) / length[type])]  # max array size is 16 bytes
            value = b''
            for element in array:
                value += struct.pack(format[type], element)
        else:
            bit = int(math.log2(length[type]))
            value = struct.pack(format[type], data)
        payload = bytearray([CMD_Data | (bit << CMD_LLL_SHIFT) | mode]) + value
        self.write(payload)

    def write(self, payload):
        self.ser.write(payload)

    def addChksm(self, array):
        chksm = 0
        for b in array:
            chksm ^= b
        chksm ^= 0xFF
        array.append(chksm)
        return array

    def setType(self, sensorType):
        return self.addChksm(bytearray([CMD_TYPE, sensorType]))

    def defineBaud(self, baud):
        rate = baud.to_bytes(4, 'little')
        return self.addChksm(bytearray([CMD_SPEED]) + rate)

    def defineVers(self, hardware, software):
        hard = hardware.to_bytes(4, 'big')
        soft = software.to_bytes(4, 'big')
        return self.addChksm(bytearray([CMD_VERSION]) + hard + soft)

    def padString(self, string, num, startNum):
        reply = bytearray([startNum])  # start with name
        reply += string
        exp = math.ceil(math.log2(len(string))) if len(string) > 0 else 0  # find the next power of 2
        size = 2 ** exp
        exp = exp << 3
        length = size - len(string)
        for i in range(length):
            reply += bytearray([0])
        return self.addChksm(bytearray([INFO_FORMAT | exp | num]) + reply)

    def buildFunctMap(self, mode, num, Type):
        exp = 1 << CMD_LLL_SHIFT
        mapType = mode[0]
        mapOut = mode[1]
        return self.addChksm(bytearray([INFO_FORMAT | exp | num, Type, mapType, mapOut]))

    def buildFormat(self, mode, num, Type):
        exp = 2 << CMD_LLL_SHIFT
        sampleSize = mode[0] & 0xFF
        dataType = mode[1] & 0xFF
        figures = mode[2] & 0xFF
        decimals = mode[3] & 0xFF
        return self.addChksm(bytearray([CMD_MODES | exp | num, Type, sampleSize, dataType, figures, decimals]))

    def buildRange(self, settings, num, rangeType):
        exp = 3 << CMD_LLL_SHIFT
        minVal = struct.pack('<f', settings[0])
        maxVal = struct.pack('<f', settings[1])
        return self.addChksm(bytearray([INFO_FORMAT | exp | num, rangeType]) + minVal + maxVal)

    def defineModes(self, modes):
        length = (len(modes) - 1) & 0xFF
        views = 0
        for i in modes:
            if (i[7]):
                views = views + 1
        views = (views - 1) & 0xFF
        return self.addChksm(bytearray([0x49, length, views]))

    def setupMode(self, mode, num):
        self.write(self.padString(mode[0], num, INFO_NAME))  # write name
        self.write(self.buildRange(mode[2], num, INFO_RAW))  # write RAW range
        self.write(self.buildRange(mode[3], num, INFO_PCT))  # write Percent range
        self.write(self.buildRange(mode[4], num, INFO_SI))  # write SI range
        self.write(self.padString(mode[5], num, INFO_UNITS))  # write symbol
        self.write(self.buildFunctMap(mode[6], num, INFO_MAPPING))  # write Function Map
        self.write(self.buildFormat(mode[1], num, INFO_FORMAT))  # write format

    def initialize(self):
        self.connected = False
        self.tx = Pin(self.txPin, Pin.OUT)
        self.rx = Pin(self.rxPin, Pin.IN)
        self.tx.value(0)
        utime.sleep_ms(500)
        self.tx.value(1)
        self.ser.init(self.baud)

        self.write(b'\x00')