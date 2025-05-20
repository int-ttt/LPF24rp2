from time import sleep

from machine import UART, Pin
import math, utime, micropython
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

ABSOLUTE,RELATIVE,DISCRETE = 16,8,4
WeDo_Ultrasonic, SPIKE_Color, SPIKE_Ultrasonic = 35, 61, 62
Ev3_Utrasonic = 34

def mode(name, size=1, type=DATA8, format='3.0', raw=[0, 100], percent=[0, 100], SI=[0, 100], symbol='',
         functionmap=[ABSOLUTE, 0], view=True):
     fig, dec = format.split('.')
     fred = [name, [size, type, int(fig), int(dec)], raw, percent, SI, symbol, functionmap, view]
     return fred


class LPF2:
    def __init__(self, modes, txPin = 0, rxPin = 1, baud=115200, type = 65):
        self.modes = modes
        self.type = type
        self.txPin = txPin
        self.rxPin = rxPin
        self.baud = baud
        self.ser = UART(0, baud, tx=Pin(txPin), rx=Pin(rxPin))
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
        self.tx = Pin(self.txPin)
        self.rx = Pin(self.rxPin)
        self.tx.value(0)
        utime.sleep_ms(500)
        self.tx.value(1)
        self.ser.init(self.baud)

        self.write(b'\x00')
micropython.alloc_emergency_exception_buf(200)

modes = [
mode('int8',type = DATA8),
mode('int16', type = DATA16),
mode('int32', type = DATA32),
mode('float', format = '2.1', type = DATAF),
mode('int8_array',size = 4, type = DATA8),
mode('int16_array',size = 4, type = DATA16),
mode('int32_array',size = 4, type = DATA32),
mode('float_array',size = 4, format = '2.1', type = DATAF)
]
pin = Pin("LED", Pin.OUT)

pin.toggle()

lpf2 = LPF2(modes)    # OpenMV
#lpf2 = LPF2(3, 'P4', 'P5', modes, SPIKE_Ultrasonic, timer = 4, freq = 5)    # OpenMV
#lpf2 = Prime_LPF2(1, 'Y1', 'Y2', modes, SPIKE_Ultrasonic, timer = 4, freq = 5)    # PyBoard
# use EV3_LPF2 or Prime_LPF2 - also make sure to select the port type on the EV3 to be ev3-uart

lpf2.initialize()
sleep(1)
value = 0
pin.toggle()
v = 0
# Loop
while True:
     # if not lpf2.connected:
     #      v=0
     #      pin.toggle()
     #      utime.sleep_ms(200)
     #      lpf2.initialize()
     # else:
     v=1
     if v == 1:
         pin.toggle()

     if value < 9:
          value = value + 1
     else:
          value = 0

     lpf2.sendData('Int8',value, 0)
     print(value)

     utime.sleep_ms(200)


[
#
#
# def readchar():
#      if uart.any():
#           c = uart.read(1)
#      else:  # Try again once
#           utime.sleep_ms(1)
#           if uart.any():
#                c = uart.read(1)
#           else:
#                return -1
#      print(c)
#      if c == None:
#           return -1
#      return ord(c)
#
#
# while True:
#      tx = Pin(0, Pin.OUT)
#      tx.value(1)
#      utime.sleep_ms(500)
#      tx.value(0)
#      uart.init(baudrate=2400, bits=8, parity=None, stop=1)
     # uart.write(b'\x00')
     # uart.write(bytearray(b'@>\x81'))
     # uart.write(bytearray(b'I\x01\x01\xb6'))
     # uart.write(bytearray(b'R\x00\xc2\x01\x00n'))
     # uart.write(bytearray(b'_\x00\x00\x00\x02\x00\x00\x00\x02\xa0'))
     # uart.write(bytearray(b'\x91\x00TEMPb'))
     # uart.write(bytearray(b'\x99\x01\x00\x00\x00\x00\x00\x00\xc8B\xed'))
     # uart.write(bytearray(b'\x99\x02\x00\x00\x00\x00\x00\x00\xc8B\xee'))
     # uart.write(bytearray(b'\x99\x03\x00\x00\x00\x00\x00\x00\xc8B\xef'))
     # uart.write(bytearray(b'\x81\x04\x00z'))
     # uart.write(bytearray(b'\x89\x05\x10\x00c'))
     # uart.write(bytearray(b'\x91\x80\x04\x00\x03\x00\xe9'))
     # uart.write(bytearray(b'\x90\x00TOF\x002'))
     # uart.write(bytearray(b'\x98\x01\x00\x00\x00\x00\x00\x00\xc8B\xec'))
     # uart.write(bytearray(b'\x98\x02\x00\x00\x00\x00\x00\x00\xc8B\xef'))
     # uart.write(bytearray(b'\x98\x03\x00\x00\x00\x00\x00\x00\xc8B\xee'))
     # uart.write(bytearray(b'\x80\x04\x00{'))
     # uart.write(bytearray(b'\x88\x05\x10\x00b'))
     # uart.write(bytearray(b'\x90\x80\x04\x01\x03\x00\xe9'))
     # uart.write(b'\x04')
     # uart.write(b'\x00')
     # uart.write(bytearray(b'\x40\x25\x9a'))
     # uart.write(bytearray(b'\x51\x07\x07\x0a\x07\xa3'))
     # uart.write(bytearray(b'\x52\x00\xc2\x01\x00\x6e'))
     # uart.write(bytearray(b'\x43\x02\xbe'))
     # uart.write(bytearray(b'\x91\x00TEMPb'))
     # uart.write(bytearray(b'\x99\x01\x00\x00\x00\x00\x00\x00\xc8B\xed'))
     # uart.write(bytearray(b'\x99\x02\x00\x00\x00\x00\x00\x00\xc8B\xee'))
     # uart.write(bytearray(b'\x99\x03\x00\x00\x00\x00\x00\x00\xc8B\xef'))
     # uart.write(bytearray(b'\x81\x04\x00z'))
     # uart.write(bytearray(b'\x89\x05\x10\x00c'))
     # uart.write(bytearray(b'\x91\x80\x04\x00\x03\x00\xe9'))
     # uart.write(bytearray(b'\x90\x00TOF\x002'))
     # uart.write(bytearray(b'\x98\x01\x00\x00\x00\x00\x00\x00\xc8B\xec'))
     # uart.write(bytearray(b'\x98\x02\x00\x00\x00\x00\x00\x00\xc8B\xef'))
     # uart.write(bytearray(b'\x98\x03\x00\x00\x00\x00\x00\x00\xc8B\xee'))
     # uart.write(bytearray(b'\x80\x04\x00{'))
     # uart.write(bytearray(b'\x88\x05\x10\x00b'))
     # uart.write(bytearray(b'\x90\x80\x04\x01\x03\x00\xe9'))
     # uart.write(b'\x04')
     # print(uart.read(1))

#>>> b'\x00'
# bytearray(b'@>\x81')
# bytearray(b'I\x01\x01\xb6')
# bytearray(b'R\x00\xc2\x01\x00n')
# bytearray(b'_\x00\x00\x00\x02\x00\x00\x00\x02\xa0')
# bytearray(b'\x91\x00TEMPb')
# bytearray(b'\x99\x01\x00\x00\x00\x00\x00\x00\xc8B\xed')
# bytearray(b'\x99\x02\x00\x00\x00\x00\x00\x00\xc8B\xee')
# bytearray(b'\x99\x03\x00\x00\x00\x00\x00\x00\xc8B\xef')
# bytearray(b'\x81\x04\x00z')
# bytearray(b'\x89\x05\x10\x00c')
# bytearray(b'\x91\x80\x04\x00\x03\x00\xe9')
# bytearray(b'\x90\x00TOF\x002')
# bytearray(b'\x98\x01\x00\x00\x00\x00\x00\x00\xc8B\xec')
# bytearray(b'\x98\x02\x00\x00\x00\x00\x00\x00\xc8B\xef')
# bytearray(b'\x98\x03\x00\x00\x00\x00\x00\x00\xc8B\xee')
# bytearray(b'\x80\x04\x00{')
# bytearray(b'\x88\x05\x10\x00b')
# bytearray(b'\x90\x80\x04\x01\x03\x00\xe9')
# b'\x04'
# Success
]# def readchar():
#      if uart.any():
#           c = uart.read(1)
#      else:  # Try again once
#           utime.sleep_ms(1)
#           if uart.any():
#                c = uart.read(1)
#           else:
#                return -1
#      print(c)
#      if c == None:
#           return -1
#      return ord(c)
#

# uart = UART(1, tx = Pin(8), rx = Pin(9))

# while True:
#      tx = Pin(0, Pin.OUT)
#      tx.value(1)
#      utime.sleep_ms(500)
#      tx.value(0)
#      uart.init(baudrate=2400, bits=8, parity=None, stop=1)
#      uart.write(b'\x00')
#      uart.write(bytearray(b'@>\x81'))
#      uart.write(bytearray(b'I\x01\x01\xb6'))
#      uart.write(bytearray(b'R\x00\xc2\x01\x00n'))
#      # uart.write(bytearray(b'_\x00\x00\x00\x02\x00\x00\x00\x02\xa0'))
#      # uart.write(bytearray(b'\x91\x00TEMPb'))
#      # uart.write(bytearray(b'\x99\x01\x00\x00\x00\x00\x00\x00\xc8B\xed'))
#      # uart.write(bytearray(b'\x99\x02\x00\x00\x00\x00\x00\x00\xc8B\xee'))
#      # uart.write(bytearray(b'\x99\x03\x00\x00\x00\x00\x00\x00\xc8B\xef'))
#      # uart.write(bytearray(b'\x81\x04\x00z'))
#      # uart.write(bytearray(b'\x89\x05\x10\x00c'))
#      # uart.write(bytearray(b'\x91\x80\x04\x00\x03\x00\xe9'))
#      # uart.write(bytearray(b'\x90\x00TOF\x002'))
#      # uart.write(bytearray(b'\x98\x01\x00\x00\x00\x00\x00\x00\xc8B\xec'))
#      # uart.write(bytearray(b'\x98\x02\x00\x00\x00\x00\x00\x00\xc8B\xef'))
#      # uart.write(bytearray(b'\x98\x03\x00\x00\x00\x00\x00\x00\xc8B\xee'))
#      # uart.write(bytearray(b'\x80\x04\x00{'))
#      # uart.write(bytearray(b'\x88\x05\x10\x00b'))
#      # uart.write(bytearray(b'\x90\x80\x04\x01\x03\x00\xe9'))
#      # uart.write(b'\x04')
#      # uart.write(b'\x00')
#      # uart.write(bytearray(b'\x40\x25\x9a'))
#      # uart.write(bytearray(b'\x51\x07\x07\x0a\x07\xa3'))
#      # uart.write(bytearray(b'\x52\x00\xc2\x01\x00\x6e'))
#      # uart.write(bytearray(b'\x43\x02\xbe'))
#      # uart.write(bytearray(b'\x91\x00TEMPb'))
#      # uart.write(bytearray(b'\x99\x01\x00\x00\x00\x00\x00\x00\xc8B\xed'))
#      # uart.write(bytearray(b'\x99\x02\x00\x00\x00\x00\x00\x00\xc8B\xee'))
#      # uart.write(bytearray(b'\x99\x03\x00\x00\x00\x00\x00\x00\xc8B\xef'))
#      # uart.write(bytearray(b'\x81\x04\x00z'))
#      # uart.write(bytearray(b'\x89\x05\x10\x00c'))
#      # uart.write(bytearray(b'\x91\x80\x04\x00\x03\x00\xe9'))
#      # uart.write(bytearray(b'\x90\x00TOF\x002'))
#      # uart.write(bytearray(b'\x98\x01\x00\x00\x00\x00\x00\x00\xc8B\xec'))
#      # uart.write(bytearray(b'\x98\x02\x00\x00\x00\x00\x00\x00\xc8B\xef'))
#      # uart.write(bytearray(b'\x98\x03\x00\x00\x00\x00\x00\x00\xc8B\xee'))
#      # uart.write(bytearray(b'\x80\x04\x00{'))
#      # uart.write(bytearray(b'\x88\x05\x10\x00b'))
#      # uart.write(bytearray(b'\x90\x80\x04\x01\x03\x00\xe9'))
#      # uart.write(b'\x04')
#      print(uart.read(1))
#      print(uart.read(3))
#      print(uart.read(4))
#      print(uart.read(6))

#>>> b'\x00'
# bytearray(b'@>\x81')
# bytearray(b'I\x01\x01\xb6')
# bytearray(b'R\x00\xc2\x01\x00n')
# bytearray(b'_\x00\x00\x00\x02\x00\x00\x00\x02\xa0')
# bytearray(b'\x91\x00TEMPb')
# bytearray(b'\x99\x01\x00\x00\x00\x00\x00\x00\xc8B\xed')
# bytearray(b'\x99\x02\x00\x00\x00\x00\x00\x00\xc8B\xee')
# bytearray(b'\x99\x03\x00\x00\x00\x00\x00\x00\xc8B\xef')
# bytearray(b'\x81\x04\x00z')
# bytearray(b'\x89\x05\x10\x00c')
# bytearray(b'\x91\x80\x04\x00\x03\x00\xe9')
# bytearray(b'\x90\x00TOF\x002')
# bytearray(b'\x98\x01\x00\x00\x00\x00\x00\x00\xc8B\xec')
# bytearray(b'\x98\x02\x00\x00\x00\x00\x00\x00\xc8B\xef')
# bytearray(b'\x98\x03\x00\x00\x00\x00\x00\x00\xc8B\xee')
# bytearray(b'\x80\x04\x00{')
# bytearray(b'\x88\x05\x10\x00b')
# bytearray(b'\x90\x80\x04\x01\x03\x00\xe9')
# b'\x04'
# Success

# uart = UART(0, 9600)
# uar1 = UART(1, 9600, timeout=2, tx=Pin(4), rx=Pin(5))
#
# while True:
#      uart.write(b'a')
#      uart.flush()
#      utime.sleep_ms(1000)
#      print(uar1.any(), uar1.read(1))