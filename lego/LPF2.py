import machine, utime, gc
import math, struct, binascii
import utime, _thread
from machine import Pin
from lego.SoftwareUART import UART

# from lego.SoftwareUART import UART

length = {'Int8' : 1, 'uInt8' : 1, 'Int16' : 2, 'uInt16' : 2, 'Int32' : 4, 'uInt32' : 4, 'float' : 4}
format = {'Int8' : '<b', 'uInt8' : '<B', 'Int16' : '<h', 'uInt16' : '<H',
     'Int32' : '<l', 'uInt32' : '<L', 'float' : '<f'}

NAME,RAW,Pct,SI,SYM,FCT, FMT = 0x0,0x1,0x2,0x3,0x4,0x5, 0x80
DATA8,DATA16,DATA32,DATAF = 0,1,2,3  # Data type codes
ABSOLUTE,RELATIVE,DISCRETE = 16,8,4

CMD_LLL_SHIFT = 3

mode0 = ['LPF2-DETECT',[1,DATA8,3,0],[0,10],[0,100],[0,10],'',[ABSOLUTE,0],True]
mode1 = ['LPF2-COUNT',[1,DATA32,4,0],[0,100],[0,100],[0,100],'CNT',[ABSOLUTE,0],True]
mode2 = ['LPF2-CAL',[3,DATA16,3,0],[0,1023],[0,100],[0,1023],'RAW',[ABSOLUTE,0],False]
defaultModes = [mode0,mode1,mode2]

def mode(name, size=1, type=DATA8, format='3.0', raw=[0, 100], percent=[0, 100], SI=[0, 100], symbol='',
         functionmap=[ABSOLUTE, 0], view=True):
    fig, dec = format.split('.')
    fred = [name, [size, type, int(fig), int(dec)], raw, percent, SI, symbol, functionmap, view]
    return fred

class LPF2(object):
    def __init__(self, modes: list = defaultModes, id = 1, txPin: int = 6, rxPin: int = 5, baud: int = 115200, type: int = 61):
        self.modes = modes
        self.txPin = txPin
        self.rxPin = rxPin
        self.band = baud
        self.type = type
        self.ser = UART(id, tx= Pin(self.txPin, Pin.OUT, value=1), rx=Pin(self.rxPin, Pin.IN, Pin.PULL_UP))
        self.connected = False
        self.payload = bytearray([])
        self.current_mode = 0
        self.textBuffer = bytearray(b'                ')

    def readchar(self):
        if self.ser.any():
            c = self.ser.read(1)
        else:  # Try again once
            utime.sleep_ms(1)
            if self.ser.any():
                c = self.ser.read(1)
            else:
                return -1
        print(c)
        if c == None:
            return -1
        return ord(c)

    # ---- data send ----
    def loadPayload(self, format, data):
        pass

    def write(self, array: bytearray | bytes):
        print(array)
        return self.ser.write(array)

    def addChksm(self, array):
        chksm = 0
        for b in array:
            chksm ^= b
        chksm ^= 0xFF
        array.append(chksm)
        return array

    def setType(self, sensorType: int):
        return self.addChksm(bytearray([0x40, sensorType]))

    def defineBaud(self, baud):
        rate = baud.to_bytes(4, 'little')
        return self.addChksm(bytearray([0x52]) + rate)

    def defineVers(self, hardware, software):
        hard = hardware.to_bytes(4, 'big')
        soft = software.to_bytes(4, 'big')
        return self.addChksm(bytearray([0x5F]) + hard + soft)

    def padString(self, string, num, startNum):
        reply = bytearray([startNum])  # start with name
        reply += string
        exp = math.ceil(math.log2(len(string))) if len(string) > 0 else 0  # find the next power of 2
        size = 2 ** exp
        exp = exp << 3
        length = size - len(string)
        for i in range(length):
            reply += bytearray([0])
        return self.addChksm(bytearray([0x80 | exp | num]) + reply)

    def buildFuncMap(self, mode: list, num: int, type: int):
        exp = 1 << CMD_LLL_SHIFT
        mapType = mode[0]
        mapOut = mode[1]
        return self.addChksm(bytearray([0x80 | exp | num, type, mapType, mapOut]))

    def buildFormat(self, mode, num, Type):
        exp = 2 << CMD_LLL_SHIFT
        sampleSize = mode[0] & 0xFF
        dataType = mode[1] & 0xFF
        figures = mode[2] & 0xFF
        decimals = mode[3] & 0xFF
        return self.addChksm(bytearray([0x80 | exp | num, Type, sampleSize, dataType, figures, decimals]))

    def buildRange(self, settings, num, rangeType):
        exp = 3 << CMD_LLL_SHIFT
        minVal = struct.pack('<f', settings[0])
        maxVal = struct.pack('<f', settings[1])
        return self.addChksm(bytearray([0x80 | exp | num, rangeType]) + minVal + maxVal)

    def defineModes(self, modes):
        length = (len(modes) - 1) & 0xFF
        views = 0
        for i in modes:
            if (i[7]):
                views = views + 1
        views = (views - 1) & 0xFF
        return self.addChksm(bytearray([0x49, length, views]))

    def waitFor(self, char, timeout=2):
        startTime = utime.time()
        currentTime = startTime
        status = False
        while (currentTime - startTime) < timeout:
            utime.sleep_us(5)
            currentTime = utime.time()
            if self.ser.any() > 0:
                data = self.readchar()
                if data == ord(char):
                    status = True
                    break
        return status

    def setupMode(self, mode, num):
        self.write(self.padString(mode[0], num, NAME))  # write name
        self.write(self.buildRange(mode[2], num, RAW))  # write RAW range
        self.write(self.buildRange(mode[3], num, Pct))  # write Percent range
        self.write(self.buildRange(mode[4], num, SI))  # write SI range
        self.write(self.padString(mode[5], num, SYM))  # write symbol
        self.write(self.buildFuncMap(mode[6], num, FCT))  # write Function Map
        self.write(self.buildFormat(mode[1], num, FMT))  # write format

    def hubCallBack(self):
        while True:
            utime.sleep_ms(10)
            if self.connected:
                chr = self.readchar()
                print("asdfasdf", chr)
                while chr >= 0:
                    if chr == 0:
                        pass
                    elif chr == 0x02:
                        pass
                    elif chr == 0x03:
                        mode=self.readchar()
                        cksm = self.readchar()
                        if cksm == 0xff ^ 0x03 ^ mode:
                            self.current_mode = mode
                    elif chr == 0x46:
                        zero = self.readchar()
                        b9 = self.readchar()
                        ck = 0xff ^ zero ^ b9
                        if (zero == 0) & (b9 == 0):
                            chr = self.readchar()
                            size = 2 ** ((chr & 0b111000)>>3)
                            mode = chr & 0b111
                            ck = ck ^ chr
                            for i in range(len(self.textBuffer)):
                                self.textBuffer[i] = ord(b' ')
                            for i in range(size):
                                self.textBuffer[i] = self.readchar()
                                ck = ck ^ self.textBuffer[i]
                            cksm = self.readchar()
                            if cksm == ck:
                                pass
                    elif chr == 0x4C:
                        thing = self.readchar()
                        cksm = self.readchar()
                        if cksm == 0xff ^ 0x4C ^ thing:
                            pass
                    else:
                        pass
                    chr = self.readchar()
            size = self.write(self.payload)
            if not size:
                self.connected = False
            if not self.connected:
                print('disconnected')
                _thread.exit()
            print('asdfasdfasfdsa')

    def init(self):
        self.connected = False
        self.ser.init(baudrate=2400, bits=8, parity=None, stop=1)
        self.write(b'\x00')
        self.write(self.setType(35))
        self.write(self.defineModes(self.modes))
        self.write(self.defineBaud(115200))
        self.write(self.defineVers(2, 2))
        num = len(self.modes) - 1
        for mode in reversed(self.modes):
            self.setupMode(mode, num)
            num -= 1
            utime.sleep_ms(5)

        self.write(b'\x04')

        self.connected = self.waitFor(b'\x04')
        # self.connected = True
        print(self.connected)
        self.ser.deinit()
        if self.connected:
            tx = Pin(self.txPin)
            tx.value(0)
            utime.sleep_ms(10)

            self.ser.init(baudrate=115200, bits=8, parity=None, stop=1)
            self.loadPayload("uInt8", 0)
            _thread.start_new_thread(self.hubCallBack, ())

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