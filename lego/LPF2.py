from math import expm1

import machine, utime, gc
import math, struct
import utime, _thread

from machine import Pin, UART

length = {'Int8' : 1, 'uInt8' : 1, 'Int16' : 2, 'uInt16' : 2, 'Int32' : 4, 'uInt32' : 4, 'float' : 4}
format = {'Int8' : '<b', 'uInt8' : '<B', 'Int16' : '<h', 'uInt16' : '<H',
     'Int32' : '<l', 'uInt32' : '<L', 'float' : '<f'}

NAME,RAW,Pct,SI,SYM,FCT, FMT = 0x0,0x1,0x2,0x3,0x4,0x5, 0x80
DATA8,DATA16,DATA32,DATAF = 0,1,2,3  # Data type codes
ABSOLUTE,RELATIVE,DISCRETE = 16,8,4

CMD_LLL_SHIFT = 3

def mode(name, size=1, type=DATA8, format='3.0', raw=[0, 100], percent=[0, 100], SI=[0, 100], symbol='',
         functionmap=[ABSOLUTE, 0], view=True):
    fig, dec = format.split('.')
    fred = [name, [size, type, int(fig), int(dec)], raw, percent, SI, symbol, functionmap, view]
    return fred

class LPF2(object):
    def __init__(self, modes: list, id: int = 0, txPin: int = 0, rxPin: int = 1, band: int = 115200, type: int = 65):
        self.modes = modes
        self.txPin = txPin
        self.rxPin = rxPin
        self.band = band
        self.type = type
        self.ser = UART(id, band, tx=Pin(txPin), rx=Pin(rxPin))
        self.connected = False
        self.payload = bytearray([])
        self.current_mode = 0
        self.textBuffer = bytearray(b'                ')

    def readchar(self):
        ch_byte = self.ser.read(1)
        char = -1
        try:
            char = ord(ch_byte.decode('ascii'))
        except:
            pass
        return char

    # ---- data send ----
    def loadPayload(self, format, data):
        pass

    def write(self, payload: bytearray | bytes):
        return self.ser.write(payload)

    def addChksm(self, array: bytearray):
        chksm = 0
        for b in array:
            chksm ^= b
        chksm &= 0xFF
        array.append(chksm)
        return array

    def setType(self, sensorType: int):
        return self.addChksm(bytearray([0x40, sensorType]))

    def defineBand(self, band: int):
        rate = band.to_bytes(4, "little")
        return self.addChksm(bytearray([0x02]) + rate)

    def defineVers(self, hardware: int, software: int):
        hardware = hardware.to_bytes(4, "big")
        software = software.to_bytes(4, "big")
        return self.addChksm(bytearray([0x07]) + hardware + software)

    def padString(self, string: str, num: int, startNum: int):
        reply = bytearray([startNum])
        reply += string
        exp = math.ceil(math.log2(len(string))) if len(string) > 0 else 0
        size = 2 ** exp
        length = size - len(reply)
        for i in range(length):
            reply += bytearray([0])
        return self.addChksm(bytearray(0x80 | exp | num) + reply)

    def buildFuncMap(self, mode: list, num: int, type: int):
        exp = 1 << CMD_LLL_SHIFT
        mapType = mode[0]
        mapOut = mode[1]
        return self.addChksm(bytearray([0x80 | exp | num, type, mapType, mapOut]))

    def buildFormat(self, mode: list, num: int , type: int):
        exp = 2 << CMD_LLL_SHIFT
        sampleSize = mode[0] & 0xFF
        dataType = mode[1] & 0xFF
        figures = mode[2] & 0xFF
        decimals = mode[3] & 0xFF
        return self.addChksm(bytearray([0x01 | exp | num, type, sampleSize, dataType, figures, decimals]))

    def buildRange(self, settings, num, rangeType):
        exp = 3 << CMD_LLL_SHIFT
        minVal = struct.pack('<f', settings[0])
        maxVal = struct.pack('<f', settings[1])
        return self.addChksm(bytearray([0x80 | exp | num, rangeType]) + minVal + maxVal)

    def defineModes(self, modes: list):
        length = (len(modes) - 1) & 0xFF
        views = 0
        for i in modes:
            if i[7]:
                views += 1
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
                data = self.ser.read(0)
                if data[0] == ord(char):
                    status = True
                    break
        return status

    def setupMode(self, mode: list, num: int):
        self.write(self.padString(mode[0], num, 0x00))
        self.write(self.buildRange(mode[2], num, 0x01))
        self.write(self.buildRange(mode[3], num, 0x02))
        self.write(self.buildRange(mode[4], num, 0x03))
        self.write(self.padString(mode[5], num, 0x04))
        self.write(self.buildFuncMap(mode[6], num, 0x05))
        self.write(self.buildRange(mode[1], num, 0x80))

    def hubCallBack(self):
        while True:
            utime.sleep_ms(10)
            if self.connected:
                chr = self.readchar()
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
                            char = self.readchar()
                            size = 2 ** ((char & 0b111000)>>3)
                            mode = char & 0b111
                            ck = ck ^ char
                            for i in range(len(self.textBuffer)):
                                self.textBuffer[i] = ord(b' ')
                            for i in range(size):
                                self.textBuffer[i] = self.readchar()
                                ck = ck ^ self.textBuffer[i]
                            cksm = self.readchar()
                            if cksm == ck:
                                pass
                    elif char == 0x4C:
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
                # _thread.exit()

    def init(self):
        self.connected = False
        tx = Pin(self.txPin)
        tx.value(0)
        utime.sleep_ms(500)
        tx.value(1)
        self.ser.init(2400)
        self.write(b'\x00')
        self.write(self.setType(self.type))
        self.write(self.defineModes(self.modes))
        self.write(self.defineBand(self.band))
        self.write(self.defineVers(2, 2))
        num = len(self.modes) - 1
        for mode in reversed(self.modes):
            self.setupMode(mode, num)
            num -=1
            utime.sleep_ms(5)

        self.write(b'\x04')

        self.connected = self.waitFor(b'\x04')
        # self.connected = True

        self.ser.deinit()
        if self.connected:
            tx = Pin(self.txPin)
            tx.value(0)
            utime.sleep_ms(10)

            self.ser.init(self.band)
            self.loadPayload("uInt8", 0)
            _thread.start_new_thread(self.hubCallBack, ())

