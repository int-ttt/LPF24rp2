# coding=utf-8
#
#  ____________               _________________________________________________________________              __________
#  |          |               |     |                                                     ____|              |        |
#  |          | -- uart_RX -> |  U  | -> rx_l -> uart_rx_q --> xbus_tx_q --> xbus_A_l -> | SM | <- xbus_A -> |        |
#  |          |               |  A  |            uart_tx_q <-- xbus_read <- xbus_A_rq <- |____|              | device |
#  |          |               |  R  |                                                     ____|              |        |
#  |          |               |  T  |            uart_rx_q --> xbus_tx_q --> xbus_B_l -> | SM | <- xbus_B -> |        |
#  |          | <- uart_TX -- |  1  | <- tx_l <- uart_tx_q <-- xbus_read <- xbus_B_rq <- |____|              |________|
#  |          |               |_____|               ^ <------- gpio events <----------------- | <- gpio
#  | RPi      |               |                     ^ <------- status/info                    |
#  | the app  |               | RP2040              ^ <------- errors                         |
#  ------------               -----------------------------------------------------------------
#
# The RP2040 serves two purposes in the design;
#   1) An intermediary between the app on the RPi and devices that utilize non-standard and/or proprietary physical
#      signaling and IO making it difficult if not impossible to support the connection of these devices directly to
#      the RPi.  Instead these devices are connected to the RP2040 and the RP2040s Programmable IO (PIO) State
#      Machines (SM) are utilized to meet the unique physical signaling and IO requirements.  Essentially the RP2040
#      performs like a programmable IO and signalling translator.  The RP2040 receives data from RPi via uart_RX which
#      it sends to devices on the eXpansion BUS (ie: xbus) via xbus_A, xbus_B, both or neither depending on the specifics
#      of the physical signaling and IO in use by the devices.  The RP2040 receives data from the devices on the xbus via
#      xbus_A, xbus_B or both (depending on the specifics of the physical signaling and IO in use by the devices) which it
#      sends to the application on the RPi via uart_TX.
#   2) It hosts GPIOs.  The number it hosts and how they are utilized will be determined later.
#
# The RP2040 receives 1) XBUS messages, 2) GPIO request messages and 3) command request messages from the app on the
# RPi.  These are received via the RP2040 uart_RX pin which is the RX pin of the RP2040's UART1.  The first 4 characters
# of messages received on UART1 define the message type; XBUS for XBUS messages, GPIO for GPIO requests and COMM for
# command requests.  GPIO and COMM messages include a message ID in characters 5 - 7 of the message, this message ID is
# included in GPIO and COMM response messages transmitted from the RP2040 to the app on the RPi enabling the app on
# the RPi to track the status of messages if so desired.  Messages received on UART1 are processed by the _uart.read()
# method as defined below;
#
#   1) XBUS messages are parsed and placed into the XBUS TX FIFO QUEUE (_xbus_tx_q) for transmission on the XBUS
#   2) GPIO request messages are parsed, the request is executed on the RP2040, the result is included in a GPIO
#      request response message which includes the GPIO request message Message ID and the GPIO request response message
#      is placed into the UART TX FIFO QUEUE (_uart_tx_q) for transmission on the UART
#   3) COMM messages are parsed, the associated command(s) are executed on the RP2040, the result is included in a COMM
#      request response message which includes the COMM request message Message ID and the COMM request response message
#      is placed into the UART TX FIFO QUEUE (_uart_tx_q) for transmission on the UART
#

import _thread
import binascii
from machine import Pin, UART
from rp2 import asm_pio, PIO, StateMachine
import usys, uselect, utime

_active = True
_version = 0.1

_uart = None
_uart_active = False
_uart_rx_max = 2
_uart_tx_max = 2

_xbus = None
_xbus_active = False
_xbus_rx_max = 2
_xbus_tx_max = 2

# the global uart_rx queue and queue lock
_uart_rx_q = []
_uart_rx_q_lock = _thread.allocate_lock()

# the global uart_tx queue and queue lock
_uart_tx_q = []
_uart_tx_q_lock = _thread.allocate_lock()

# the global xbus_rx queue and queue lock
_xbus_rx_q = []
_xbus_rx_q_lock = _thread.allocate_lock()

# the global xbus_tx queue and queue lock
_xbus_tx_q = []
_xbus_tx_q_lock = _thread.allocate_lock()

#
# start of PIO ASM
#

# device_poller_tx
# device uses a polling mechanism to query for clients with data to send
# polling consists of setting line low for ~13ms then sending 3 0xFF bytes w/o parity
# with a ~1ms delay between each byte.  Clients use the rising edge of the start bit
# to sync their responses.  All clients respond to the first 2 bytes with 0x00.  Any
# client with data to send responds to the 3rd byte with a byte in which the bit
# representing their address is set high
#
# device_poller_tx implements the ~13ms delay, the 3 0xFF bytes and the 3 ~1ms delays
# after each byte.  It sets an IRQ immediately prior to asserting the start bit for the
# transmission of each of the 3 0xFF bytes and sets an IRQ immediately after the ~1ms
# delay following the 3rd 0xFF byte and the line reset to high - the IRQ is used to
# signal the upper layer on the progress of the polling cycle
#

# device_poller_tx()
#
#         ______               ___ ___ ___ ___ ___ ___ ___ ___ ___
# Byte 1        |_____________|   |   |   |   |   |   |   |   |   |___|___|________|
#          Idle  13.02083333ms   S   1   2   3   4   5   6   7   8   T   T .59895ms
#
#          ___ ___ ___ ___ ___ ___ ___ ___ ___
# Byte 2  |   |   |   |   |   |   |   |   |   |___|___|________|
#           S   1   2   3   4   5   6   7   8   T   T  .59895ms
#
#          ___ ___ ___ ___ ___ ___ ___ ___ ___                  ________
# Byte 3  |   |   |   |   |   |   |   |   |   |___|___|________|
#           S   1   2   3   4   5   6   7   8   T   T  .59895ms   Idle
#
# For transmission of polling messages device uses a modified 8N2 scheme, polling
# messages consist of 3 bytes of 0x00 (inverted to 0xFF).
#
# While idle the line is high.  Before transmitting the first polling byte the
# line is pulled low for 13.02083333ms.  This is followed immediately by an inverted
# start bit (S), 8 inverted data bits (1 - 8) and two inverted stop bits (T).  The
# 2nd stop bit is followed by a .59895ms delay with the line low.  The 13.02083333ms
# delay does not apply to polling bytes 2 & 3.  For bytes 2 & 3 their inverted start
# bit (S) begins immediately after the .59895ms delay of the prior byte.  After the
# .59895ms delay of the last polling byte the line transitions to idle state with the
# line high.
#
# device_poller_tx implements this logic in full including definition of the 3 bytes
# as 0xFF.  The upper layer DOES NOT transmit the 3 0xFF bytes to device_poller_tx.
# To trigger device_poller_tx the upper layer MUST ONLY transmit one byte of data to
# device_poller_tx via .put() and it MUST DO SO ONLY ONCE.  Once the upper layer sends
# this device_poller_tx immediately starts the ~13ms delay followed by transmission of
# the 3 0xFF bytes.
#
# device_poller_tx sets IRQ 0 immediately prior to starting transmission of the start
# bit of each of the 3 0xFF bytes.  It sets IRQ 0 again immediately prior to setting
# the line high for idle state after the .59895ms delay following the 3rd 0xFF byte.
# The use of IRQs permits the upper layer to track the progress of the polling byte
# transmittals and process conclusion
#
@asm_pio(sideset_init=PIO.OUT_HIGH, out_init=PIO.OUT_HIGH, out_shiftdir=PIO.SHIFT_RIGHT)
def device_poller_tx():  #
    pull()                          # wait for upper layer to trigger us
    out(null, osr)                  # dump whatever upper layer sent into the bit bucket

                                    # this block asserts line low for 500 cycles, 13.02083333ms
    set(y, 2).side(0)[6]            # set loop counter, set line low, consume 7 cycles
    label("pre_tx_outer")           # loop 3 times, consume 162 cycles per loop (486 cycles total)
    set(x, 31)                      # set loop counter, consume 1 cycle
    label("pre_tx_inner")           # loop 32 times, consume 160 cycles
    jmp(x_dec, "pre_tx_inner")[4]   # decrement counter, consume 5 cycles
    jmp(y_dec, "pre_tx_outer")      # decrement counter, consume 1 cycle
    set(y, 2)[5]                    # set loop counter for "tx" block, consume 6 cycles for 499 total

                                    # this block sends out 3 0xFF polling bytes each followed by .59895ms line low
    label("tx")                     # loop 3 times, consume 111 cycles per loop
                                    # this IRQ is 500th cycle for block above, 111th cycle for bytes 1 & 2
    irq(0)                          # signal upper layer, consume 1 cycle
    set(x, 9).side(1)[1]            # set loop counter, set line high for start+byte, consume 2 cycles
    label("bits_loop")              # loop 10 times, consume 70 cycles
    jmp(x_dec, "bits_loop")[6]      # decrement counter, consume 7 cycles
    set(x, 5).side(0)               # set loop counter, set line low, consume 1 cycle
    label("stop_loop")              # loop 6 times, consume 36 cycles
    jmp(x_dec, "stop_loop")[5]      # decrement counter, consume 6 cycles
    jmp(y_dec, "tx")                # decrement counter, consume 1 cycle

    irq(0)                          # signal upper layer, provide 11th cycle for byte 3
    nop().side(1)                   # set pin back to default of high

# device_tx()
#
#             ______             ___ ___ ___ ___ ___ ___ ___ ___ ___ ___
# First Byte        |___________|   |___|___|___|___|___|___|___|___|___|___|___|___|
#              Idle   4ms delay   S   1   2   3   4   5   6   7   8   P   T   T   D
#
#              ___ ___ ___ ___ ___ ___ ___ ___ ___ ___
#     Byte n  |___|___|___|___|___|___|___|___|___|___|___|___|___|
#               S   1   2   3   4   5   6   7   8   P   T   T   D
#
#              ___ ___ ___ ___ ___ ___ ___ ___ ___ ___             _________________
#  Last Byte  |___|___|___|___|___|___|___|___|___|___|___|___|___|
#               S   1   2   3   4   5   6   7   8   P   T   T   D    Idle
#
# For transmission of all messages (except polling) device uses a modified 8E2 scheme.
# While idle the line is high.  Before transmitting the first byte of a message the
# line is pulled low for ~4ms.  This is followed immediately by an inverted start bit
# (S), 8 inverted data bits (1 - 8), an inverted parity bit (P) and two inverted stop
# bits (T).  The 2nd stop bit is followed by a 1 bit delay (D) with the line low.  The
# 4ms delay does not apply to bytes 2+ of the message.  For bytes 2+ their inverted
# start bit (S) begins immediately after the 1 bit delay (D) of the prior message byte.
# After the 1 bit delay (D) of the last byte of the message the line transitions to idle
# state with the line high.
#
# ademoc_tx implements this logic EXCEPT for the inversion of the 8 data bits, the
# calculation of the parity bit and the inversion of the parity bit.  These MUST be
# handled by the upper layer.  The upper layer MUST send device_tx 9 bits for each
# 8 bit byte.  The upper layer MUST send these 9 bits Big Endian, with parity (P)
# in MSB
#
@asm_pio(sideset_init=PIO.OUT_HIGH, out_init=PIO.OUT_HIGH, out_shiftdir=PIO.SHIFT_RIGHT, fifo_join=PIO.JOIN_TX)
def device_tx():                    #
    jmp(not_osre, "pull")           # if there is data to pull we're in middle of sending a message
    nop()[2]                        # consume 3 cycles in case we wrapped around from TX block to finish the 1 bit delay
    nop().side(1)                   # re/set pin to normal idle state
    label("pull")                   #
    pull()                          # blocking pull to read bits in from upper layer
    jmp(pin, "pre_tx")              # if pin is high this is start of sending a message, do pre_tx
    jmp("tx")                       # else this is continuation of sending a message, go directly to tx

                                    # device uses a ~4ms line low delay to alert receivers a message is coming
    label("pre_tx")                 # this block sets pin low and holds it there for 154 cycles, 4.01417ms
    set(x, 18).side(0)[1]           # set counter, set pin low & delay 1 cycle; consume 2 cycles
    label("pre_tx_loop")            # loop 19 times, consume 152 cycles
    jmp(x_dec, "pre_tx_loop")[7]    # decrement counter, consume 8 cycles

    label("tx")                     # TX 8 data bits & 1 parity bit
    set(x, 8).side(1)[7]            # set loop counter, set pin high for start bit for 8 cycles
    label("tx_loop")                # This loop will run 9 times (8 data bits + 1 parity bit)
    out(pins, 1)                    # Shift 1 bit from OSR to the pin
    jmp(x_dec, "tx_loop")[6]        # Each loop iteration is 8 cycles.
    nop().side(0)[7]                # stop bit 1
    nop()[7]                        # stop bit 2
    nop()[3]                        # 1 bit delay

# device_test_tx() - an 8E2 + 1 bit delay TX tester
#
@asm_pio(sideset_init=PIO.OUT_LOW, out_init=PIO.OUT_LOW, out_shiftdir=PIO.SHIFT_RIGHT, fifo_join=PIO.JOIN_TX)
def device_test_tx():               #
    pull().side(0)                  # Pull or block with line in idle state
    set(x, 8).side(1)[7]            # Preload bit counter, assert start bit for 8 clocks
    label("bitloop")                # This loop will run 9 times (8E2)
    out(pins, 1)                    # Shift 1 bit from OSR to the first OUT pin
    jmp(x_dec, "bitloop")[6]        # Each loop iteration is 8 cycles.
    nop().side(0)[7]                # stop bit 1
    nop().side(0)[7]                # stop bit 2
    nop().side(0)[6]                # 1 bit delay

# device_rx()
#
#                          ___ ___ ___ ___ ___ ___ ___ ___ ___
#       Byte  ____________|   |___|___|___|___|___|___|___|___|___|___|___|
#                 Idle      S   1   2   3   4   5   6   7   8   T   T   D
#
# For receive device uses a modified 8N2 scheme.  While idle the line is low, start
# bit (S) is high and stop bits (T) are low.  Immediately after stop bit 2 there is
# a 1 bit delay (D) where the line remains low.
#
# The 8 data bits (1 - 8) are inverted, device_rx delivers these 8 bits to the upper
# layer inverted.  To protect against situations where a parity bit may be included
# and/or the transmitter is slow device_rx consumes 14 cycles (1.25 bits) to either
# 1) the 2nd cycle of stop bit 2 or 2) the 2nd cycle of the 1 bit delay (D) which
# yields at least 1) 14 or 2) 6 cycles remaining of line low once device_rx returns
# to wait for the start bit (S) of the next incoming byte.  If the next byte does not
# immediately start after the 1 bit delay of the prior byte the wait will be longer
#
# @asm_pio(set_init=PIO.IN_LOW, in_shiftdir=PIO.SHIFT_RIGHT, fifo_join=PIO.JOIN_RX)
# def device_rx():                    #
#    wait(1, pin, 0)                 # Wait for start bit
#    set(x, 7)[10]                   # Preload bit counter, delay until eye of first data bit
#    label("bitloop")                # Loop 8 times
#    in_(pins, 1)                    # Sample 1 bit data, consume 1 cycle
#    jmp(x_dec, "bitloop")[6]        # decrement counter, consume 7 cycles
#    push()
#    nop()[5]                        # this should put us at the eye of stop bit 1 or stop bit 2
#    push()[7]                       # push out the 8 bits we read in, consume 8 cycles
#    nop()[7]                        # consume 8 cycles
#    nop()[3]                        # consume 4 more cycles in case prior bit was parity and/or
# transmitter is slow, loop here bc stop bits & delay are all low

@asm_pio(set_init=PIO.IN_LOW, in_shiftdir=PIO.SHIFT_RIGHT, fifo_join=PIO.JOIN_RX)
def device_rx():                    #
    wait(1, pin, 0)                 # Wait for start bit
    set(x, 7)[9]                    # Preload bit counter, delay until eye of first data bit
    label("bitloop")                # Loop 8 times
    in_(pins, 1)                    # Sample 1 bit data, consume 1 cycle
    jmp(x_dec, "bitloop")[6]        # decrement counter, consume 7 cycles
    push()
    nop()[5]                        # this should put us at the eye of stop bit 1 or stop bit 2
    #push()[7]                      # push out the 8 bits we read in, consume 8 cycles
    nop()[6]                        # consume 8 cycles

#
# end of PIO ASM
#

def _parity(n, p):
    # n = the 8 bit number
    # p = 0 for even parity, 1 for odd parity
    while n > 0:
        p = p ^ (n & 1)
        n = n >> 1
    return p

def _encode_8NN(n):
    # encode an 8-bit numerical value (0 - 255) as an 8-bit numerical value (0 - 255) with
    # reverse endianess
    return _endianess(n)

def _encode_8NI(n):
    # encode an 8-bit numerical value (0 - 255) as an 8-bit numerical value (0 - 255) with
    # reverse endianess & bit inversion
    return (_endianess(n) ^ 0xFF)

def _encode_8EN(n):
    # encode an 8-bit numerical value (0 - 255) as a 9-bit numerical value (0 - 512) with
    # reverse endianess & an even parity bit added in most significant bit
    return ((_parity(n, 0) << 8) + _endianess(n))
    # return ((n << 1) + _parity(n, 0))

def _encode_8ON(n):
    # encode an 8-bit numerical value (0 - 255) as a 9-bit numerical value (0 - 512) with
    # reverse endianess & an odd parity bit added in most significant bit
    return ((_parity(n, 1) << 8) + _endianess(n))
    # return ((n << 1) + _parity(n, 1))

def _encode_8EI(n):
    # encode an 8-bit numerical value (0 - 255) as a 9-bit numerical value (0 - 512) with
    # reverse endianess, an even parity bit added in most significant bit & bit inversion
    return ((_parity(n, 0) << 8) + (_endianess(n) ^ 0xFF))
    # return (((n ^ 0xFF) << 1) + _parity(n, 0))

def _encode_8OI(n):
    # encode an 8-bit numerical value (0 - 255) as a 9-bit numerical value (0 - 512) with
    # reverse endianess, an odd parity bit added in most significant bit & bit inversion
    return ((_parity(n, 1) << 8) + (_endianess(n) ^ 0xFF))
    # return (((n ^ 0xFF) << 1) + _parity(n, 1))

def _decode_8NN(n):
    # decode an 8-bit numerical value (0 - 255) as an 8-bit numerical value (0 - 255) with
    # reverse endianess
    return _endianess(n)

def _decode_8NI(n):
    # decode an 8-bit numerical value (0 - 255) as an 8-bit numerical value (0 - 255) with
    # reverse endianess & bit inversion
    return (_endianess(n) ^ 0xFF)

def _endianess(n):
    # reverse the endianess (bit order) of an 8 bit integer; MSb->LSb, LSb->MSb
    # in normal python this would be done using int('{:08b}'.format(n)[::-1], 2)
    # but in micropython a -1 slicer isn't supported so we do it this way
    #
    # for now we're returning n as it came in just for debugging purposes
    #
    i1 = '{:08b}'.format(n)
    i2 = "%s%s%s%s%s%s%s%s" % (i1[7], i1[6], i1[5], i1[4], i1[3], i1[2], i1[1], i1[0])
    #i2 = i1[7] + i1[6] + i1[5] + i1[4] + i1[3] + i1[2] + i1[1] + i1[0]
    i3 = int(i2, 2)
    return i3

# the global uart_tx_q lock capture
def _capture_uart_tx_q():
    while (_uart_tx_q_lock.locked()):
        #print("*** capture  _uart_tx_q")
        utime.sleep_us(100)
    else:
        _uart_tx_q_lock.acquire()
        #print("captured _uart_tx_q")

# the global uart_tx_q lock release
def _release_uart_tx_q():
    _uart_tx_q_lock.release()
    #print("released _uart_tx_q")

# the global xbus_tx_q lock capture
def _capture_xbus_tx_q():
    while (_xbus_tx_q_lock.locked()):
        #print("*** capture  _xbus_tx_q")
        utime.sleep_us(100)
    else:
        _xbus_tx_q_lock.acquire()
        #print("captured _xbus_tx_q")

# the global xbus_tx_q lock release
def _release_xbus_tx_q():
    _xbus_tx_q_lock.release()
    #print("released _xbus_tx_q")

class _uart_rpi():
    def __init__(self):
        self._run = True
        self._rx_pin = Pin(5, Pin.IN, Pin.PULL_UP)
        self._tx_pin = Pin(4, Pin.OUT)
        self._uart = UART(1, baudrate=115200, bits=8, parity=None, stop=1, tx=self._tx_pin, rx=self._rx_pin, timeout=1)
        self._uart_rx_q = []
        self._uart_rx_q_lock = _thread.allocate_lock()
        self._uart_rx_lock = _thread.allocate_lock()
        self._uart_tx_lock = _thread.allocate_lock()

        global _uart_active
        _uart_active = True

    def _capture_uart_rx(self):
        while (self._uart_rx_lock.locked()):
            #print("*** capture  self._xbus_rx")
            utime.sleep_us(100)
        else:
            self._uart_rx_lock.acquire()
            #print("captured self._uart_rx")

    def _release_uart_rx(self):
        self._uart_rx_lock.release()
        #print("released self._uart_rx")

    def _capture_uart_rx_q(self):
        while (self._uart_rx_q_lock.locked()):
            #print("*** capture  _uart_rx_q")
            utime.sleep_us(100)
        else:
            self._uart_rx_q_lock.acquire()
            #print("captured _uart_rx_q")

    def _release_uart_rx_q(self):
        self._uart_rx_q_lock.release()
        #print("released _uart_rx_q")

    def _capture_uart_tx(self):
        while (self._uart_tx_lock.locked()):
            #print("*** capture  self._xbus_tx")
            utime.sleep_us(100)
        else:
            self._uart_tx_lock.acquire()
            #print("captured self._uart_tx")

    def _release_uart_tx(self):
        self._uart_tx_lock.release()
        #print("released self._uart_tx")

    def read(self):
        global _uart_rx_q
        global _uart_rx_max
        _uart_rx_count = 0
        if (self._run):
            self._capture_uart_rx()
            while ((self._uart.any() > 0) and (_uart_rx_count < _uart_rx_max)):
                _uart_rx_count = _uart_rx_count + 1
                _rx = self._uart.readline()
                if not (_rx is None):
                    self._capture_uart_rx_q()
                    self._uart_rx_q.append(_rx)
                    self._release_uart_rx_q()
            self._release_uart_rx()

    def write(self):
        global _uart_tx_q
        global _uart_tx_max
        _uart_tx_count = 0
        if (self._run):
            _capture_uart_tx_q()
            while ((len(_uart_tx_q) > 0) and (_uart_tx_count < _uart_tx_max)):
                _uart_tx_count = _uart_tx_count + 1
                _tx = _uart_tx_q[0]
                _uart_tx_q.pop(0)
                self._capture_uart_tx()
                self._uart.write(_tx) #this probably needs some conversion before we write it
                self._release_uart_tx()
            _release_uart_tx_q()

    def route(self):
        # route the contents of the uart_rx queue
        global _uart_rx_max
        global _uart_tx_q
        global _xbus_tx_q
        _uart_route_count = 0
        if (self._run):
            self._capture_uart_rx_q()
            while ((len(self._uart_rx_q) > 0) and (_uart_route_count < _uart_rx_max)):
                _uart_route_count = _uart_route_count + 1
                _rx = self._uart_rx_q[0]
                self._uart_rx_q.pop(0)
                if (len(_rx) > 3):
                    if ((_rx[0:4] == 'XBUS') and (_xbus_active)):
                        # route XBUS messages to the xbus_tx queue if the xbus is active and the message is at least
                        # 1 byte in hex
                        _tx = _rx[4:len(_rx)]
                        if (len(_tx) > 1):
                            _capture_xbus_tx_q()
                            _xbus_tx_q.append(_tx)
                            _release_xbus_tx_q()
                            print("_uart.route() sent %s" % (_tx))
                    elif (_rx[0:7] == 'VERSION'):
                        _capture_uart_tx_q()
                        _uart_tx_q.append(_rx + '%s' % (_version))
                        _release_uart_tx_q()
            self._release_uart_rx_q()

    def test(self):
        global _xbus_tx_q
        global _xbus_tx_q_lock
        _test_rx = ['XBUS1009010203040506070809', 'XBUS1109010203040506070809', 'XBUS1209010203040506070809', 'XBUS5009010203040506070809', 'XBUS5109010203040506070809', 'XBUSFFFFFE', 'XBUS520701020304050607', 'XBUS900203040506070809', 'XBUS1F0203040506070809']
        i = 0
        ii = 0
        if (self._run):
            _q_depth = len(self._uart_rx_q)
            while ((_q_depth == 0) and (i < 9)):
                self._capture_uart_rx_q()
                self._uart_rx_q.append(_test_rx[i])
                self._release_uart_rx_q()
                print("_uart.test()  sent %s" % (_test_rx[i]))
                i = i + 1
                utime.sleep_ms(100)
                #if (i > 8):
                    #i = 0
                    #ii = ii + 1

    def stop(self):
        global _uart_active
        self._run = False
        _uart_active = False

class _xbus_device():
    def __init__(self):
        self._pin_A = Pin(2, Pin.OUT)
        self._pin_B = Pin(3, Pin.IN, Pin.PULL_UP)
        self._run = True
        self._rx_read = -1
        self._rx_read_msg = ""
        self._rx_read_f6 = ""
        self._rx_reading = False
        self._rx_reading_f6 = False
        self._reader = StateMachine(0, device_rx, freq=38400, set_base=self._pin_B, in_base=self._pin_B)
        self._writer = StateMachine(1, device_test_tx, freq=38400, sideset_base=self._pin_A, out_base=self._pin_A, jmp_pin=self._pin_A)
        self._read_thread_running = False
        self._poller = None #StateMachine(4, device_poller_tx, freq=38400, sideset_base=self._tx_pin, out_base=self._tx_pin)
        self._xbus_rx_q = []
        self._xbus_rx_q_lock = _thread.allocate_lock()
        self._xbus_rx_lock = _thread.allocate_lock()
        self._xbus_tx_lock = _thread.allocate_lock()

        global _xbus_active

        if not (self._read_thread_running):
            _read_thread = _thread.start_new_thread(self._read_thread,())
        self._writer.active(1)
#        self._poller.active(1)

        _xbus_active = True

    def _capture_xbus_rx(self):
        while (self._xbus_rx_lock.locked()):
            #print("*** capture  self._xbus_rx")
            utime.sleep_us(100)
        else:
            self._xbus_rx_lock.acquire()
            #print("captured self._xbus_rx")

    def _release_xbus_rx(self):
        self._xbus_rx_lock.release()
        #print("released self._xbus_rx")

    def _capture_xbus_rx_q(self):
        while (self._xbus_rx_q_lock.locked()):
            #print("*** capture  self._xbus_rx_q")
            utime.sleep_us(100)
        else:
            self._xbus_rx_q_lock.acquire()
            #print("captured self._xbus_rx_q")

    def _release_xbus_rx_q(self):
        self._xbus_rx_q_lock.release()
        #print("released self._xbus_rx_q")

    def _capture_xbus_tx(self):
        while (self._xbus_tx_lock.locked()):
            print("*** capture  self._xbus_tx")
            utime.sleep_us(100)
        else:
            self._xbus_tx_lock.acquire()
            print("captured self._xbus_tx")

    def _release_xbus_tx(self):
        self._xbus_tx_lock.release()
        print("released self._xbus_tx")

    def read(self):
        # read collected bytes placed into the xbus_rx_q queue by the _read thread into known messages
        # once all bytes for a known message have been collected place the message in the UART TX QUEUE (_uart_tx_q)
        global _uart_tx_q
        global _xbus_rx_max
        _xbus_rx_count = 0
        _xbus_er_count = 0
        print("_xbus.read()  _xbus_rx_q")
        print(self._xbus_rx_q)
        if (self._run):
            while ((len(self._xbus_rx_q) > 0) and (_xbus_rx_count < _xbus_rx_max)):
                self._capture_xbus_rx_q()
                _rx = self._xbus_rx_q[0]
                self._xbus_rx_q.pop(0)
                self._release_xbus_rx_q()
                print("_xbus.read()  while _xbus_rx_q")
                print(self._xbus_rx_q)
                if (type(_rx) == int):
                    # here just in case, must be an int
                    if (0 <= _rx <= 255):
                        # here just in case, must be between 0 & 255 inclusive
                        _rx = _decode_8NI(_rx)
                        _rx_as_hex = "0x{:02X}".format(_rx)[2:4]
                        print("_xbus.read()  read %s" % (_rx_as_hex))
                        if (self._rx_reading):
                            # we are actively reading a series of bytes into a message
                            self._rx_read_msg = self._rx_read_msg + _rx_as_hex
                            if (self._rx_read == -1):
                                # message is variable length, the 2nd byte of the message defines the number of bytes
                                # remaining in the message following the 2nd byte.  This should be the 2nd byte so
                                # set _rx_read to the value of _rx + 1, we +1 it because the next statement
                                # decrements the counter
                                self._rx_read = _rx + 1
                            self._rx_read = self._rx_read - 1
                            if (self._rx_read == 0):
                                # we have read all bytes that are expected for the message type, turn off the
                                # reading flag so we know the next _rx should be the start of a new message and
                                # add the fully read message to the UART TX FIFO QUEUE (_uart_tx_q).  Increment the
                                # message read count
                                self._rx_reading = False
                                _capture_uart_tx_q()
                                _uart_tx_q.append(self._rx_read_msg)
                                _release_uart_tx_q()
                                print("_xbus.read()  sent %s" % (self._rx_read_msg))
                                if (self._rx_reading_f6):
                                    # we have read all bytes that are expected for an F6 response message
                                    # the sending device expects to receive an acknowledgement message which we handle
                                    # locally instead of putting that burden on the app on the RPi
                                    _tx = self._rx_read_f6 + "00"
                                    self._rx_reading_f6 = False
                                    self._rx_read_f6 = ""
                                    _capture_xbus_tx_q()
                                    #_xbus_tx_q.append(_tx) #turned off while we're in loopback testing mode
                                    _release_xbus_tx_q()
                                _xbus_rx_count = _xbus_rx_count + 1
                        else:
                            # we are not currently reading a series of bytes into a message so this byte is the
                            # first byte of a new message or it is garbage.  If the value of the byte is the value
                            # of the first byte of a known message start reading for that known message type.  If the
                            # value of the byte is not the value of the first byte of a known message treat the byte as
                            # an xbus error (XBER) and add an XBER message to the UART TX QUEUE (_uart_tx_q).  Increment
                            # the message read count
                            if (_rx == 255):
                                # FF polling response message
                                # Fixed at 3 bytes, collect 2 more bytes
                                self._rx_read_msg = "XBUS" + _rx_as_hex
                                self._rx_reading = True
                                self._rx_read = 2
                            elif ((_rx == 135) or (_rx == 248)):
                                # 87 & F8 messages
                                # Fixed at 9 bytes, collect 8 more bytes
                                self._rx_read_msg = "XBUS" + _rx_as_hex
                                self._rx_reading = True
                                self._rx_read = 8
                            elif ((16 <= _rx <= 31) or (80 <= _rx <= 95) or (144 <= _rx <= 159) or (208 <= _rx <= 223)):
                                # device response to an F6 message sent by the app
                                #           ___ ___ ___ ___ ___ ___ ___ ___
                                #     Byte |___|___|___|___|___|___|___|___|
                                #            1   2   3   4   5   6   7   8
                                #            S   S       D   D   D   D   D
                                #
                                # In the first byte of an F6 response message bits 1 & 2 represent a message sequence
                                # number and bits 4 - 8 represent the address of the sending device, bit 3 is not used.
                                #
                                # Sequence Number
                                # The sending device retransmits F6 response messages until it receives an F6 response
                                # received acknowledgement from the app.  The first F6 response message includes a
                                # sequence value of 0 (first 2 bits are 00), a second F6 response message increments the
                                # sequence value by 1 (first 2 bits are 01), a third F6 response message increments the
                                # sequence value by 1 (first 2 bits are 10), a fourth F6 response message increments the
                                # sequence value by 1 (first 2 bits are 11) and a fifth F6 response message resets the
                                # sequence value to 0 (first 2 bits are 00) and the pattern continues.
                                #
                                # Device Address
                                # The sending device includes its address (16 - 31) in bits 4 - 8.  Addresses 16 - 23
                                # should be a keypad, addresses 24 - 31 would be other device types.  It is not known
                                # yet if all devices send F6 response messages or these are only sent by keypads.  The
                                # code here receives & processes these from all device addresses.
                                #
                                # F6 response messages are variable length, the length is defined by the 2nd byte of the
                                # F6 response message.  Here we set _rx_collect to -1 so that on the next cycle through
                                # the collector we know we're collecting an F6 response message and the next collected
                                # byte should be the 2nd byte and we are to set _rx_collect to its value.
                                #
                                self._rx_read = -1
                                self._rx_read_msg = "XBUS" + _rx_as_hex
                                self._rx_reading = True
                                self._rx_read_f6 = _rx_as_hex
                                self._rx_reading_f6 = True
                            else:
                                #unknown message type, push it to the app as an XBus ERror
                                self._rx_read_msg = "XBER" + _rx_as_hex
                                _capture_uart_tx_q()
                                _uart_tx_q.append(self._rx_read_msg)
                                _release_uart_tx_q()
                                _xbus_er_count = _xbus_er_count + 1
                                if ((_xbus_er_count % 20) == 0):
                                    # for every 20 error bytes increment _xbus_rx_count by 1
                                    # we want to pull bad bytes out of the queue faster than good bytes i think
                                    _xbus_rx_count = _xbus_rx_count + 1

    def _read_thread(self):
        # read bytes out of the state machine RX FIFO queue
        # add read bytes to the XBUS RX QUEUE (self._xbus_rx_q) for processing by collector
        #
        # this MUST be run as a thread on core 2
        #
        self._reader.active(1)
        self._read_thread_running = True
        while ((self._run) and (self._reader.active())):
            while ((self._run) and (self._reader.rx_fifo() > 0)): # probably should wrap this in try/except
                _rx = self._reader.get() >> 24
                self._capture_xbus_rx_q()
                self._xbus_rx_q.append(_rx)
                self._release_xbus_rx_q()
            else:
                # calculations below are based on PIO SM @ 38400 frequency
                #  1ms =  38.4 PIO SM cycles, at most  4.8 bits & RX FIFO @  6% full
                #  5ms = 192.0 PIO SM cycles, at most 24.0 bits & RX FIFO @ 30% full
                #  6ms = 230.4 PIO SM cycles, at most 28.8 bits & RX FIFO @ 36% full
                #  7ms = 268.8 PIO SM cycles, at most 33.6 bits & RX FIFO @ 42% full
                #  8ms = 307.2 PIO SM cycles, at most 38.4 bits & RX FIFO @ 48% full
                #  9ms = 345.6 PIO SM cycles, at most 43.2 bits & RX FIFO @ 54% full
                # 10ms = 384.0 PIO SM cycles, at most 48.0 bits & RX FIFO @ 60% full
                utime.sleep_us(10)
        self._reader.active(0)
        self._read_thread_running = False
        _thread.exit()

    def timed_events(self):
        #
        # to be written.  it is likely the external loop will pass in a counter or a timestamp
        # and this method will execute activities that need to be performed every n number of cycles, seconds, ms or us
        #
        if (self._run):
            pass
            # trigger the poller state machine every 350ms
            #utime.sleep_ms(350)
            #self._capture_xbus_tx("XBUS_poll")
            #self._tx_lock.acquire()
            #self._poller.put(0) #polling cycle takes 21.796875ms
            #utime.sleep_ms(22) #block TX for 22ms
            #self._release_xbus_tx("XBUS_poll")
            #self._tx_lock.release()

    def write(self):
        # take messages off the _xbus_tx_q
        global _xbus_tx_q
        global _xbus_rx_max
        _xbus_tx_count = 0
        if (self._run):
            while ((len(_xbus_tx_q) > 0) and (_xbus_tx_count < _xbus_rx_max)):
                _capture_xbus_tx_q()
                _txRaw = _xbus_tx_q[0]
                _xbus_tx_q.pop(0)
                _release_xbus_tx_q()
                print("_xbus.write() send %s" % (_txRaw))
                if (_txRaw[0:2] == 'F7'):
                    _txMessage = binascii.unhexlify(_txRaw[00:24]) + bytearray(_txRaw[24:56], 'ascii') + binascii.unhexlify(_txRaw[56:66])
                else:
                    _txMessage = binascii.unhexlify(_txRaw)
                self._capture_xbus_tx()
                for i in range(0, len(_txMessage)):
                    _tx = _encode_8EI(_txMessage[i])
                    while (self._writer.tx_fifo() > 7):
                        #
                        # calculations below are based on PIO SM @ 38400 frequency
                        # the delay before starting to send each message (series of bytes) is
                        # 4.010417ms and it takes 2.708333ms to send each byte of a message
                        # so it takes at least 2.708333ms to empty 1 slot in the 8 slot TX
                        # FIFO once the pre-TX delay has passed.
                        #
                        #  2.708333ms = 1 empty TX FIFO slot, at best
                        #  5.416667ms = 2 empty TX FIFO slots, at best
                        #  8.125000ms = 3 empty TX FIFO slots, at best
                        # 10.833333ms = 4 empty TX FIFO slots, at best
                        # 13.541667ms = 5 empty TX FIFO slots, at best
                        # 16.250000ms = 6 empty TX FIFO slots, at best
                        # 18.958333ms = 7 empty TX FIFO slots, at best
                        # 21.666667ms = 8 empty TX FIFO slots, at best
                        #
                        # we're in the loop for the delivery of a message so if the TX FIFO
                        # has filled up (ie: message is 8+ bytes) it will take at least
                        # 2.708333ms for 1 TX FIFO slot to become available.  For the 9th
                        # byte of a message it will take 6.718750ms for 1 TX FIFO slot to
                        # become available because of the 4.0104167ms pre-TX delay plus the
                        # 2.708333ms to TX the first byte of the message.  For bytes 10+
                        # it will take 2.708333ms for 1 TX FIFO slot to become available.
                        #
                        # sleep for 3ms, enough to clear 1 TX FIFO slot.  9th byte MAY
                        # require we sleep for 3ms twice
                        #
                        print("_xbus.write() queue %d" % (self._writer.tx_fifo()))
                        utime.sleep_us(100)
                    else:
                        print("_xbus.write()   pre %d" % (_tx))
                        self._writer.put(_tx)
                        print("_xbus.write()  post %d" % (_tx))
                self._release_xbus_tx()
                _xbus_tx_count = _xbus_tx_count + 1

    def stop(self):
        global _xbus_active
        self._run = False
        self._writer.active(0)
        # assumption is _read_thread will see ._run = False & stop self._reader SM, set ._read_thread_running = False
        # and itself exit and shut down
        _xbus_active = False

def run():
    global _active
    global _uart
    global _uart_active
    global _xbus
    global _xbus_active
    if not (_uart_active):
        _uart = _uart_rpi()
    if not (_xbus_active):
        _xbus = _xbus_device()
    while (_active):
        if (_xbus_active):
            _xbus.read()
            _xbus.timed_events()
            _xbus.write()
        if (_uart_active):
            #_uart.read()
            _uart.test()
            _uart.route()
            _uart.write()
        utime.sleep_us(500)
    if (_xbus_active):
        _xbus.stop()
    if (_uart_active):
        _uart.stop()