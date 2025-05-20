from machine import Pin
from rp2     import asm_pio, PIO, StateMachine
import machine

@asm_pio(set_init     = PIO.IN_HIGH,
         in_shiftdir  = PIO.SHIFT_RIGHT,
         fifo_join    = PIO.JOIN_RX)
def PioRx8N1():
    wait( 0, pin, 0)
    set(x, 7)                   [10]
    label("rxloop")
    in_(pins, 1)
    jmp(x_dec, "rxloop")        [6]
    push()

@asm_pio(sideset_init=PIO.OUT_HIGH, out_init=PIO.OUT_HIGH, out_shiftdir=PIO.SHIFT_RIGHT)
def uart_tx():
    # fmt: off
    # Block with TX deasserted until data available
    pull()
    # Initialise bit counter, assert start bit for 8 cycles
    set(x, 7)  .side(0)       [7]
    # Shift out 8 data bits, 8 execution cycles per bit
    label("bitloop")
    out(pins, 1)              [6]
    jmp(x_dec, "bitloop")
    # Assert stop bit for 8 cycles total (incl 1 for pull())
    nop()      .side(1)       [6]
    # fmt: on
num = 0

class UART:
    def __init__(self, id, tx = None, rx = None, baudrate=115200, bits=8, parity=None, stop=1):
        global num
        self.tx = tx
        self.rx = rx
        self.tx_num = num
        self.baud = baudrate
        self.id = id
        num += 1
        self.txState = StateMachine(self.tx_num, uart_tx, freq = baudrate * 8, sideset_base = 5, out_base = 5)
        print(id, baudrate, self.rx)
        self.rxState = machine.UART(id, baudrate, rx = self.rx)
        self.txState.active(1)

    def init(self, tx=None, rx=None, baudrate=115200, bits=8, parity=None, stop=1):
        global num
        self.txState = StateMachine(self.tx_num, uart_tx, freq=baudrate, sideset_base=5, out_base=5)
        self.rxState.init(baudrate=2400, bits=8, parity=None, stop=1)
        self.txState.active(1)


    def deinit(self):
        self.rxState = None
        self.txState = None

    def any(self):
        return self.rxState.any()

    def write(self, data):
        self.txState.put(data)

    def read(self, nbytes):
        return self.rxState.read(nbytes)