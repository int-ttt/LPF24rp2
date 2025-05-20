from machine import Pin, UART
from rp2     import asm_pio, PIO, StateMachine

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

@asm_pio(sideset_init = PIO.OUT_HIGH,
         out_init     = PIO.OUT_HIGH,
         out_shiftdir = PIO.SHIFT_RIGHT,
         fifo_join    = PIO.JOIN_TX)
def PioTx8N1():
    pull()            .side(1)  [7]
    set(x, 7)         .side(0)  [7]
    label("txloop")
    out(pins, 1)
    jmp(x_dec, "txloop")        [6]

RX_BAUD = 2400

# --- Added for debugging below



tx0Pin = Pin(2, Pin.OUT, value=1)

tx0 = StateMachine(1, PioTx8N1, freq=RX_BAUD * 8, sideset_base=tx0Pin, out_base=tx0Pin)

tx0.active(1)



rx0Pin = Pin(3, Pin.IN, Pin.PULL_UP)

rx0 = StateMachine(0, PioRx8N1, freq=RX_BAUD * 8, in_base=rx0Pin)

rx0.active(1)

def Write(N, txN, c):
    b = c
    print("TX{}  >  {}".format(N, b))
    txN.put(b)

def Read(N, rxN):
    while rxN.rx_fifo():
      b = (rxN.get() >> 24).to_bytes()
      print("RX{}  < {}".format(N, b))

import time
time.sleep(1)
print("")
Write(0, tx0, b'\x00')
time.sleep(1)
Read(0, rx0)
Write(0, tx0, b'\x04')
time.sleep(1)
Read(0, rx0)
Write(0, tx0, b'\xf0')
time.sleep(1)
Read(0, rx0)
Write(0, tx0, b'@')
time.sleep(1)
Read(0, rx0)