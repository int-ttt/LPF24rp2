# from machine import Pin, UART
# from rp2 import PIO, StateMachine, asm_pio
# import  utime
#
# UART_BAUD = 115200
# PIN_BASE = 6
# NUM_UARTS = 1
#
#
# @asm_pio(sideset_init=PIO.OUT_HIGH, out_init=PIO.OUT_HIGH, out_shiftdir=PIO.SHIFT_RIGHT)
# def uart_tx():
#     # fmt: off
#     # Block with TX deasserted until data available
#     pull()
#     # Initialise bit counter, assert start bit for 8 cycles
#     set(x, 7)  .side(0)       [7]
#     # Shift out 8 data bits, 8 execution cycles per bit
#     label("bitloop")
#     out(pins, 1)              [6]
#     jmp(x_dec, "bitloop")
#     # Assert stop bit for 8 cycles total (incl 1 for pull())
#     nop()      .side(1)       [6]
#     # fmt: on
#
#
# uart = UART(1, UART_BAUD, rx=Pin(5))
# # Now we add 8 UART TXs, on pins 10 to 17. Use the same baud rate for all of them.
# sm = StateMachine(
#     0, uart_tx, freq=8 * UART_BAUD, sideset_base=Pin(PIN_BASE), out_base=Pin(PIN_BASE)
# )
# sm.active(1)
#
#
# # We can print characters from each UART by pushing them to the TX FIFO
# def pio_uart_print(sm, s):
#     for c in s:
#         sm.put(ord(c))
#
# # Print a different message from each UART
# pio_uart_print(sm, b"\xaa\xff\x11\x0d\x04\xfd".format(1))
# utime.sleep_ms(500)
# print(uart.any(), uart.read(uart.any()))

from lego import LPF2
from machine import Pin
import utime

led = Pin(25, Pin.OUT)

modes = [
    LPF2.mode("TOF", size=4),
    LPF2.mode("TEMP", size=4)
]
led.on()
utime.sleep_ms(100)
led.off()

# print(modes)

lpf2 = LPF2.LPF2(modes)
lpf2.init()

value = 0
while True:
    if not lpf2.connected:
        led.toggle()
        utime.sleep_ms(5)
        lpf2.init()

    else:
        led.on()

        if value < 9:
            value = value + 1
        else:
            value = 0

        # lpf2.load_payload('Int8',value)
        print(lpf2.connected)

        utime.sleep_ms(200)
