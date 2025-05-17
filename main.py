from lego import LPF2
from machine import Pin
import gc,utime
import micropython

led = Pin("LED", Pin.OUT)

modes = [
     LPF2.mode("int8", type=LPF2.DATA8),
     LPF2.mode("int8", type=LPF2.DATA8)
]
led.on()
utime.sleep(5)
led.off()

print(modes)

lpf2 = LPF2.LPF2()
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

          lpf2.loadPayload('Int8',value)
          print(value)

          utime.sleep_ms(200)
