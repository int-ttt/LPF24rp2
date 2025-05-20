#Source: Electrocredible.com, Language: MicroPython.
from machine import Pin,UART
import time
uart = UART(1, baudrate=9600, tx=Pin(8), rx=Pin(9))
uart.init(bits=8, parity=None, stop=2)
led = Pin("LED", Pin.OUT)

while True:
    print(uart.write('t'))
    if uart.any():
        data = uart.read(1)
        if data== b'm':
            led.toggle()
    time.sleep(1)


# arduino code
"""
bool ledState=1; //variable used to save the state of LED
void setup() {
  Serial.begin(9600);// set baud rate to 9600
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() { 
  if(Serial.read()== 't') {
  digitalWrite(LED_BUILTIN, ledState);
  ledState=!ledState;
  Serial.print('m'); //write 'm' to the UART
  } 
}
"""