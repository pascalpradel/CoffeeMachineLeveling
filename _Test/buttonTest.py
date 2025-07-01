from machine import Pin
import time

# Pin 14 als Eingang mit Pull-up-Widerstand
button = Pin(4, Pin.IN, Pin.PULL_UP)

while True:
    if button.value() == 0:  # 0 = gedrückt (weil gegen GND geschaltet)
        print("Button gedrückt!")
    else:
        print("Button nicht gedrückt.")
    time.sleep(0.2)
