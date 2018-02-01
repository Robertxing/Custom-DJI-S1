#Physical Computing on Raspberry Pi

#turning LED on and off (blink like Arduino)

from gpiozero import LED
from time import sleep
led = LED(17)

while True:
  led.on()
  sleep(1)
  led.off()
  sleep(1)
  
  #
