import wiringpi
from wiringpi import GPIO
import time
import pwmtest as pwm

# Configuración mínima
wiringpi.wiringPiSetup()

def direction(dir,pulse):
    #direcion
    wiringpi.pinMode(dir, GPIO.OUTPUT)
    wiringpi.digitalWrite(dir,GPIO.LOW)
    pwm.pwmsetup(pulse)
    pwm.pwmdutty(pulse,0)

direction(24,21)

# Loop principal
try:
    while True:
        pwm.pwmdutty(21,100)
        time.sleep(0.115)
        pwm.pwmdutty(21,0)
        time.sleep(0.004)
except KeyboardInterrupt:
    print("Terminado")