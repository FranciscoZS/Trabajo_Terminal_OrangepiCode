import pwmOrangpiBash
import encoderOrangepi
import wiringpi
from wiringpi import GPIO
import time

#wiringpi.wiringPiSetup() #puedes omitir esta linea 

dir = 22
canal_a = 23
canal_b = 25
ppr = 1000

#encoder = encoderOrangepi.OpticalEncoder(canal_a,canal_a,ppr)
encoder = encoderOrangepi.OpticalEncoder()
pulse = pwmOrangpiBash.PWMOrangepi(0) #activamos pwm para el pin wPi 1  fisico 5
pulse.setup()

#encoder.setup_pins()
wiringpi.pinMode(dir, GPIO.OUTPUT)
wiringpi.digitalWrite(dir,GPIO.LOW)


pulse.configurePWM(2000,50,True)

try:
    while True:
        print(f"Counter: {encoder.counter}")
        time.sleep(0.1)
except KeyboardInterrupt:
    pulse.cleanup()
    print("Terminado")