
import pwmOrangpiBash
import time

pulse = pwmOrangpiBash.PWMOrangepi(0) #activamos pwm para el pin wPi 1  fisico 5
pulse.setup()
pulse.configurePWM(2000,50,True)

# Loop principal
try:
    while True:
        #print(pulse.duty_cycle)
        time.sleep(1)
except KeyboardInterrupt:
    print("Terminado")
    pulse.cleanup()



"""
import encoderOrangepi
#import wiringpi
#from wiringpi import GPIO
import time

#wiringpi.wiringPiSetup() #puedes omitir esta linea 

dir = 22
canal_a = 23
canal_b = 25
ppr = 1000

#encoder = encoderOrangepi.OpticalEncoder(canal_a,canal_a,ppr)
encoder = encoderOrangepi.OpticalEncoder()


#encoder.setup_pins()
#wiringpi.pinMode(dir, GPIO.OUTPUT)
#wiringpi.digitalWrite(dir,GPIO.LOW)


try:
    while True:
        print(f"Counter: {encoder.counter}")
except KeyboardInterrupt:
    print("Terminado")

"""