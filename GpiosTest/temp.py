import pwmOrangpiBash
import wiringpi
from wiringpi import GPIO
import time


# Configuración mínima
wiringpi.wiringPiSetup()

pwm0 = pwmOrangpiBash.PWMOrangepi(0)
# Configurar PWM 1
pwm0.setup()
pwm0.configure(period=500000, duty_cycle=50000, enable=True)


# Variables globales (simplificado)
counter = 0
last_state_a = 0

def isr_callback():
    global counter, last_state_a
    state_a = wiringpi.digitalRead(23)
    state_b = wiringpi.digitalRead(25)
    
    if state_a != last_state_a:
        counter += 1 if state_a == state_b else -1
    last_state_a = state_a

# Configurar pines e interrupciones
wiringpi.pinMode(23, GPIO.INPUT)
wiringpi.pinMode(25, GPIO.INPUT)
wiringpi.pullUpDnControl(23, GPIO.PUD_UP)
wiringpi.pullUpDnControl(25, GPIO.PUD_UP)
wiringpi.wiringPiISR(23, GPIO.INT_EDGE_BOTH, isr_callback)
wiringpi.wiringPiISR(25, GPIO.INT_EDGE_BOTH, isr_callback)

# Loop principal
try:
    while True:
        print(f"Counter: {counter}")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Terminado")

