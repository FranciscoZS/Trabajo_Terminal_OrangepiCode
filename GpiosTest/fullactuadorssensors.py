import pwmOrangpiBash
import encoderOrangepi
import time
import subprocess
import sdc40Orangepi
import mpuOrangepi


mpu = mpuOrangepi.MPU()
sdc40 = sdc40Orangepi.SCD40_SMBus()
#sdc40Orangepi.main()
time.sleep(2)
sdc40.start_periodic_measurement()

encoder_0 = encoderOrangepi.OpticalEncoder(pin_a=4,pin_b=6,ppr=1000)
encoder_1 = encoderOrangepi.OpticalEncoder(pin_a=9,pin_b=10,ppr=1000)
encoder_2 = encoderOrangepi.OpticalEncoder(pin_a=13,pin_b=15,ppr=1000)
encoder_3 = encoderOrangepi.OpticalEncoder(pin_a=16,pin_b=18,ppr=1000)

wheel_0 = pwmOrangpiBash.PWMOrangepi(chip_number=2,pwm_number=0,pin_dir=21,dir=0) #activamos pwm para el pin wPi 1  fisico 5
wheel_1 = pwmOrangpiBash.PWMOrangepi(chip_number=4,pwm_number=0,pin_dir=24,dir=0)
wheel_2 = pwmOrangpiBash.PWMOrangepi(chip_number=5,pwm_number=0,pin_dir=26,dir=0)
wheel_3 = pwmOrangpiBash.PWMOrangepi(chip_number=6,pwm_number=0,pin_dir=27,dir=0)

wheel_0.setup()
wheel_1.setup()
wheel_2.setup()
wheel_3.setup()

wheel_0.configurePWM(2000,50,True)
wheel_1.configurePWM(2000,50,True)
wheel_2.configurePWM(2000,50,True)
wheel_3.configurePWM(2000,50,True)


try:
    while True:
        print(f"wheel0 = {wheel_0.duty_cycle}")
        print(f"wheel1 = {wheel_1.duty_cycle}")
        print(f"wheel2 = {wheel_2.duty_cycle}")
        print(f"wheel3 = {wheel_3.duty_cycle}")
        mpu.readMPU()
        sdc40.read_measurement()
        time.sleep(5)
except KeyboardInterrupt:
    subprocess.run("gpio readall",shell=True)
    wheel_0.cleanup()
    wheel_1.cleanup()
    wheel_2.cleanup()
    wheel_3.cleanup()
    sdc40.stop_periodic_measurement()
    sdc40.close()
    print("Terminado")
    subprocess.run("gpio readall",shell=True)
