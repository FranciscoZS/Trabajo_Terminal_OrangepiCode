import sdc40Orangepi
import mpuOrangepi
import time

mpu = mpuOrangepi.MPU()
sdc40 = sdc40Orangepi.SCD40_SMBus()
#sdc40Orangepi.main()
time.sleep(2)
sdc40.start_periodic_measurement()

try:
    while True:
        mpu.readMPU()
        sdc40.read_measurement()
        time.sleep(5)
except KeyboardInterrupt:
    sdc40.stop_periodic_measurement()
    sdc40.close()
    print("Terminado")