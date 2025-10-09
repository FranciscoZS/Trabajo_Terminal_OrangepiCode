import wiringpi
import time

# Configuración directa
MPU_ADDR = 0x68

wiringpi.wiringPiSetup()
fd = wiringpi.wiringPiI2CSetupInterface("/dev/i2c-2", MPU_ADDR)

if fd < 0:
    print("Error I2C")
    exit(1)

# Intentar método más agresivo de despertar
print("Despertando sensor...")

# 1. Reset primero
wiringpi.wiringPiI2CWriteReg8(fd, 0x6B, 0x80)  # Reset
time.sleep(0.1)

# 2. Despertar
wiringpi.wiringPiI2CWriteReg8(fd, 0x6B, 0x00)  # Wake up
time.sleep(0.1)



# 3. Verificar si responde
try:
    # Leer registro que siempre debería tener valor conocido
    whoami = wiringpi.wiringPiI2CReadReg8(fd, 0x75)
    print(f"WHO_AM_I: {whoami:#x}")
    
    if whoami in [0x68, 0x70]:
        print("✅ Sensor listo")

        # Lectura simple del giroscopio Z
        while True:
            gz_h = wiringpi.wiringPiI2CReadReg8(fd, 0x47)
            gz_l = wiringpi.wiringPiI2CReadReg8(fd, 0x48)
            gz_raw = (gz_h << 8) | gz_l
            if gz_raw > 32767: gz_raw -= 65536
            print(f"GZ: {gz_raw/131.0:.1f}°/s")
            time.sleep(0.1)
            
except Exception as e:
    print(f"Error: {e}")
