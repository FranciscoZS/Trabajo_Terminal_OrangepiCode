import wiringpi
import time
import sys

# Configuración MPU-6050
MPU_ADDR = 0x68

def main():
    # Inicializar wiringpi (igual que tu ejemplo)
    wiringpi.wiringPiSetup()
    
    # Usar /dev/i2c-1 que es común en Orange Pi 5 Max
    fd = wiringpi.wiringPiI2CSetupInterface("/dev/i2c-1", MPU_ADDR)
    
    if fd < 0:
        print("Error: No se pudo inicializar I2C")
        sys.exit(1)
    
    # Despertar el sensor (registro 0x6B = 0)
    if wiringpi.wiringPiI2CWriteReg8(fd, 0x6B, 0) < 0:
        print("Error al despertar sensor")
        sys.exit(1)
    
    # Calibrar giroscopio
    print("Calibrando...")
    offset_sum = 0
    for i in range(100):
        high = wiringpi.wiringPiI2CReadReg8(fd, 0x47)  # GYRO_ZOUT_H
        low = wiringpi.wiringPiI2CReadReg8(fd, 0x48)   # GYRO_ZOUT_L
        
        if high < 0 or low < 0:
            print("Error lectura I2C")
            continue
            
        raw = (high << 8) | low
        if raw > 32767: 
            raw -= 65536
        offset_sum += raw
        time.sleep(0.01)

    offset_z = offset_sum / 100
    angle_z = 0.0
    last_time = time.time()

    print("Leyendo ángulo Z...")
    
    try:
        while True:
            # Leer giroscopio Z (igual estructura que tu función read_register)
            high = wiringpi.wiringPiI2CReadReg8(fd, 0x47)
            low = wiringpi.wiringPiI2CReadReg8(fd, 0x48)
            
            if high < 0 or low < 0:
                print("Error lectura")
                time.sleep(0.1)
                continue
            
            # Convertir a valor signed (complemento a 2)
            raw = (high << 8) | low
            if raw > 32767: 
                raw -= 65536
            
            # Calcular velocidad angular (°/s) - factor 131 para ±250°/s
            velocity_z = (raw - offset_z) / 131.0
            
            # Integrar para obtener ángulo
            current_time = time.time()
            dt = current_time - last_time
            angle_z += velocity_z * dt
            last_time = current_time
            
            print(f"Ángulo Z: {angle_z:7.2f}° | Velocidad: {velocity_z:6.2f} °/s")
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nPrograma detenido")
        sys.exit(0)

if __name__ == '__main__':
    main()