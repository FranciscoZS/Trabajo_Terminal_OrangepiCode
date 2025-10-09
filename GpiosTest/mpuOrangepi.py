import wiringpi
import time

wiringpi.wiringPiSetup()

class MPU:
    def __init__(self,addres=0x68,i2cbus=2):
        """
        Inicializa el mpu
        addres: direccion que detecta la orangepi con el comando
        bash i2cdetect -y #Numero de bus
        i2cbus: bus de elecion de la orange pi 
        los buses disponibles se enlistan con el comando
        ls /dev/i2c-*
        en este caso son: 0,1,2,3,6,7,9,10
        estos buses se habilitan desde orangepi-config en bash
        consultar manual de usuario 
        """
        self.addres = addres
        self.i2cbus = i2cbus

        self.fd = wiringpi.wiringPiI2CSetupInterface(f"/dev/i2c-{self.i2cbus}", self.addres)
        if self.fd < 0:
            print(f"Error I2C no se pudo conectar al bus {self.i2cbus}, o la direaccón {self.addres} es incorrecta")
            return False
        

        # 1. Reset primero
        wiringpi.wiringPiI2CWriteReg8(self.fd, 0x6B, 0x80)  # Reset
        time.sleep(0.1)

        # 2. Despertar
        wiringpi.wiringPiI2CWriteReg8(self.fd, 0x6B, 0x00)  # Wake up
        time.sleep(0.1)

    def readMPU(self):

        """
        checar datasheet del mpu6050 para mas informacion sobre los registros 
        """
        gz_h = wiringpi.wiringPiI2CReadReg8(self.fd, 0x47)
        gz_l = wiringpi.wiringPiI2CReadReg8(self.fd, 0x48)
        gz_raw = (gz_h << 8) | gz_l
        if gz_raw > 32767: gz_raw -= 65536
        print(f"GZ: {gz_raw/131.0:.1f}°/s")
        time.sleep(0.1)

