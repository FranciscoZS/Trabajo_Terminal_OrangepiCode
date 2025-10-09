import wiringpi
import time

class SimpleRGBPCA9685:
    def __init__(self, bus=2):
        self.fd = wiringpi.wiringPiI2CSetupInterface(f"/dev/i2c-{bus}", 0x40)
        if self.fd < 0: raise IOError("Error I2C")
        
        # Configurar PCA9685 a 1000Hz para LEDs
        wiringpi.wiringPiI2CWriteReg8(self.fd, 0x00, 0x10)
        time.sleep(0.005)
        prescale = int(25000000.0 / (4096.0 * 1000) + 0.5) - 1
        wiringpi.wiringPiI2CWriteReg8(self.fd, 0xFE, prescale)
        time.sleep(0.005)
        wiringpi.wiringPiI2CWriteReg8(self.fd, 0x00, 0x00)
        time.sleep(0.005)
    
    def set_color(self, r_chan, g_chan, b_chan, r, g, b):
        """Establece color RGB (valores 0-100)"""
        def set_chan(chan, val):
            pwm = int(val * 40.95)  # Convertir 0-100 a 0-4095
            base = 0x06 + 4 * chan
            wiringpi.wiringPiI2CWriteReg8(self.fd, base + 2, pwm & 0xFF)
            wiringpi.wiringPiI2CWriteReg8(self.fd, base + 3, pwm >> 8)
        
        set_chan(r_chan, r)
        set_chan(g_chan, g)
        set_chan(b_chan, b)

# Uso inmediato
rgb = SimpleRGBPCA9685()

# Canales: 0=Rojo, 1=Verde, 2=Azul (ajusta según tu conexión)

for i in range(101):

    rgb.set_color(0, 1, 2, i, i, i)   # Rojo
    time.sleep(1)
#rgb.set_color(0, 1, 2, 0, 100, 0)   # Verde
#rgb.set_color(0, 1, 2, 0, 0, 100)   # Azul