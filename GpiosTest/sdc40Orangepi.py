import smbus2
import time

class SCD40_SMBus:
    def __init__(self, bus_number=2, address=0x62):
        """
        Inicializa el sensor SCD40
        
        Args:
            bus_number: Número del bus I2C (por defecto 2 para Orange Pi)
            address: Dirección I2C del SCD40 (0x62)
        """
        self.address = address
        self.bus = smbus2.SMBus(bus_number)
        self.co2 = None
        self.temperature = None
        self.humidity =None
        print(f"✅ SCD40 inicializado en bus I2C-{bus_number}, dirección 0x{address:02x}")
        #self.start_periodic_measurement()

    def calculate_crc8(self, data):
        """
        Calcula CRC-8 para los datos (requerido por SCD40)
        
        Args:
            data: Lista de bytes [byte1, byte2]
        
        Returns:
            byte: Código CRC de 8 bits
        """
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = crc << 1
                crc &= 0xFF
        return crc
    
    def send_command(self, command):
        """
        Envía un comando al sensor SCD40
        
        Args:
            command: Comando de 16 bits (ej: 0x21B1 para start_measurement)
        """
        # Dividir comando en 2 bytes
        high_byte = (command >> 8) & 0xFF
        low_byte = command & 0xFF
        
        # Calcular CRC
        crc = self.calculate_crc8([high_byte, low_byte])
        
        # Enviar los 3 bytes (comando + CRC)
        data = [high_byte, low_byte, crc]
        self.bus.write_i2c_block_data(self.address, data[0], [data[1], data[2]])
        time.sleep(0.001)
    
    def read_measurement(self):
        """
        Lee medición del sensor SCD40
        
        Returns:
            tuple: (co2, temperature, humidity)
        """
        # Enviar comando de lectura
        self.send_command(0xEC05)
        time.sleep(0.01)  # Espera crítica según datasheet
        
        # Leer 9 bytes de respuesta usando i2c_rdwr para repeated start
        write_msg = smbus2.i2c_msg.write(self.address, [0xEC, 0x05, 0xB6])
        read_msg = smbus2.i2c_msg.read(self.address, 9)
        
        try:
            self.bus.i2c_rdwr(write_msg, read_msg)
        except Exception as e:
            self.close()
            raise Exception(f"Error en lectura I2C: {e}")
        
        # Convertir datos a lista
        data = list(read_msg)
        
        if len(data) != 9:
            raise Exception(f"Datos incompletos: {len(data)}/9 bytes")
        
        print(f"📦 Datos crudos: {[hex(x) for x in data]}")
        
        # Verificar CRCs
        for i in range(0, 9, 3):
            if i + 2 < len(data):
                crc_calc = self.calculate_crc8([data[i], data[i+1]])
                if crc_calc != data[i+2]:
                    print(f"⚠️  CRC error en grupo {i//3}, continuando...")
        
        # Extraer valores
        self.co2 = (data[0] << 8) | data[1]
        temp_raw = (data[3] << 8) | data[4]
        hum_raw = (data[6] << 8) | data[7]
        
        # Convertir a valores reales (fórmulas del datasheet Sensirion)
        self.temperature = -45 + 175 * (temp_raw / 65536.0)
        self.humidity = 100 * (hum_raw / 65536.0)
        print("=" * 30)
        print(f"CO₂: {self.co2} ppm")
        print(f"Temperatura: {self.temperature:.2f} °C")
        print(f"Humedad: {self.humidity:.2f} %")
        """
        # Indicador de calidad del aire
        if self.co2 < 800:
            calidad = "✅ Excelente"
        elif self.co2 < 1200:
            calidad = "⚠️  Bueno"
        elif self.co2 < 2000:
            calidad = "⚠️  Regular"
        else:
            calidad = "❌ Malo - ¡Ventilar!"
        print(f"Calidad del aire: {calidad}")
        print("=" * 30)
        #return co2, temperature, humidity
        """

    def start_periodic_measurement(self):
        """Inicia mediciones periódicas"""
        print("🎬 Iniciando mediciones periódicas...")
        self.send_command(0x21B1)
        print("⏳ Esperando 30 segundos para primera medición...")
        for i in range(30, 0, -1):
            print(f"{i}...", end=" ", flush=True)
            time.sleep(1)
        print()
    
    def stop_periodic_measurement(self):
        """Detiene mediciones periódicas"""
        print("⏹️ Deteniendo mediciones...")
        self.send_command(0x3F86)
    
    def close(self):
        """Cierra la conexión I2C"""
        self.bus.close()


"""
def main():
    print("=== SCD40 con SMBus2 - Mediciones de CO2, Temperatura y Humedad ===")
    
    sensor = None
    try:
        # 1. Inicializar sensor
        sensor = SCD40_SMBus(bus_number=2)  # Usar bus 2 para Orange Pi
        
        # 2. Soft reset
        #sensor.soft_reset()
        time.sleep(2)
        
        # 3. Iniciar mediciones
        sensor.start_periodic_measurement()
        
        # 4. Espera inicial CRÍTICA (30 segundos para primera medición)
        print("⏳ Esperando 30 segundos para primera medición...")
        for i in range(30, 0, -1):
            print(f"{i}...", end=" ", flush=True)
            time.sleep(1)
        print()
        
        # 5. Bucle de lecturas
        print("\n=== INICIANDO LECTURAS ===")
        lectura_count = 0
        
        while lectura_count < 10:  # Limitar a 10 lecturas para prueba
            try:
                co2, temp, hum = sensor.read_measurement()
                
                print(f"\n📊 LECTURA {lectura_count + 1}")
                print("=" * 30)
                print(f"CO₂: {co2} ppm")
                print(f"Temperatura: {temp:.2f} °C")
                print(f"Humedad: {hum:.2f} %")
                
                # Indicador de calidad del aire
                if co2 < 800:
                    calidad = "✅ Excelente"
                elif co2 < 1200:
                    calidad = "⚠️  Bueno"
                elif co2 < 2000:
                    calidad = "⚠️  Regular"
                else:
                    calidad = "❌ Malo - ¡Ventilar!"
                print(f"Calidad del aire: {calidad}")
                print("=" * 30)
                
                lectura_count += 1
                time.sleep(5)  # Esperar 5 segundos entre lecturas
                
            except Exception as e:
                print(f"❌ Error en lectura: {e}")
                print("Reintentando en 5 segundos...")
                time.sleep(5)
    
    except KeyboardInterrupt:
        print("\n🛑 Programa interrumpido por el usuario")
    except Exception as e:
        print(f"💥 Error general: {e}")
    finally:
        if sensor:
            sensor.stop_periodic_measurement()
            sensor.close()
        print("🔚 Programa terminado")

"""