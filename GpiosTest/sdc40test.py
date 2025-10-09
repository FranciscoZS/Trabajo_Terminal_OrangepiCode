import wiringpi
import time
import struct

# Configuración del sensor SCD40
SCD4X_DEFAULT_ADDR = 0x62
I2C_BUS = "/dev/i2c-2"

class SCD40:
    def __init__(self, i2c_bus=I2C_BUS, address=SCD4X_DEFAULT_ADDR):
        self.address = address
        print(f"Intentando conectar a SCD40 en {i2c_bus}, dirección 0x{address:02x}")
        
        self.fd = wiringpi.wiringPiI2CSetupInterface(i2c_bus, address)
        
        if self.fd < 0:
            raise Exception(f"Error al inicializar I2C. File descriptor: {self.fd}")
        
        print(f"✅ I2C inicializado correctamente (fd: {self.fd})")
        
        # Configurar I2C para mayor velocidad (opcional)
        wiringpi.wiringPiI2CSetup(self.fd)  # Re-inicializar con configuración por defecto
        
        # Espera para estabilización
        time.sleep(1)
    
    def calculate_crc8(self, data):
        """Calcula CRC-8 para los datos"""
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
        """Envía un comando de 2 bytes al sensor - MÉTODO CORREGIDO"""
        # Convertir comando a bytes
        cmd_high = (command >> 8) & 0xFF
        cmd_low = command & 0xFF
        
        # Calcular CRC para los 2 bytes del comando
        crc = self.calculate_crc8([cmd_high, cmd_low])
        
        print(f"Enviando comando: 0x{command:04x} -> [{cmd_high:02x}, {cmd_low:02x}, crc:{crc:02x}]")
        
        # Enviar los 3 bytes seguidos usando escritura directa
        data = [cmd_high, cmd_low, crc]
        
        # Usar wiringPiI2CWriteReg8 para enviar bytes individuales
        for i, byte in enumerate(data):
            result = wiringpi.wiringPiI2CWriteReg8(self.fd, i, byte)
            if result == -1:
                raise Exception(f"Error escribiendo byte {i} del comando")
        
        time.sleep(0.001)  # Espera mínima después del comando
    
    def read_bytes(self, num_bytes):
        """Lee bytes del sensor - MÉTODO CORREGIDO"""
        data = []
        for i in range(num_bytes):
            try:
                # Leer byte individual
                byte = wiringpi.wiringPiI2CReadReg8(self.fd, 0)
                if byte == -1:
                    raise Exception("Error de lectura I2C")
                data.append(byte)
                time.sleep(0.0001)  # Pequeña espera entre lecturas
            except Exception as e:
                raise Exception(f"Error leyendo byte {i}: {e}")
        return data
    
    def read_data(self, num_bytes):
        """Lee datos del sensor con verificación CRC - MÉTODO CORREGIDO"""
        if num_bytes % 3 != 0:
            raise Exception("El número de bytes debe ser múltiplo de 3")
        
        # Leer todos los bytes primero
        raw_data = self.read_bytes(num_bytes)
        print(f"Bytes crudos leídos: {[hex(x) for x in raw_data]}")
        
        data = []
        # Procesar en grupos de 3 bytes (2 datos + 1 CRC)
        for i in range(0, len(raw_data), 3):
            if i + 2 >= len(raw_data):
                break
                
            byte1 = raw_data[i]
            byte2 = raw_data[i + 1]
            crc_received = raw_data[i + 2]
            
            # Verificar CRC
            crc_calculated = self.calculate_crc8([byte1, byte2])
            
            print(f"Grupo {i//3}: bytes=[{byte1:02x}, {byte2:02x}], CRC_recibido={crc_received:02x}, CRC_calculado={crc_calculated:02x}")
            
            if crc_calculated != crc_received:
                raise Exception(f"Error CRC en grupo {i//3}: esperado {crc_calculated:02x}, recibido {crc_received:02x}")
            
            data.extend([byte1, byte2])
        
        return data
    
    def soft_reset(self):
        """Realiza un soft reset del sensor - MÉTODO CORREGIDO"""
        print("Realizando soft reset...")
        try:
            # Enviar comando de soft reset (0x0006)
            reset_cmd = 0x0006
            self.send_command(reset_cmd)
            time.sleep(1.0)  # Espera larga después del reset
            print("Soft reset completado")
        except Exception as e:
            print(f"Error en soft reset: {e}")
    
    def start_periodic_measurement(self):
        """Inicia mediciones periódicas"""
        print("Iniciando mediciones periódicas...")
        self.send_command(0x21B1)
        time.sleep(1)
        print("Mediciones periódicas iniciadas")
    
    def read_measurement(self):
        """Lee medición de CO2, temperatura y humedad - MÉTODO CORREGIDO"""
        print("Solicitando lectura de medición...")
        
        # Primero verificar si hay datos disponibles
        try:
            self.send_command(0xE4B8)  # Comando get_data_ready_status
            time.sleep(0.01)
            
            # Leer estado (3 bytes: 2 datos + 1 CRC)
            status_data = self.read_data(3)
            if len(status_data) >= 2:
                status = (status_data[0] << 8) | status_data[1]
                data_ready = (status & 0x07FF) != 0
                print(f"Estado de datos listos: {data_ready} (status: 0x{status:04x})")
                
                if not data_ready:
                    raise Exception("Datos no listos, esperando...")
        except Exception as e:
            print(f"Advertencia al verificar estado: {e}")
        
        # Leer medición
        self.send_command(0xEC05)  # Comando read_measurement
        time.sleep(0.05)  # Espera más larga para la lectura
        
        # Leer 9 bytes (3 valores × 3 bytes cada uno)
        data = self.read_data(9)
        
        if len(data) != 6:
            raise Exception(f"Datos incompletos. Esperados 6 bytes, obtenidos {len(data)}")
        
        # Convertir bytes a valores
        co2_raw = (data[0] << 8) | data[1]
        temp_raw = (data[2] << 8) | data[3]
        hum_raw = (data[4] << 8) | data[5]
        
        # Convertir a valores reales (fórmulas del datasheet)
        co2 = co2_raw
        temperature = -45 + 175 * (temp_raw / 65536.0)
        humidity = 100 * (hum_raw / 65536.0)
        
        print(f"Valores crudos - CO2: {co2_raw}, Temp: {temp_raw}, Hum: {hum_raw}")
        
        return co2, temperature, humidity
    
    def get_serial_number(self):
        """Obtiene número de serie del sensor - MÉTODO SIMPLIFICADO"""
        print("Solicitando número de serie...")
        
        try:
            self.send_command(0x3682)  # Comando get_serial_number
            time.sleep(0.05)  # Espera más larga
            
            # Leer 9 bytes (3 words + CRC)
            data = self.read_data(9)
            
            if len(data) >= 6:
                serial = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
                return f"{serial:08X}"
            else:
                raise Exception("Datos insuficientes para número de serie")
                
        except Exception as e:
            print(f"Error obteniendo serial: {e}")
            return "Desconocido"
    
    def check_sensor_presence(self):
        """Verifica si el sensor está presente - MÉTODO SIMPLIFICADO"""
        try:
            # Intentar un comando simple que no debería fallar
            self.send_command(0x3682)  # Get serial number
            time.sleep(0.01)
            return True
        except Exception as e:
            print(f"Error verificando presencia: {e}")
            return False

def main():
    try:
        print("=== INICIALIZANDO SCD40 ===")
        
        # Intentar conectar al sensor
        try:
            sensor = SCD40(i2c_bus="/dev/i2c-2")
            print("✅ Conexión I2C establecida")
        except Exception as e:
            print(f"❌ Error de conexión: {e}")
            return
        
        # Verificar presencia del sensor
        if sensor.check_sensor_presence():
            print("✅ Sensor detectado")
        else:
            print("❌ Sensor no responde, intentando reset...")
            sensor.soft_reset()
            time.sleep(2)
        
        # Obtener número de serie (método más robusto)
        try:
            serial = sensor.get_serial_number()
            print(f"✅ Número de serie: {serial}")
        except Exception as e:
            print(f"⚠️  No se pudo obtener número de serie: {e}")
        
        # Iniciar mediciones
        print("Iniciando mediciones...")
        sensor.start_periodic_measurement()
        
        print("Esperando 10 segundos para primera medición...")
        time.sleep(10)
        
        # Intentar lecturas
        max_attempts = 5
        for attempt in range(max_attempts):
            try:
                print(f"\n--- Intento de lectura {attempt + 1}/{max_attempts} ---")
                co2, temp, hum = sensor.read_measurement()
                
                print("\n🎉 ¡LECTURA EXITOSA!")
                print(f"CO₂: {co2} ppm")
                print(f"Temperatura: {temp:.2f} °C")
                print(f"Humedad: {hum:.2f} %")
                
                # Si llegamos aquí, salir del loop
                break
                
            except Exception as e:
                print(f"❌ Error en intento {attempt + 1}: {e}")
                
                if attempt < max_attempts - 1:
                    print(f"Reintentando en 3 segundos...")
                    time.sleep(3)
                else:
                    print("❌ No se pudieron obtener lecturas después de todos los intentos")
        
        print("\n--- Finalizado ---")
        
    except KeyboardInterrupt:
        print("\n⏹️  Programa interrumpido por el usuario")
    except Exception as e:
        print(f"💥 Error general: {e}")

if __name__ == "__main__":
    main()