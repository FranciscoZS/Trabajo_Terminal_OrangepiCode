# encoder_reader_1000ppr.py
import wiringpi
from wiringpi import GPIO
import time

wiringpi.wiringPiSetup()

class OpticalEncoder:
    def __init__(self, pin_a=23, pin_b=25, ppr=1000):
        """
        Inicializa el encoder óptico para 1000 PPR
        
        Args:
            pin_a (int): Pin GPIO para el canal A
            pin_b (int): Pin GPIO para el canal B  
            ppr (int): Pulsos por revolución del encoder (1000)
        """
        self.pin_a = pin_a #Pin que aparece en la columna Wpi de gpio readall
        self.pin_b = pin_b
        self.ppr = ppr  # 1000 pulsos por revolución
        self.counter = 0
        self.last_state_a = 0
        self.rpm = 0
        self.last_time = time.time()
        self.last_counter = 0  # Para cálculo de RPM
        self.total_revolutions = 0  # Revoluciones totales
        
        # Configuración de wiringpi
        #wiringpi.wiringPiSetup()
        
        #Configurar pines
        self.setup_pins()
        
    def setup_pins(self):
        """Configura los pines GPIO e interrupciones"""
        wiringpi.pinMode(self.pin_a, GPIO.INPUT)
        wiringpi.pinMode(self.pin_b, GPIO.INPUT)
        wiringpi.pullUpDnControl(self.pin_a, GPIO.PUD_UP)
        wiringpi.pullUpDnControl(self.pin_b, GPIO.PUD_UP)
        
        # Configurar interrupciones
        wiringpi.wiringPiISR(self.pin_a, GPIO.INT_EDGE_BOTH, self._isr_callback)
        wiringpi.wiringPiISR(self.pin_b, GPIO.INT_EDGE_BOTH, self._isr_callback)
        
    def _isr_callback(self):
        """Callback para las interrupciones del encoder"""
        state_a = wiringpi.digitalRead(self.pin_a)
        state_b = wiringpi.digitalRead(self.pin_b)
        
        #if state_a != self.last_state_a:
        #    if state_a == state_b:
        #        self.counter += 1
        #    else:
        #        self.counter -= 1
        if state_a != self.last_state_a:
            self.counter += 1 if state_a == state_b else -1
        self.last_state_a = state_a
        
    
    def calculate_rpm(self):
        """
        Calcula las RPM basado en el cambio del contador
        Para 1000 PPR: 1000 pulsos = 1 revolución
        """
        current_time = time.time()
        time_elapsed = current_time - self.last_time
        
        # Calcular RPM solo si ha pasado suficiente tiempo
        #if time_elapsed >= 0.1:  # Mínimo 100ms entre cálculos
        # Calcular pulsos desde la última medición
        pulses = self.counter - self.last_counter
        
        # Calcular revoluciones (1000 pulsos = 1 revolución)
        revolutions = pulses / self.ppr  # pulses / 1000
        
        # Calcular RPM: (revoluciones / tiempo_en_minutos)
        # revoluciones / (time_elapsed / 60) = (revoluciones * 60) / time_elapsed
        self.rpm = (revolutions * 60) / time_elapsed
        
        # Actualizar revoluciones totales
        self.total_revolutions += abs(revolutions)
        
        # Actualizar para próxima medición
        self.last_counter = self.counter
        self.last_time = current_time
            
        return self.rpm
    
    def get_rpm(self):
        """Retorna el valor actual de RPM"""
        return self.calculate_rpm()
    
    def get_counter(self):
        """Retorna el valor actual del contador de pulsos"""
        return self.counter
    
    def get_revolutions(self):
        """Retorna el número total de revoluciones"""
        return self.total_revolutions
    
    def get_revolutions_since_last(self):
        """Retorna las revoluciones desde la última lectura de RPM"""
        pulses = self.counter - self.last_counter
        return pulses / self.ppr
    
    def reset_counter(self):
        """Resetea el contador de pulsos a cero"""
        self.counter = 0
        self.last_counter = 0
    
    def reset_revolutions(self):
        """Resetea el contador de revoluciones totales a cero"""
        self.total_revolutions = 0

    def cleanup(self):
        """Limpia los recursos"""
        pass