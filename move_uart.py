import serial
import time
import threading
from enum import Enum

class Movement(Enum):
    STOP = 0
    FORWARD = 1
    BACKWARD = 2
    LEFT = 3
    RIGHT = 4
    CLOCKWISE = 5
    COUNTER_CLOCKWISE = 6
    DIAGONAL_FR = 7
    DIAGONAL_FL = 8
    DIAGONAL_BR = 9
    DIAGONAL_BL = 10
    DEG90_LEFT = 11
    DEG90_RIGHT = 12

class Direction:
    FORWARD = 1
    BACKWARD = 0

class OmnidirectionalRobot:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(3)
            print(f"✅ Conectado al robot en {port}")
            
            self.last_response = ""
            self._start_reader()
            time.sleep(2)
            
        except Exception as e:
            print(f"❌ Error conectando con el robot: {e}")
    
    def _start_reader(self):
        def reader():
            while True:
                try:
                    if self.ser.in_waiting > 0:
                        response = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if response:
                            print(f"ROBOT: {response}")
                            self.last_response = response
                except Exception as e:
                    if "Bad file descriptor" not in str(e):
                        print(f"Error leyendo serial: {e}")
                    time.sleep(0.1)
        
        thread = threading.Thread(target=reader, daemon=True)
        thread.start()
    
    def send_command(self, command):
        try:
            full_command = f"{command}\n"
            self.ser.write(full_command.encode())
            self.ser.flush()
            time.sleep(0.3)
            return True
        except Exception as e:
            print(f"❌ Error enviando comando: {e}")
            return False
    
    def config_motor(self, motor_index, pwm_pin, dir_pin, frequency=6000):
        """Configurar un motor específico con 1 pin de dirección"""
        command = f"CONFIG_MOTOR,{motor_index},{pwm_pin},{dir_pin},{frequency}"
        return self.send_command(command)
    
    def set_motor(self, motor_index, direction, duty):
        """Control individual de motor"""
        command = f"SET_MOTOR,{motor_index},{direction},{duty}"
        return self.send_command(command)
    
    def move(self, movement_type, speed=50):
        """Ejecutar movimiento predefinido"""
        if isinstance(movement_type, Movement):
            movement_type = movement_type.value
        command = f"MOVE,{movement_type},{speed}"
        return self.send_command(command)
    
    def stop(self):
        """Detener todos los motores"""
        return self.send_command("STOP")
    
    def get_status(self):
        """Obtener estado del robot"""
        return self.send_command("STATUS")
    
    def close(self):
        """Cerrar conexión"""
        try:
            self.stop()
            time.sleep(1)
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.close()
            print("✅ Conexión cerrada correctamente")
        except Exception as e:
            print(f"❌ Error cerrando: {e}")

# Configuración y pruebas del robot
def setup_robot():
    """Configurar el robot con 4 motores (1 pin dirección cada uno)"""
    robot = OmnidirectionalRobot('/dev/ttyUSB0', 115200)
    time.sleep(2)
    
    print("🛠️ Configurando motores (1 pin dirección)...")
    
    # Configurar 4 motores (PWM pin, DIR pin)
    motor_configs = [
        (2, 3, 6000),   # Motor 0: PWM=2, DIR=3 dir no invertida
        (7, 6, 6000),   # Motor 1: PWM=7, DIR=6 dir  invertida
        (8, 5, 6000),   # Motor 2: PWM=8, DIR=5 dir no invertida
        (9, 4, 6000)  # Motor 3: PWM=9, DIR=4 dir invertida
    ]
    
    for i, config in enumerate(motor_configs):
        print(f"Configurando motor {i}: PWM{config[0]}, DIR{config[1]}")
        robot.config_motor(i, config[0], config[1], config[2])
        time.sleep(1)
    
    return robot

def test_individual_motors(robot):
    """Probar control individual de motores"""
    print("\n🎯 Probando control individual de motores...")
    
    # Probar cada motor individualmente
    for i in range(4):
        print(f"Motor {i} - Adelante 50%")
        robot.set_motor(i, Direction.FORWARD, 50)
        time.sleep(2)
        robot.set_motor(i, Direction.FORWARD, 0)  # Duty 0 para parar
        time.sleep(1)
        
        print(f"Motor {i} - Atrás 30%")
        robot.set_motor(i, Direction.BACKWARD, 30)
        time.sleep(2)
        robot.set_motor(i, Direction.BACKWARD, 0)  # Duty 0 para parar
        time.sleep(1)

def test_movements(robot):
    """Probar movimientos predefinidos"""
    print("\n🎯 Probando movimientos predefinidos...")
    
    movements = [
        (Movement.FORWARD, "Adelante"),
        (Movement.BACKWARD, "Atrás"),
        (Movement.LEFT, "Izquierda"),
        (Movement.RIGHT, "Derecha"),
        (Movement.CLOCKWISE, "Giro horario"),
        (Movement.COUNTER_CLOCKWISE, "Giro antihorario"),
        (Movement.DIAGONAL_FR, "Diagonal frontal derecha"),
        (Movement.DIAGONAL_FL, "Diagonal frontal izquierda"),
        (Movement.DEG90_LEFT, "90° izquierda")
    ]
    
    for movement, description in movements:
        print(f"Movimiento: {description}")
        robot.move(movement, 60)
        time.sleep(3)
        robot.stop()
        time.sleep(1)

def interactive_control(robot):
    """Control interactivo del robot"""
    print("\n🎮 MODO CONTROL INTERACTIVO")
    print("Comandos:")
    print("  w: Adelante    s: Atrás")
    print("  a: Izquierda   d: Derecha")
    print("  q: Giro CCW    e: Giro CW")
    print("  1: Diagonal FL 2: Diagonal FR")
    print("  3: Diagonal BL 4: Diagonal BR")
    print("  5: 90° izquierda 6: 90° derecha")
    print("  espacio: Parar")
    print("  i: Estado")
    print("  x: Salir")
    
    try:
        while True:
            key = input("Presiona una tecla: ").lower()
            
            if key == 'w':
                robot.move(Movement.FORWARD, 70)
            elif key == 's':
                robot.move(Movement.BACKWARD, 70)
            elif key == 'a':
                robot.move(Movement.LEFT, 70)
            elif key == 'd':
                robot.move(Movement.RIGHT, 70)
            elif key == 'q':
                robot.move(Movement.COUNTER_CLOCKWISE, 60)
            elif key == 'e':
                robot.move(Movement.CLOCKWISE, 60)
            elif key == '1':
                robot.move(Movement.DIAGONAL_FL, 50)
            elif key == '2':
                robot.move(Movement.DIAGONAL_FR, 50)
            elif key == '3':
                robot.move(Movement.DIAGONAL_BL, 50)
            elif key == '4':
                robot.move(Movement.DIAGONAL_BR, 50)
            elif key == '5':
                robot.move(Movement.DEG90_LEFT, 50)
            elif key == '6':
                robot.move(Movement.DEG90_RIGHT, 50)
            elif key == ' ':
                robot.stop()
            elif key == 'i':
                robot.get_status()
            elif key == 'x':
                break
            else:
                print("Tecla no reconocida")
                
    except KeyboardInterrupt:
        print("\nSaliendo del control interactivo...")

def main():
    """Función principal"""
    print("🤖 SISTEMA DE CONTROL DE ROBOT OMNIDIRECCIONAL")
    print("CONFIGURACIÓN: 1 PIN DE DIRECCIÓN POR MOTOR")
    print("=" * 50)
    
    robot = setup_robot()
    time.sleep(2)
    
    # Mostrar estado inicial
    robot.get_status()
    time.sleep(2)
    
    while True:
        print("\n🔧 MENU PRINCIPAL")
        print("1. Probar motores individualmente")
        print("2. Probar movimientos predefinidos")
        print("3. Control interactivo")
        print("4. Estado del robot")
        print("5. Salir")
        
        choice = input("Selecciona opción: ").strip()
        
        if choice == "1":
            test_individual_motors(robot)
        elif choice == "2":
            test_movements(robot)
        elif choice == "3":
            interactive_control(robot)
        elif choice == "4":
            robot.get_status()
        elif choice == "5":
            break
        else:
            print("Opción no válida")
    
    print("Apagando robot...")
    robot.stop()
    time.sleep(1)
    robot.close()

if __name__ == "__main__":
    main()