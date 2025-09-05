import cv2
import argparse
import numpy as np
import time
from ultralytics import YOLO
from TauLidarCommon.frame import FrameType
from TauLidarCamera.camera import Camera
import threading
import queue

# Configuración de prioridades
YOLO_FRAME_QUEUE_MAXSIZE = 2
LIDAR_FRAME_QUEUE_MAXSIZE = 2

class YOLOThread(threading.Thread):
    def __init__(self, frame_queue, result_queue):
        super().__init__()
        self.frame_queue = frame_queue
        self.result_queue = result_queue
        self.running = True
        self.model = YOLO(r"/home/orangepi/TestTT/TestCameraAndYolo/best_ncnn_model")
        self.daemon = True

    def run(self):
        print("Hilo YOLO iniciado")
        while self.running:
            try:
                # Obtener frame de la cola con timeout
                frame = self.frame_queue.get(timeout=0.1)
                
                # Procesar detecciones YOLO
                results = self.model(frame)
                
                # Procesar cada detección
                detections = []
                for result in results:
                    for box in result.boxes:
                        # Obtener coordenadas del bounding box
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        
                        # Obtener clase y confianza
                        cls = int(box.cls[0])
                        confidence = float(box.conf[0])
                        label = self.model.names[cls]
                        
                        detections.append({
                            'coords': (x1, y1, x2, y2),
                            'label': label,
                            'confidence': confidence
                        })
                
                # Enviar resultados al proceso principal
                if detections:
                    self.result_queue.put(detections)
                    
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error en hilo YOLO: {e}")
                continue
    
    def stop(self):
        self.running = False

class LiDARThread(threading.Thread):
    def __init__(self, camera_port, lidar_queue):
        super().__init__()
        self.camera_port = camera_port
        self.lidar_queue = lidar_queue
        self.running = True
        self.daemon = True
        self.camera = None

    def run(self):
        print("Hilo LiDAR iniciado")
        try:
            # Configurar y abrir la cámara LiDAR
            self.camera = Camera.open(self.camera_port)
            if self.camera:
                self.camera.setModulationChannel(0)
                self.camera.setIntegrationTime3d(0, 1000)
                self.camera.setMinimalAmplitude(0, 10)
                Camera.setRange(0, 4500)
                print("Cámara LiDAR configurada en hilo")
        except Exception as e:
            print(f"Error abriendo cámara LiDAR: {e}")
            return
        
        try:
            while self.running:
                # Leer frame de la cámara de profundidad
                frame_depth = self.camera.readFrame(FrameType.DISTANCE)

                if frame_depth:
                    mat_depth_rgb = np.frombuffer(frame_depth.data_depth_rgb, dtype=np.uint16, count=-1, offset=0).reshape(frame_depth.height, frame_depth.width, 3)
                    mat_depth_rgb = mat_depth_rgb.astype(np.uint8)

                    # Escalar la imagen
                    upscale = 4
                    img_depth = cv2.resize(mat_depth_rgb, (frame_depth.width * upscale, frame_depth.height * upscale))
                    img_depth = cv2.flip(img_depth, 1)
                    
                    # Poner en cola para el proceso principal
                    if self.lidar_queue.qsize() < LIDAR_FRAME_QUEUE_MAXSIZE:
                        self.lidar_queue.put(img_depth)
                
                time.sleep(0.01)  # Pequeña pausa
                
        except Exception as e:
            print(f"Error en hilo LiDAR: {e}")
        finally:
            if self.camera:
                self.camera.close()
    
    def stop(self):
        self.running = False
        if self.camera:
            self.camera.close()

def setup(serialPort=None):
    port = None
    camera = None
    if serialPort is None:
        ports = Camera.scan()
        if len(ports) > 0:
            port = ports[0]
    else:
        port = serialPort
                                                                                             
    if port is not None:                                                                     
        Camera.setRange(0, 4500)                                                                                        
                                                                                             
        camera = Camera.open(port)        
        camera.setModulationChannel(0)
        camera.setIntegrationTime3d(0, 1000)
        camera.setMinimalAmplitude(0, 10)
                                                                                             
        cameraInfo = camera.info()                                                           
                                                                                             
        print("\nToF camera opened successfully:")                                           
        print("    model:      %s" % cameraInfo.model)                                       
        print("    firmware:   %s" % cameraInfo.firmware)                                    
        print("    uid:        %s" % cameraInfo.uid)                                         
        print("    resolution: %s" % cameraInfo.resolution)                                  
        print("    port:       %s" % cameraInfo.port)                                        
                                                                                             
        print("\nPress Esc key over GUI or Ctrl-c in terminal to shutdown ...")              
                                                                                             
    return camera, port

def run_with_threads(camera_port):
    print("Iniciando procesamiento con threading...")
    print("Presiona ESC para salir")
    
    # Crear colas para comunicación entre hilos
    yolo_frame_queue = queue.Queue(maxsize=YOLO_FRAME_QUEUE_MAXSIZE)
    yolo_result_queue = queue.Queue()
    lidar_queue = queue.Queue(maxsize=LIDAR_FRAME_QUEUE_MAXSIZE)
    
    # Inicializar cámara normal
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    # Crear e iniciar hilos
    yolo_thread = YOLOThread(yolo_frame_queue, yolo_result_queue)
    lidar_thread = LiDARThread(camera_port, lidar_queue)
    
    yolo_thread.start()
    lidar_thread.start()
    
    # Variables para control de FPS
    last_time = time.time()
    frame_count = 0
    fps = 0
    
    try:
        while True:
            # Leer frame de la cámara normal
            ret, frame_normal = cap.read()
            
            if ret:
                frame_normal = cv2.flip(frame_normal, 1)
                frame_normal = cv2.flip(frame_normal, 0)
                
                # Enviar frame a YOLO si la cola no está llena
                if yolo_frame_queue.qsize() < YOLO_FRAME_QUEUE_MAXSIZE:
                    try:
                        yolo_frame_queue.put_nowait(frame_normal.copy())
                    except queue.Full:
                        pass  # La cola está llena, omitir este frame
                
                # Procesar resultados de YOLO si hay disponibles
                yolo_frame = frame_normal.copy()
                detections_count = 0
                
                try:
                    while not yolo_result_queue.empty():
                        detections = yolo_result_queue.get_nowait()
                        for result in detections:
                            coords = result['coords']
                            label = result['label']
                            confidence = result['confidence']
                            detections_count += 1
                            
                            # Dibujar rectángulo
                            cv2.rectangle(yolo_frame, (coords[0], coords[1]), (coords[2], coords[3]), (0, 255, 0), 2)
                            
                            # Crear texto de la etiqueta
                            label_text = f"{label} {confidence:.2f}"
                            
                            # Dibujar fondo para el texto
                            (text_width, text_height), baseline = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                            cv2.rectangle(yolo_frame, (coords[0], coords[1] - text_height - 5), 
                                         (coords[0] + text_width, coords[1]), (0, 255, 0), -1)
                            
                            # Dibujar texto
                            cv2.putText(yolo_frame, label_text, (coords[0], coords[1] - 5), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                except queue.Empty:
                    pass
                
                # Calcular y mostrar FPS
                frame_count += 1
                if time.time() - last_time >= 1.0:
                    fps = frame_count
                    frame_count = 0
                    last_time = time.time()
                
                cv2.putText(yolo_frame, f"FPS: {fps} | Detections: {detections_count}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Mostrar frame con detecciones
                cv2.imshow("YOLO Detection", yolo_frame)
            
            # Procesar frame LiDAR si disponible
            try:
                img_depth = lidar_queue.get_nowait()
                cv2.imshow('Depth Map', img_depth)
            except queue.Empty:
                pass
            
            # Salir con tecla ESC
            if cv2.waitKey(1) == 27: 
                break
                
    except Exception as e:
        print(f"Error en proceso principal: {e}")
    
    finally:
        # Detener hilos
        yolo_thread.stop()
        lidar_thread.stop()
        
        # Esperar a que los hilos terminen
        yolo_thread.join(timeout=2.0)
        lidar_thread.join(timeout=2.0)
        
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Programa que integra YOLO con cámara normal y Tau LiDAR Camera')
    parser.add_argument('--port', metavar='<serial port>', default=None,
                        help='Especificar puerto serial para la Tau Camera')
    args = parser.parse_args()

    camera_obj = None
    try:
        camera_obj, camera_port = setup(args.port)
        if camera_obj:
            # Ejecutar con threading
            run_with_threads(camera_port)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print('\nCerrando programa...')
        cv2.destroyAllWindows()
        if camera_obj:
            camera_obj.close()