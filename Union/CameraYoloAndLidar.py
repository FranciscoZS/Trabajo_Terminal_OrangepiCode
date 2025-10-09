import cv2
import argparse
import numpy as np
from ultralytics import YOLO
from TauLidarCommon.frame import FrameType
from TauLidarCamera.camera import Camera

# Inicializar cámara normal
cap = cv2.VideoCapture(0)

# Cargar modelo YOLO
model = YOLO(r"/home/orangepi/Trabajo_Terminal_OrangepiCode/TestCameraAndYolo/best_ncnn_model")

def setup(serialPort=None):
    port = None
    camera = None
    if serialPort is None:
        ports = Camera.scan()  # Escanear dispositivos Tau Camera disponibles
        if len(ports) > 0:
            port = ports[0]
    else:
        port = serialPort
                                                                                             
    if port is not None:                                                                     
        Camera.setRange(0, 4500)  # Puntos en el rango de distancia a colorear                                                                                        
                                                                                             
        camera = Camera.open(port)  # Abrir la primera Tau Camera disponible        
        camera.setModulationChannel(0)  # autoChannelEnabled: 0, channel: 0      
        camera.setIntegrationTime3d(0, 1000)  # set integration time 0: 1000           
        camera.setMinimalAmplitude(0, 10)  # set minimal amplitude 0: 80            
                                                                                             
        cameraInfo = camera.info()                                                           
                                                                                             
        print("\nToF camera opened successfully:")                                           
        print("    model:      %s" % cameraInfo.model)                                       
        print("    firmware:   %s" % cameraInfo.firmware)                                    
        print("    uid:        %s" % cameraInfo.uid)                                         
        print("    resolution: %s" % cameraInfo.resolution)                                  
        print("    port:       %s" % cameraInfo.port)                                        
                                                                                             
        print("\nPress Esc key over GUI or Ctrl-c in terminal to shutdown ...")              
                                                                                             
    return camera 

def process_yolo_detection(frame):
    """
    Procesa detecciones YOLO en el frame de la cámara normal
    """
    frame = cv2.flip(frame,1)
    frame = cv2.flip(frame,0)

    results = model(frame)
    
    # Procesar cada detección
    for result in results:
        for box in result.boxes:
            # Obtener coordenadas del bounding box
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            
            # Obtener clase y confianza
            cls = int(box.cls[0])
            confidence = float(box.conf[0])
            label = model.names[cls]  # Usar los nombres de clases del modelo entrenado
            
            # Dibujar rectángulo
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Crear texto de la etiqueta
            label_text = f"{label} {confidence:.2f}"
            
            # Dibujar fondo para el texto
            (text_width, text_height), baseline = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (x1, y1 - text_height - 5), (x1 + text_width, y1), (0, 255, 0), -1)
            
            # Dibujar texto
            cv2.putText(frame, label_text, (x1, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    
    return frame

def run(camera):
    print("Iniciando procesamiento...")
    print("Presiona ESC para salir")
    
    while True:
        # Leer frame de la cámara de profundidad
        frame_depth = camera.readFrame(FrameType.DISTANCE)

        if frame_depth:
            mat_depth_rgb = np.frombuffer(frame_depth.data_depth_rgb, dtype=np.uint16, count=-1, offset=0).reshape(frame_depth.height, frame_depth.width, 3)
            mat_depth_rgb = mat_depth_rgb.astype(np.uint8)

            # Escalar la imagen
            upscale = 4
            img_depth = cv2.resize(mat_depth_rgb, (frame_depth.width * upscale, frame_depth.height * upscale))
            img_depth = cv2.flip(img_depth, 1)
            cv2.imshow('Depth Map', img_depth)
        
        # Leer frame de la cámara normal
        ret, frame_normal = cap.read()
        
        if ret:
            
            # Procesar detecciones YOLO
            frame_with_detections = process_yolo_detection(frame_normal)
            
            # Mostrar frame con detecciones
            cv2.imshow("YOLO Detection", frame_with_detections)

        # Salir con tecla ESC
        if cv2.waitKey(1) == 27: 
            break

def cleanup(camera):
    print('\nCerrando programa...')
    cv2.destroyAllWindows()
    cap.release()
    if camera:
        camera.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Programa que integra YOLO con cámara normal y Tau LiDAR Camera')
    parser.add_argument('--port', metavar='<serial port>', default=None,
                        help='Especificar puerto serial para la Tau Camera')
    args = parser.parse_args()

    camera = None
    try:
        camera = setup(args.port)
        if camera:
            run(camera)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cleanup(camera)
