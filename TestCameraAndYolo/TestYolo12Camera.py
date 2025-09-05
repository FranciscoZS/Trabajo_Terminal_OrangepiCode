import cv2
from ultralytics import YOLO

# Cargar modelo
model = YOLO(r"/home/orangepi/TestTT/TestCameraAndYolo/best_ncnn_model")

# Inicializar la cámara (0 para cámara por defecto)
cap = cv2.VideoCapture(0)

# Configurar resolución de la cámara
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Verificar que la cámara se abrió correctamente
if not cap.isOpened():
    print("Error: No se pudo abrir la cámara")
    exit()

print("Presiona 'q' para salir...")

# Crear la ventana una sola vez fuera del bucle
cv2.namedWindow("Detección en tiempo real - YOLO", cv2.WINDOW_NORMAL)

try:
    while True:
        # Capturar frame por frame
        ret, frame = cap.read()
        frame = cv2.flip(frame,1)
        frame = cv2.flip(frame,0)
        
        # Verificar si se capturó el frame correctamente
        if not ret:
            print("Error: No se pudo capturar el frame")
            break
        
        # Realizar la detección con YOLO
        results = model(frame)
        
        # Procesar las detecciones
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Coordenadas del bounding box
            cls = int(box.cls[0])                   # Clase detectada
            label = model.names[cls]                # Nombre de la clase
            confidence = float(box.conf[0])         # Confianza de la detección

            # Dibujar el bounding box y etiqueta
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {confidence:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        
        # Mostrar el frame en la misma ventana
        cv2.imshow("Detección en tiempo real - YOLO", frame)
        
        # Salir con la tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Liberar recursos
    cap.release()
    cv2.destroyAllWindows()
