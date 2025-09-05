import cv2
from ultralytics import YOLO

# Cargar modelo
model = YOLO(r"/home/orangepi/TestTT/TestCameraAndYolo/best_ncnn_model")  # Asegúrate de tener OpenCV y PyTorch en Jetson

# Captura de video (cámara o stream)
frame = cv2.imread(r"/home/orangepi/TestTT/resource/image/testclassroom.jpeg")
#f,c,_ = frame.shape
#print(f,c)
#frame = cv2.resize(frame, (640, 640))

# Detección
results = model(frame, conf=0.5)  # Umbral de confianza 50%
for box in results[0].boxes:
    x1, y1, x2, y2 = map(int, box.xyxy[0])  # Coordenadas del bounding box
    cls = int(box.cls[0])                   # Clase detectada
    label = model.names[cls]                # Nombre de la clase (ej: "silla")
    confidence = float(box.conf[0])         # Confianza de la detección

    # Dibujar el bounding box y etiqueta
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(frame, f"{label} {confidence:.2f}", (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

# 5. Mostrar la imagen con las detecciones
#cv2.imwrite('debug_frame.jpg', frame)
frame = cv2.resize(frame, (1920,1080))
cv2.imshow("test",frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
