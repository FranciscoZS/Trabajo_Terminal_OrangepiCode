import cv2
import numpy as np
import mediapipe as mpipe



mp_face_detection = mpipe.solutions.face_detection
mp_drawing = mpipe.solutions.drawing_utils



with mp_face_detection.FaceDetection(
    model_selection=0, min_detection_confidence=0.2) as face_detection:
    # Aqui se "lee" la imagen
    image = cv2.imread(r"D:\clanTreviAndrade\pythonProject\img\facetest.jpg")
    # Cambiar de BGR a RGB
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # Detección de rostros
    results = face_detection.process(image)
    print(f"detección: {results.detections}")

    #dibujar los resultados
    if results.detections:
      for detection in results.detections:
        mp_drawing.draw_detection(image, detection, #imagne, detecion
                                  mp_drawing.DrawingSpec(color=(255,0,255), #color de la linea
                                thickness=10, circle_radius=20)) #anchura, radio
    cv2.imshow("prueba",cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    cv2.waitKey(0)