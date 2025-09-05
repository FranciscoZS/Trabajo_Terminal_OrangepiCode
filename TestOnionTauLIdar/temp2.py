import argparse
import numpy as np
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Para crear visualizaciones 3D en matplotlib

from TauLidarCommon.frame import FrameType
from TauLidarCamera.camera import Camera

def setup(serialPort=None):
    port = None
    camera = None

    # Si no se especifica un puerto serial, escanear los dispositivos de cámara Tau disponibles
    if serialPort is None:
        ports = Camera.scan()  # Escanear dispositivos
        if len(ports) > 0:
            port = ports[0]
    else:
        port = serialPort

    if port is not None:
        Camera.setRange(0, 4500)  # Establecer el rango de distancia

        camera = Camera.open(port)  # Abrir la cámara Tau disponible
        camera.setModulationChannel(0)  # Habilitar canal de modulación
        camera.setIntegrationTime3d(0, 1000)  # Tiempo de integración
        camera.setMinimalAmplitude(0, 10)  # Amplitud mínima

        cameraInfo = camera.info()

        print("\nToF camera opened successfully:")
        print(f"    model:      {cameraInfo.model}")
        print(f"    firmware:   {cameraInfo.firmware}")
        print(f"    uid:        {cameraInfo.uid}")
        print(f"    resolution: {cameraInfo.resolution}")
        print(f"    port:       {cameraInfo.port}")

        print("\nPress Esc key over GUI or Ctrl-c in terminal to shutdown ...")

        cv2.namedWindow('Depth Map')
        cv2.moveWindow('Depth Map', 20, 20)

    return camera

# Función para generar y actualizar la visualización de la nube de puntos
def plot_point_cloud(points_3d, ax):
    if points_3d is None or len(points_3d) == 0:
        return
    
    points_3d = np.array(points_3d)

    # Separar las coordenadas XYZ
    x = points_3d[:, 0]
    y = points_3d[:, 1]
    z = points_3d[:, 2]

    # Limpiar el gráfico
    ax.cla()

    # Crear la nube de puntos en 3D
    ax.scatter(x, y, z, c=z, cmap='cool', marker='o', s=1)

    # Configurar límites de los ejes y etiquetas
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim([np.min(x), np.max(x)])
    ax.set_ylim([np.min(y), np.max(y)])
    ax.set_zlim([np.min(z), np.max(z)])

    plt.draw()
    plt.pause(0.01)  # Pequeña pausa para actualizar la visualización

def run(camera):
    # Inicializar la figura de matplotlib
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')  # Crear un gráfico 3D

    while True:
        frame = camera.readFrame(FrameType.DISTANCE_AMPLITUDE)

        if frame:
            # Obtener el mapa de profundidad y los puntos 3D
            mat_depth_map = np.frombuffer(frame.data_depth, dtype=np.float32, count=-1, offset=0).reshape(frame.height, frame.width)

            # Upscaling de la imagen de profundidad (opcional)
            upscale = 4
            depth_img = cv2.resize(mat_depth_map, (frame.width * upscale, frame.height * upscale))

            cv2.imshow('Depth Map', depth_img)

            # Obtener los puntos 3D
            points_3d = frame.points_3d

            # Generar y visualizar la nube de puntos
            if points_3d:
                plot_point_cloud(points_3d, ax)

            # Salir con la tecla ESC
            if cv2.waitKey(1) == 27:
                plt.close()  # Cerrar la ventana de matplotlib
                break

                

def cleanup(camera):
    print('\nShutting down ...')
    cv2.destroyAllWindows()
    camera.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Programa de ejemplo para adquirir datos de una cámara Tau LiDAR')
    parser.add_argument('--port', metavar='<serial port>', default=None,
                        help='Especificar un puerto serial para la cámara Tau')
    args = parser.parse_args()

    camera = setup('COM6')

    if camera:
        try:
            run(camera)
        except Exception as e:
            print(e)

        cleanup(camera)
