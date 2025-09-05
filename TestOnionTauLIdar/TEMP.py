import argparse
import numpy as np
import cv2
import time
import open3d as o3d

from TauLidarCommon.frame import FrameType
from TauLidarCamera.camera import Camera

def setup(serialPort=None):
    port = None
    camera = None
    # if no serial port is specified, scan for available Tau Camera devices
    if serialPort is None:
        ports = Camera.scan()                      ## Scan for available Tau Camera devices

        if len(ports) > 0:
            port = ports[0]
    else:
        port = serialPort

    if port is not None:
        Camera.setRange(0, 4500)                   ## points in the distance range to be colored

        camera = Camera.open(port)             ## Open the first available Tau Camera
        camera.setModulationChannel(0)             ## autoChannelEnabled: 0, channel: 0
        camera.setIntegrationTime3d(0, 1000)       ## set integration time 0: 1000
        camera.setMinimalAmplitude(0, 10)          ## set minimal amplitude 0: 80

        cameraInfo = camera.info()

        print("\nToF camera opened successfully:")
        print("    model:      %s" % cameraInfo.model)
        print("    firmware:   %s" % cameraInfo.firmware)
        print("    uid:        %s" % cameraInfo.uid)
        print("    resolution: %s" % cameraInfo.resolution)
        print("    port:       %s" % cameraInfo.port)

        print("\nPress Esc key over GUI or Ctrl-c in terminal to shutdown ...")

    return camera


def run(camera):
    while True:
        frame = camera.readFrame(FrameType.DISTANCE)

        if frame:
            mat_depth_rgb = np.frombuffer(frame.data_depth_rgb, dtype=np.uint16, count=-1, offset=0).reshape(frame.height, frame.width, 3)
            mat_depth_rgb = mat_depth_rgb.astype(np.uint8)

            # Upscalling the image
            upscale = 4
            img =  cv2.resize(mat_depth_rgb, (frame.width*upscale, frame.height*upscale))

            #print(img) imprimir la matriz
            cv2.imshow('Depth Map', img)

            if cv2.waitKey(1) == 27: break



def cleanup(camera):
    print('\nShutting down ...')
    cv2.destroyAllWindows()
    camera.close()


def depth_image_to_point_cloud(depth_image, h_fov=(-90, 90), v_fov=(-24.9, 2.0), d_range=(0,100)):
    # Adjusting angles for broadcasting
    h_angles = np.deg2rad(np.linspace(h_fov[0], h_fov[1], depth_image.shape[1]))
    v_angles = np.deg2rad(np.linspace(v_fov[0], v_fov[1], depth_image.shape[0]))
    # Reshaping angles for broadcasting
    h_angles = h_angles[np.newaxis, :]  # Shape becomes (1, 1440)
    v_angles = v_angles[:, np.newaxis]  # Shape becomes (64, 1)
    # Calculate x, y, and z
    x = depth_image * np.sin(h_angles) * np.cos(v_angles)
    y = depth_image * np.cos(h_angles) * np.cos(v_angles)
    z = depth_image * np.sin(v_angles)
    # Filter out points beyond the distance range
    valid_indices = (depth_image >= d_range[0]) & (depth_image <= d_range[1])
     
    # Apply the mask to each coordinate array
    x = x[valid_indices]
    y = y[valid_indices]
    z = z[valid_indices]
    # Stack to get the point cloud
    point_cloud = np.stack((x, y, z), axis=-1)
    return point_cloud

def animate_point_clouds(point_clouds):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    # Set background color to black
    vis.get_render_option().background_color = np.array([0, 0, 0])
    # Initialize point cloud geometry
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(point_clouds[0])
    vis.add_geometry(point_cloud)
    frame_index = 0
    last_update_time = time.time()
    update_interval = 0.25  # Time in seconds between frame updates
    while True:
        current_time = time.time()
        if current_time - last_update_time > update_interval:
            # Update point cloud with new data
            point_cloud.points = o3d.utility.Vector3dVector(point_clouds[frame_index])
            vis.update_geometry(point_cloud)
            # Move to the next frame
            frame_index = (frame_index + 1) % len(point_clouds)
            last_update_time = current_time
        vis.poll_events()
        vis.update_renderer()
        if not vis.poll_events():
            break
    vis.destroy_window()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Sample program to demonstrate acquiring frames with distance / depth images from the Tau LiDAR Camera')
    parser.add_argument('--port', metavar='<serial port>', default=None,
                        help='Specify a serial port for the Tau Camera')
    args = parser.parse_args()

    camera = setup(args.port)

    if camera:
        try:
            run(camera)
        except Exception as e:
            print(e)

        cleanup(camera)
