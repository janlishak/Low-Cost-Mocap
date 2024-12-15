import numpy as np
from vispy import scene, app
import threading
import time
import serial
import re

class SerialReader(threading.Thread):
    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.anchors = {"an0": 3, "an1": 3, "an2": 3}
        self.anchor_regex = re.compile(r"^an(\d+):(\d+\.\d+)m$")

    def on_update(self, values):
        print("on update", values)

    def run(self):
        """Thread's main loop to read serial data."""
        self.running = True
        ser = serial.Serial(self.port, self.baudrate, timeout=1)

        try:
            while self.running:
                # Read a line from the serial port
                line = ser.readline().decode('utf-8').strip()

                # Match the line against the regex
                match = self.anchor_regex.match(line)
                if match:
                    anchor_id = f"an{match.group(1)}"
                    distance = float(match.group(2))

                    # Update the corresponding anchor distance
                    if anchor_id in self.anchors:
                        self.anchors[anchor_id] = distance
                        l = [self.anchors["an0"]*97, self.anchors["an1"]*87, self.anchors["an2"]*71]
                        # print(f"Updated {match.group(1)}: {distance}")
                        print(f"Would triangulate {l}")
                        self.on_update(l)

        except Exception as e:
            print(f"Error: {e}")
        finally:
            ser.close()
            print("Serial port closed.")

    def stop(self):
        """Stops the thread."""
        self.running = False

RES = 100
MID = RES // 2


x, y, z = np.indices((RES, RES, RES))

# Define the sphere function
def sphere(d=20, r=5, target_x=MID, target_y=MID, target_z=MID):
    distance = np.sqrt((x - target_x)**2 + (y - target_y)**2 + (z - target_z)**2)
    outline = 1 - (np.abs(distance - d) / r)
    outline_clipped = np.clip(outline, 0, 1)
    return outline_clipped

def get_vol_data(a,b,c):
    # First sphere at the center
    a0 = sphere(d=a, r=10, target_x=0, target_y=RES, target_z=0)
    a1 = sphere(d=b, r=10, target_x=RES, target_y=RES, target_z=0)
    a2 = sphere(d=c, r=10, target_x=RES//2, target_y=0, target_z=0)

    # Add the two spheres together
    matrix = (a0 * a1 * a2)
    # matrix = (a0 + a1 + a2)

    # Prepare the volume data
    vol_data = np.flipud(matrix)
    vol_data = vol_data.swapaxes(0, 2)
    vol_data = vol_data.astype(np.float32)

    # Find the position of the maximum value in the volume
    mp = np.unravel_index(np.argmax(matrix), matrix.shape)
    mp = [-mp[0]+RES,mp[1], mp[2]]
    mp = np.array([mp])
    return vol_data, mp

# Set up the visualization
canvas = scene.SceneCanvas(keys='interactive', bgcolor='black', size=(1024, 1024), show=True)
view = canvas.central_widget.add_view()

# Set up the camera
view.camera = scene.TurntableCamera(
    elevation=30,
    azimuth=-20,
    scale_factor=500,
    center=(100,0,100)  # Move the camera's focus to x = -100
)

# Add the volume
vol_data, marker_position = get_vol_data(141/2,141/2,100/2)
volume = scene.visuals.Volume(vol_data, parent=view.scene, interpolation='nearest', threshold=0.5)


# Add a red marker
markers = scene.visuals.Markers(parent=view.scene)
markers.set_data(marker_position, face_color='red', size=5)
markers.set_gl_state(blend=True, depth_test=False)

# Define the corners of the bounding box
bbox_corners = np.array([
    [0, 0, 0],
    [RES, 0, 0],
    [RES, RES, 0],
    [0, RES, 0],
    [0, 0, RES],
    [RES, 0, RES],
    [RES, RES, RES],
    [0, RES, RES],
])

# Define the edges of the box (outline only)
edges = np.array([
    [0, 1], [1, 2], [2, 3], [3, 0],  # Bottom face
    [4, 5], [5, 6], [6, 7], [7, 4],  # Top face
    [0, 4], [1, 5], [2, 6], [3, 7]   # Vertical edges connecting top and bottom
])

# Create lines from the edges
border_lines = bbox_corners[edges].reshape(-1, 3)

# Add the lines to the scene
border = scene.visuals.Line(pos=border_lines, color='white', width=2, connect='segments', parent=view.scene)
border.set_gl_state(blend=True, depth_test=False)

def update(values):
    a,b,c = values
    vol_data2, marker_position = get_vol_data(a/2,b/2,c/2)
    volume.set_data(vol_data2)
    markers.set_data(marker_position, face_color='red', size=5)
    canvas.update()

def modify_parameters():
    i = 0.0
    while True:
        i = (i+20)%100
        update([141-(i/3),141+(i/5),100+(i/2)])
        time.sleep(1)




# # Start the parameter modification thread
# thread = threading.Thread(target=modify_parameters, daemon=True)
# thread.start()

# Start the app

serial_reader = SerialReader(port='/dev/ttyUSB3')
serial_reader.start()
serial_reader.on_update = update

app.run()
