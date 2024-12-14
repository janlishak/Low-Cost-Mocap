import numpy as np
from vispy import scene, app
import threading
import time

lock = threading.Lock()

RES = 200
MID = RES // 2

# Initial parameters for the spheres
params = {
    'd0': 141,
    'd1': 141,
    'd2': 100,
}

x, y, z = np.indices((RES, RES, RES))

# Define the sphere function
def sphere(d=20, r=5, target_x=MID, target_y=MID, target_z=MID):
    distance = np.sqrt((x - target_x)**2 + (y - target_y)**2 + (z - target_z)**2)
    outline = 1 - (np.abs(distance - d) / r)
    outline_clipped = np.clip(outline, 0, 1)
    return outline_clipped

def get_vol_data(a,b,c):
    # First sphere at the center
    a0 = sphere(d=a, r=20, target_x=0, target_y=RES, target_z=0)
    a1 = sphere(d=b, r=20, target_x=RES, target_y=RES, target_z=0)
    a2 = sphere(d=c, r=20, target_x=RES//2, target_y=0, target_z=0)

    # Add the two spheres together
    # matrix = (a0 * a1 * a2)
    matrix = (a0 + a1 + a2)

    # Prepare the volume data
    vol_data = np.flipud(matrix)
    vol_data = vol_data.swapaxes(0, 2)
    vol_data = vol_data.astype(np.float32)

    # Find the position of the maximum value in the volume
    max_position = np.unravel_index(np.argmax(matrix), matrix.shape)
    return vol_data, max_position

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
vol_data, max_position = get_vol_data(141,141,100)
volume = scene.visuals.Volume(vol_data, parent=view.scene, interpolation='nearest', threshold=0.5)

# Convert the position to the marker position (no additional conversion required here since it's in the same grid)
marker_position = np.array([max_position])

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

def update(a,b,c):
    vol_data2, max_position = get_vol_data(a,b,c)
    volume.set_data(vol_data2)
    marker_position = np.array([max_position])
    markers.set_data(marker_position, face_color='red', size=5)
    canvas.update()

def modify_parameters():
    time.sleep(1)
    update(141,141,200)


# Start the parameter modification thread
thread = threading.Thread(target=modify_parameters, daemon=True)
thread.start()

# Start the app
app.run()
