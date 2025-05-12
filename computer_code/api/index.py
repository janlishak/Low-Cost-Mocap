import sys
import traceback
from helpers import camera_pose_to_serializable, calculate_reprojection_errors, bundle_adjustment, Cameras, triangulate_points, Serial
from KalmanFilter import KalmanFilter

from flask import Flask, Response, request
import cv2 as cv
import numpy as np
import json
from scipy import linalg
from scipy.spatial.transform import Rotation as R

from flask_socketio import SocketIO
import copy
import time
from ruckig import InputParameter, OutputParameter, Result, Ruckig
from flask_cors import CORS
import json
import eventlet
import logging
import json
import pickle
import os
import hashlib

from send_lines import LineBufferWriter
from pose_buffer import PoseBufferReader
from utils import rotationToOpenCV, translationToOpenCV
writer = LineBufferWriter("bevy_line_input_app")
reader = PoseBufferReader(name="bevy_pose_input_app", max_lines=4)

from itertools import permutations, product
permutation_index = 0
all_48_transforms = []
for axes in permutations([0, 1, 2]):
    for signs in product([-1, 1], repeat=3):
        mat = np.zeros((3, 3), dtype=int)
        for i in range(3):
            mat[i, axes[i]] = signs[i]
        all_48_transforms.append(mat)

permutation_index_2 = 0
flip_translations = [
    np.array([[ 1], [ 1], [ 1]]),
    np.array([[ 1], [ 1], [-1]]),
    np.array([[ 1], [-1], [ 1]]),
    np.array([[ 1], [-1], [-1]]),
    np.array([[-1], [ 1], [ 1]]),
    np.array([[-1], [ 1], [-1]]),
    np.array([[-1], [-1], [ 1]]),
    np.array([[-1], [-1], [-1]])
]

# order_num = 5
# def point_order_generator(image_points):
#     global order_num
#     orders = [
#         [0, 1, 2],
#         [0, 2, 1],
#         [1, 0, 2],
#         [1, 2, 0],
#         [2, 0, 1],
#         [2, 1, 0]
#     ]
#     order = orders[order_num]
#     order_num = (order_num + 1) % 6
#     return np.array([image_points[order[0]], image_points[order[1]], image_points[order[2]]], dtype=np.float32)

def generate_all_point_orders(image_points):
    orders = [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0]
    ]
    return np.array([[image_points[i] for i in order] for order in orders], dtype=np.float32)

# def compute_lookat_rotation(camera_pos, target=np.array([0,0,0]), up=np.array([0,1,0])):
#     forward = target - camera_pos
#     forward = forward / np.linalg.norm(forward)

#     right = np.cross(up, forward)
#     right = right / np.linalg.norm(right)

#     true_up = np.cross(forward, right)

#     R = np.vstack([right, true_up, forward]).T
#     return R

app = Flask(__name__)
CORS(app, supports_credentials=True)
socketio = SocketIO(app, cors_allowed_origins='*')
cameras_init = False
num_objects = 2
serial = Serial.instance()
logging.getLogger("werkzeug").setLevel(logging.WARNING)

@app.route("/api/camera-stream")
def camera_stream():
    cameras = Cameras.instance()
    cameras.set_socketio(socketio)
    cameras.set_num_objects(num_objects)

    def gen(cameras):
        frequency = 60
        loop_interval = 1.0 / frequency
        last_run_time = time.time()
        i = 0

        while True:
            try:
                time_now = time.time()
                elapsed_time = time_now - last_run_time

                # If it's time to emit the frame
                if elapsed_time >= loop_interval:
                    frames = cameras.get_frames()
                    jpeg_frame = cv.imencode('.jpg', frames)[1].tobytes()

                    yield (b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n' + jpeg_frame + b'\r\n')

                    # Emit the FPS every second
                    i = (i+1)%10
                    if i == 0:
                        socketio.emit("fps", {"fps": round(1 / elapsed_time)})

                    last_run_time = time.time()  # Update to the current time
                else:
                    # Sleep to maintain the target frame rate
                    eventlet.sleep(loop_interval - elapsed_time)
            except Exception as e:
                print("Fatal exception in generator:", e)
                traceback.print_exc()
                sys.exit(1)  # hard exit

    return Response(gen(cameras), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/api/trajectory-planning", methods=["POST"])
def trajectory_planning_api():
    data = json.loads(request.data)

    waypoint_groups = [] # grouped by continuious movement (no stopping)
    for waypoint in data["waypoints"]:
        stop_at_waypoint = waypoint[-1]
        if stop_at_waypoint:
            waypoint_groups.append([waypoint[:3*num_objects]])
        else:
            waypoint_groups[-1].append(waypoint[:3*num_objects])
    
    setpoints = []
    for i in range(0, len(waypoint_groups)-1):
        start_pos = waypoint_groups[i][0]
        end_pos = waypoint_groups[i+1][0]
        waypoints = waypoint_groups[i][1:]
        setpoints += plan_trajectory(start_pos, end_pos, waypoints, data["maxVel"], data["maxAccel"], data["maxJerk"], data["timestep"])

    return json.dumps({
        "setpoints": setpoints
    })

def plan_trajectory(start_pos, end_pos, waypoints, max_vel, max_accel, max_jerk, timestep):
    otg = Ruckig(3*num_objects, timestep, len(waypoints))  # DoFs, timestep, number of waypoints
    inp = InputParameter(3*num_objects)
    out = OutputParameter(3*num_objects, len(waypoints))

    inp.current_position = start_pos
    inp.current_velocity = [0,0,0]*num_objects
    inp.current_acceleration = [0,0,0]*num_objects

    inp.target_position = end_pos
    inp.target_velocity = [0,0,0]*num_objects
    inp.target_acceleration = [0,0,0]*num_objects

    inp.intermediate_positions = waypoints

    inp.max_velocity = max_vel*num_objects
    inp.max_acceleration = max_accel*num_objects
    inp.max_jerk = max_jerk*num_objects

    setpoints = []
    res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)
        setpoints.append(copy.copy(out.new_position))
        out.pass_to_input(inp)

    return setpoints

@socketio.on("arm-drone")
def arm_drone(data):
    global cameras_init
    if not cameras_init:
        return
    
    Cameras.instance().drone_armed = data["droneArmed"]
    for droneIndex in range(0, num_objects):
        serial_data = {
            "armed": data["droneArmed"][droneIndex],
        }
        serial.write(f"{str(droneIndex)}{json.dumps(serial_data)}".encode('utf-8'))

@socketio.on("set-drone-pid")
def arm_drone(data):
    serial_data = {
        "pid": [float(x) for x in data["dronePID"]],
    }
    serial.write(f"{str(data['droneIndex'])}{json.dumps(serial_data)}".encode('utf-8'))

@socketio.on("set-drone-setpoint")
def arm_drone(data):
    serial_data = {
        "setpoint": [float(x) for x in data["droneSetpoint"]],
    }
    serial.write(f"{str(data['droneIndex'])}{json.dumps(serial_data)}".encode('utf-8'))

@socketio.on("set-drone-trim")
def arm_drone(data):
    serial_data = {
        "trim": [int(x) for x in data["droneTrim"]],
    }
    serial.write(f"{str(data['droneIndex'])}{json.dumps(serial_data)}".encode('utf-8'))



def get_plane_points(fit):
    # Fitted parameters
    A = fit[0]
    B = fit[1]
    C = -1  # Normal component
    D = fit[2]

    # Define three arbitrary points in the XY plane
    points = []
    for (x, y) in [(0, 0), (1, 0), (0, 1)]:
        z = - (A * x + B * y + D) / C  # Solve for z using the plane equation
        points.append((x, y, z))

    return points

def compute_euler_angles(plane_normal):
    # Normalize the plane normal
    plane_normal = plane_normal / np.linalg.norm(plane_normal)

    # Calculate roll, pitch, and yaw from the normal vector
    roll = np.arctan2(plane_normal[1], plane_normal[2])  # Rotation around x-axis
    pitch = np.arctan2(-plane_normal[0], np.sqrt(plane_normal[1]**2 + plane_normal[2]**2))  # Rotation around y-axis
    yaw = 0  # No rotation around z-axis, adjust if necessary
    
    return roll, pitch, yaw

def euler_to_rotation_matrix(roll, pitch, yaw):
    # Create rotation matrices for each angle
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    # Combined rotation matrix
    R = R_z @ R_y @ R_x
    return R

def euler_degrees_to_rotation_matrix(roll, pitch, yaw):
    # Convert degrees to radians
    roll = np.radians(roll)
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)
    
    # Create rotation matrices for each angle
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    # Combined rotation matrix
    R = R_z @ R_y @ R_x
    return R

@socketio.on("acquire-floor")
def acquire_floor(data):
    cameras = Cameras.instance()
    object_points = data["objectPoints"]
    object_points = np.array([item for sublist in object_points for item in sublist])

    # Prepare A matrix and b vector for least squares
    tmp_A = []
    tmp_b = []
    
    for i in range(len(object_points)):
        tmp_A.append([object_points[i, 0], object_points[i, 1], 1])
        tmp_b.append(object_points[i, 2])
        
    b = np.matrix(tmp_b).T
    A = np.matrix(tmp_A)

    # Fit the plane
    fit, residual, rnk, s = linalg.lstsq(A, b)
    fit = fit.T[0]

    triangle_points = get_plane_points(fit)
    socketio.emit("triangle-points", {'triangle_points': triangle_points})

    # Wait for a while before proceeding (optional, consider removing for performance)
    time.sleep(5)

    # Plane normal from fitted coefficients
    plane_normal = np.array([fit[0], fit[1], -1])  # Create a 1D array for the normal

    # Compute Euler angles from the plane normal
    roll, pitch, yaw = compute_euler_angles(plane_normal)
    print([roll, pitch, yaw])

    roll = 90
    pitch = 0
    yaw = 0

    # Compute the rotation matrix from the Euler angles
    R_matrix = euler_to_rotation_matrix(roll, pitch, yaw)

    # Translation vector (you can adjust this based on your requirements)
    t_x = 0
    t_y = 0
    # t_z = -fit[2]  # Aligning the plane to z=0 using the fitted plane's z-intercept
    t_z = 0

    # Construct the transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = R_matrix
    transformation_matrix[:3, 3] = np.array([t_x, t_y, t_z])

    # Set the camera world coordinates matrix
    cameras.to_world_coords_matrix = transformation_matrix

    # Emit the new world coordinates matrix
    socketio.emit("to-world-coords-matrix", {"to_world_coords_matrix": cameras.to_world_coords_matrix.tolist()})

@socketio.on("update-camera-rotation")
def acquire_floor(data):
    roll = data["roll"]
    pitch = data["pitch"]
    yaw = data["yaw"]
    t_x = data["x"]
    t_y = data["y"]
    t_z = data["z"]

    # Compute the rotation matrix from the Euler angles
    R_matrix = euler_degrees_to_rotation_matrix(roll, pitch, yaw)

    # Construct the transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = R_matrix
    transformation_matrix[:3, 3] = np.array([t_x, t_y, t_z])

    # Set the camera world coordinates matrix
    cameras = Cameras.instance()
    cameras.to_world_coords_matrix = transformation_matrix

    # Emit the new world coordinates matrix
    socketio.emit("to-world-coords-matrix", {"to_world_coords_matrix": cameras.to_world_coords_matrix.tolist()})



@socketio.on("set-origin")
def set_origin(data):
    cameras = Cameras.instance()
    object_point = np.array(data["objectPoint"])
    to_world_coords_matrix = np.array(data["toWorldCoordsMatrix"])
    transform_matrix = np.eye(4)

    object_point[1], object_point[2] = object_point[2], object_point[1] # i dont fucking know why
    transform_matrix[:3, 3] = -object_point

    to_world_coords_matrix = transform_matrix @ to_world_coords_matrix
    cameras.to_world_coords_matrix = to_world_coords_matrix

    socketio.emit("to-world-coords-matrix", {"to_world_coords_matrix": cameras.to_world_coords_matrix.tolist()})

@socketio.on("update-camera-settings")
def change_camera_settings(data):
    cameras = Cameras.instance()
    
    cameras.edit_settings(data["exposure"], data["gain"])

@socketio.on("capture-points")
def capture_points(data):
    start_or_stop = data["startOrStop"]
    cameras = Cameras.instance()

    if (start_or_stop == "start"):
        cameras.start_capturing_points()
        return
    elif (start_or_stop == "stop"):
        cameras.stop_capturing_points()


@socketio.on("calculate-camera-pose-triangle")
def calculate_camera_pose(data):
    HASHINPUT = "acc7cda5678cbd7d6a95944a592986ca"

    # make sure cache dir exists
    CACHE_DIR ="cache"
    os.makedirs(CACHE_DIR, exist_ok=True)
    # Save new input
    computed_hash = hashlib.md5(pickle.dumps(data)).hexdigest()
    cache_path = os.path.join(CACHE_DIR, f"{computed_hash}_input_new.pkl")
    if not os.path.exists(cache_path):
        with open(cache_path, "wb") as f:
            pickle.dump(data, f)
    print(f"POINTS HASHS NEW: {computed_hash}")  # Return the hash for reference
    # Load saved input
    if HASHINPUT is not None:
        cache_path = os.path.join(CACHE_DIR, f"{HASHINPUT}_input_new.pkl")
        if os.path.exists(cache_path):
            with open(cache_path, "rb") as f:
                data = pickle.load(f)
        else:
            raise FileNotFoundError(f"No cached input found for hash: {HASHINPUT}")
        
    
    # global order
    # print(f"ORD: {order_num}") 

    # get data
    camera_points = data["cameraPoints"]
    cameras = Cameras.instance()
    results = [[] for _ in range(4)]

    for camera_index in range(cameras.num_cameras):
    # for camera_index in [3]:
        # fill missing points where camera seen less than 3 points
        standardized_camera_points = []
        for points in camera_points:
            while len(points) < 3:
                print("missing points")
                points.append([None, None])
            standardized_camera_points.append(points)

        image_points = np.array(standardized_camera_points)
        image_points_t = image_points.transpose((1, 0, 2, 3))

        # print(f"(image_points_t): {image_points_t[0][0]}")

        # # Define object points like in bevy
        # object_points = np.array([
        #     [-0.075, 0.0,   0.0],
        #     [ 0.075, 0.0,   0.0],
        #     [ 0.0,   0.0, -0.058]
        # ], dtype=np.float32)

        # in object space of openCV
        object_points = np.array([
            [-0.075, 0.0,   0.0],
            [ 0.075, 0.0,   0.0],
            [ 0.0,   0.0, 0.058]
        ], dtype=np.float32)


        # object_points = np.array([
        #     [-0.075, 0.0,   0.0],
        #     [ 0.075, 0.0,   0.0],
        #     [ 0.0,   0.058, 0.0]
        # ], dtype=np.float32)

        # Example: image_points from your detection (Camera 0, Frame 0)
        observerd_order_image_points = np.array(image_points_t[camera_index][0], dtype=np.float32)  # Shape (3,2)

        ord_num = 0
        for image_points in generate_all_point_orders(observerd_order_image_points):
            # print(f"(image_points): {image_points}")

            # Camera intrinsics
            K = cameras.get_camera_params(0)["intrinsic_matrix"]
            dist_coeffs = np.zeros((4,1))   # Assuming no distortion, or load actual values

            # Solve PnP
            # Provide an initial guess (identity rotation, zero translation)
            rvec_init = np.zeros((3, 1), dtype=np.float32)
            tvec_init = np.zeros((3, 1), dtype=np.float32)

            success, rvecs, tvecs, errors = cv.solvePnPGeneric(
                object_points, 
                image_points, 
                K, 
                dist_coeffs, 
                # rvec=rvec_init, 
                # tvec=tvec_init, 
                # useExtrinsicGuess=True, 
                flags=cv.SOLVEPNP_SQPNP
            )

            if success:
                for i in range(len(rvecs)):
                    rvec = rvecs[i]
                    tvec = tvecs[i]

                    R, _ = cv.Rodrigues(rvec)
                    # print("Rotation Matrix:\n", R)
                    # print("Translation Vector:\n", tvec)
                    distance = np.linalg.norm(tvec)
                    # print(f"Estimated distance from triangle to camera: {distance:.4f} meters")

                    # 1. Define axis to visualize (length = 0.05m)
                    axis_points = np.float32([
                        [0, 0, 0],      # Origin
                        [0.15, 0, 0],   # X
                        [0, 0.15, 0],   # Y
                        [0, 0, 0.15]    # Z
                    ])

                    # 2. Project axis points
                    projected_axis, _ = cv.projectPoints(axis_points, rvec, tvec, K, dist_coeffs)

                    # 3. Project object points (should align with detected points)
                    reprojected_points, _ = cv.projectPoints(object_points, rvec, tvec, K, dist_coeffs)

                    # ---- Drawing Section ----
                    frame = cameras.cameras.bevy_cams[camera_index].read_frame()   # Get a frame from Camera 0
                    frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)        # Ensure correct color format

                    # Extract points
                    origin = tuple(projected_axis[0].ravel().astype(int))
                    x_end  = tuple(projected_axis[1].ravel().astype(int))
                    y_end  = tuple(projected_axis[2].ravel().astype(int))
                    z_end  = tuple(projected_axis[3].ravel().astype(int))

                    # 3. Draw axes from origin
                    cv.line(frame, origin, x_end, (0,0,255), 1)  # X - Red
                    cv.line(frame, origin, y_end, (0,255,0), 1)  # Y - Green
                    cv.line(frame, origin, z_end, (255,0,0), 1)  # Z - Blue


                    height, width = frame.shape[:2]
                    # Offset
                    offset = 5
                    # Define points with offset
                    top_left = (offset, offset)
                    top_right = (width - offset, offset)
                    bottom_right = (width - offset, height - offset)
                    bottom_left = (offset, height - offset)
                    # print(top_left)
                    # print(top_right)
                    # print(bottom_right)
                    # print(bottom_left)
                    # Draw lines
                    # cv.line(frame, top_left, top_right, (0, 255, 0), 2)
                    # cv.line(frame, top_right, bottom_right, (0, 255, 0), 2)
                    # cv.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
                    # cv.line(frame, bottom_left, top_left, (0, 255, 0), 2)

                    corners_2d = np.array([
                        [0, 0],                 # Top-left
                        [width, 0],             # Top-right
                        [width, height],        # Bottom-right
                        [0, height],            # Bottom-left
                        [width//2, 0]           # Top-middle
                    ], dtype=np.float32)

                    # Back-project to 3D camera space (Z = 1)
                    K_inv = np.linalg.inv(K)
                    camera_points_3d = []

                    for corner in corners_2d:
                        pixel_homogeneous = np.array([corner[0], corner[1], 1.0])
                        direction = K_inv @ pixel_homogeneous
                        point_cam_space = direction / direction[2]   # Normalize so Z = 1
                        camera_points_3d.append(point_cam_space)

                    camera_points_3d = np.array(camera_points_3d, dtype=np.float32)

                    rvec_zero = np.zeros((3,1), dtype=np.float32)
                    tvec_zero = np.zeros((3,1), dtype=np.float32)

                    reprojected_points, _ = cv.projectPoints(camera_points_3d, rvec_zero, tvec_zero, K, dist_coeffs)

                    # print("Original 2D Points:\n", corners_2d)
                    # print("Reprojected Points:\n", reprojected_points.reshape(-1, 2))

                    # Invert the transformation
                    R_obj_to_cam, _ = cv.Rodrigues(rvec)
                    R_cam_in_obj = R_obj_to_cam.T
                    t_cam_in_obj = -R_cam_in_obj @ tvec

                    # 3. Apply inverse transformation oP = point In object space
                    P_o = (R_cam_in_obj @ camera_points_3d.T).T + t_cam_in_obj.T

                    # Convert all object points and the translation vector to OpenCV (or vice versa)
                    P_b = np.array([translationToOpenCV(p) for p in P_o])
                    cam_b = translationToOpenCV(t_cam_in_obj)

                    # Frame
                    writer.next_line(P_b[0], P_b[1])
                    writer.next_line(P_b[1], P_b[2])
                    writer.next_line(P_b[2], P_b[3])
                    writer.next_line(P_b[3], P_b[0])
                    writer.next_line(P_b[4], cam_b)

                    # # frustom
                    # writer.next_line(p5, p1)
                    # writer.next_line(p5, p2)
                    # writer.next_line(p5, p3)
                    # writer.next_line(p5, p4)

                    # # Draw reprojected points
                    # for pt in reprojected_points:
                    #     cv.circle(frame, tuple(pt.ravel().astype(int)), 5, (0,255,255), -1)

                    # # OPTIONAL: Draw detected points for comparison
                    # for pt in image_points:
                    #     cv.circle(frame, tuple(pt.ravel().astype(int)), 5, (255,0,255), 2)

                    # # Show the frame (for debugging, or send it somewhere)
                    # print(ord_num)
                    # print(f"tvec: {t_cam_in_obj} rvec: {R_cam_in_obj}")
                    # cv.imshow("Pose Debug", frame)
                    # cv.waitKey(0)
                    # cv.destroyAllWindows()

                    res = {
                        "R": R_cam_in_obj,
                        "t": t_cam_in_obj
                    }
                    results[camera_index].append(res)
                    ord_num += 1
            else:
                print("PnP failed")
        ord_num += 1



    # camera_rotations = [
    #     np.array([
    #         [-0.88464818,  0.18091275, -0.42973035],
    #         [-0.02716579, -0.94008888, -0.33984543],
    #         [-0.46546709, -0.28896968,  0.83656256]
    #     ]),
    #     np.array([
    #         [-0.87930107, -0.1059963,   0.46432145],
    #         [ 0.01764399, -0.98150028, -0.19064598],
    #         [ 0.4759394,  -0.15944274,  0.86490444]
    #     ]),
    #     np.array([
    #         [ 0.84041556, -0.10075993,  0.53249331],
    #         [-0.04297638, -0.99186082, -0.11985468],
    #         [ 0.5402358,   0.07784311, -0.83790556]
    #     ]),
    #     np.array([
    #         [ 0.76256755,  0.15331514, -0.62847848],
    #         [-0.02529311, -0.96370153, -0.26578115],
    #         [-0.64641395,  0.21857225, -0.7310097]
    #     ])
    # ]

    # camera_translations = [
    #     np.array([[1.84351665], [2.25643426], [-3.13078911]]),
    #     np.array([[-1.91002897], [1.98233738], [-3.06587392]]),
    #     np.array([[-1.96112615], [1.81881143], [3.06741785]]),
    #     np.array([[2.21896577], [2.11639218], [2.96994236]])
    # ]
    
    
    cams = []
    cams.append(results[0][10])
    cams.append(results[1][6])
    cams.append(results[2][1])
    cams.append(results[3][8])

    camera_poses = []
    camera_rotations = []
    camera_translations = []
    print("CAMERA POSES:")
    for i, cam in enumerate(cams):
        rvec = cam["R"]
        tvec = cam["t"]
        camera_translations.append(tvec)
        camera_rotations.append(rvec)
        print(f"\nCamera {i}:")
        print(f"Rotation Vector (rvec): \n{rotationToOpenCV(rvec)}")
        print(f"Translation Vector (tvec): \n{translationToOpenCV(tvec)}")
        print(f"--- --- --- --- ---")

        camera_pose = {
            "R": cam["R"],
            "t": cam["t"]
        }
        camera_poses.append(camera_pose) 



    # # Use first camera as origin
    # camera_poses = []
    # R0_inv = camera_rotations[0].T
    # t0 = camera_translations[0]

    # camera_poses.append({
    #     "R": np.eye(3),
    #     "t": np.zeros((3, 1))
    # })

    # for R, t in zip(camera_rotations[1:], camera_translations[1:]):
    #     R_rel = R0_inv @ R
    #     t_rel = R0_inv @ (t - t0)
    #     camera_poses.append({
    #         "R": R_rel,
    #         "t": t_rel,
    #     })

    

    # Just send tham as they are
    camera_poses = []
    for R, t in zip(camera_rotations, camera_translations):
        camera_poses.append({ "R": R, "t": t,})

    socketio.emit("camera-pose", {"camera_poses": camera_pose_to_serializable(camera_poses)})


    
    # Caluclate only first camera, rest are in line
    # R, _ = cv.Rodrigues(rvec)
    # camera_poses = [{
    #     "R": R,
    #     "t": tvec
    # }]
    # # # api requires all cameras set, just place them on a line
    # for i in range(3):
    #     camera_poses.append(
    # {
    #     "R": np.eye(3),
    #     "t": np.array([[1 + i], [0], [0]], dtype=np.float32)
    # })
        
    
    # camera_poses = []
    # for i in range(4):
    #     cam_pos = np.random.uniform(-1, 1, size=(3,))
    #     R = compute_lookat_rotation(cam_pos)
    #     t = cam_pos.reshape(3,1)

    #     camera_poses.append({
    #         "R": R,
    #         "t": t
    #     })
  
    # socketio.emit("camera-pose", {"camera_poses": camera_pose_to_serializable(camera_poses)})



@socketio.on("calculate-camera-pose")
def calculate_camera_pose(data):
    # [x[0] for x in image_points]
    # HASHINPUT = "8671c7ee44482ff7b411ffe5180d5d64"
    # HASHINPUT = "80ba7f4f0cd00be0db42379006fef197"
    HASHINPUT = "0e89542d39b7d98a28321871773eccf1"

    # HASHINPUT = None

    # make sure cache dir exists
    CACHE_DIR ="cache"
    os.makedirs(CACHE_DIR, exist_ok=True)

    # Save new input
    computed_hash = hashlib.md5(pickle.dumps(data)).hexdigest()
    cache_path = os.path.join(CACHE_DIR, f"{computed_hash}_input.pkl")
    if not os.path.exists(cache_path):
        with open(cache_path, "wb") as f:
            pickle.dump(data, f)
    
    print(f"POINTS HASHS: {computed_hash}")  # Return the hash for reference

    # Load saved input
    if HASHINPUT is not None:
        cache_path = os.path.join(CACHE_DIR, f"{HASHINPUT}_input.pkl")
        if os.path.exists(cache_path):
            with open(cache_path, "rb") as f:
                data = pickle.load(f)
        else:
            raise FileNotFoundError(f"No cached input found for hash: {HASHINPUT}")

    print("running old")
    cameras = Cameras.instance()

    # get first frame and first camera
    first_cam = data["cameraPoints"][0][0]
    if isinstance(first_cam[0], list) or isinstance(first_cam[0], tuple):
        trimmed = [[cam[0] for cam in frame] for frame in data["cameraPoints"]] # format (frames, cameras, points, 2)
    else:
        trimmed = data["cameraPoints"] # format (frames, cameras, 2)

    image_points = np.array(trimmed)  # shape: (n_frames, n_cameras, 2)

    image_points_t = image_points.transpose((1, 0, 2))

    camera_poses = [{
        "R": np.eye(3),
        "t": np.array([[0],[0],[0]], dtype=np.float32)
    }]
    for camera_i in range(0, cameras.num_cameras-1):
        camera1_image_points = image_points_t[camera_i]
        camera2_image_points = image_points_t[camera_i+1]
        not_none_indicies = np.where(np.all(camera1_image_points != None, axis=1) & np.all(camera2_image_points != None, axis=1))[0]
        camera1_image_points = np.take(camera1_image_points, not_none_indicies, axis=0).astype(np.float32)
        camera2_image_points = np.take(camera2_image_points, not_none_indicies, axis=0).astype(np.float32)

        F, _ = cv.findFundamentalMat(camera1_image_points, camera2_image_points, cv.FM_RANSAC, 1, 0.99999)
        E = cv.sfm.essentialFromFundamental(F, cameras.get_camera_params(0)["intrinsic_matrix"], cameras.get_camera_params(1)["intrinsic_matrix"])
        possible_Rs, possible_ts = cv.sfm.motionFromEssential(E)

        R = None
        t = None
        max_points_infront_of_camera = 0
        for i in range(0, 4):
            object_points = triangulate_points(np.hstack([np.expand_dims(camera1_image_points, axis=1), np.expand_dims(camera2_image_points, axis=1)]), np.concatenate([[camera_poses[-1]], [{"R": possible_Rs[i], "t": possible_ts[i]}]]))
            object_points_camera_coordinate_frame = np.array([possible_Rs[i].T @ object_point for object_point in object_points])

            points_infront_of_camera = np.sum(object_points[:,2] > 0) + np.sum(object_points_camera_coordinate_frame[:,2] > 0)

            if points_infront_of_camera > max_points_infront_of_camera:
                max_points_infront_of_camera = points_infront_of_camera
                R = possible_Rs[i]
                t = possible_ts[i]

        R = R @ camera_poses[-1]["R"]
        t = camera_poses[-1]["t"] + (camera_poses[-1]["R"] @ t)

        camera_poses.append({
            "R": R,
            "t": t
        })

    camera_poses = bundle_adjustment(image_points, camera_poses, socketio)

    object_points = triangulate_points(image_points, camera_poses)
    error = np.mean(calculate_reprojection_errors(image_points, object_points, camera_poses))

    socketio.emit("camera-pose", {"camera_poses": camera_pose_to_serializable(camera_poses)})

@socketio.on("locate-objects")
def start_or_stop_locating_objects(data):
    cameras = Cameras.instance()
    start_or_stop = data["startOrStop"]

    if (start_or_stop == "start"):
        cameras.start_locating_objects()
        return
    elif (start_or_stop == "stop"):
        cameras.stop_locating_objects()

@socketio.on("determine-scale")
def determine_scale(data):
    object_points = data["objectPoints"]
    camera_poses = data["cameraPoses"]
    actual_distance = 0.81
    observed_distances = []

    for object_points_i in object_points:
        if len(object_points_i) != 2:
            continue

        object_points_i = np.array(object_points_i)

        observed_distances.append(np.sqrt(np.sum((object_points_i[0] - object_points_i[1])**2)))

    scale_factor = actual_distance/np.mean(observed_distances)
    for i in range(0, len(camera_poses)):
        camera_poses[i]["t"] = (np.array(camera_poses[i]["t"]) * scale_factor).tolist()

    socketio.emit("camera-pose", {"error": None, "camera_poses": camera_poses})


@socketio.on("determine-average")
def determine_average(data):
    object_points = data["objectPoints"]
    # camera_poses = data["cameraPoses"]

    # Flatten the list and convert to a NumPy array
    flattened_points = np.vstack(object_points)

    # Compute the average for each coordinate (x, y, z)
    average_point = np.mean(flattened_points, axis=0)

    print("Average point:", average_point)

@socketio.on("add-beacons")
def add_beacons(data):
    points = [
        [-0.3307645344934884, 1.2850473280458907, 0.6427937719874424]
    ]

    points = [
        [1,1,0.00], [1,1,0.05], [1,1,0.10], [1,1,0.15], [1,1,0.20], # Point A (up-left)
        [-1,1,0.00], [-1,1,0.05], [-1,1,0.10], [-1,1,0.15], [-1,1,0.20], # Point B (up-right)
        [0,-1,0.00], [0,-1,0.05], [0,-1,0.10], [0,-1,0.15], [0,-1,0.20], # Point C (down-middle)
    ]

    errors = [1,2,3,4,5]

    socketio.emit("object-points", {
        "object_points": points, 
        "errors": errors,
        "objects": [], 
        "filtered_objects": []
    })

@socketio.on("save-points")
def save_points(data):
    object_points = data["objectPoints"]
    objectPointErrors = data["objectPointErrors"]
    save_number = data["saveNumber"]
    print("--- DATA BEGIN ---")
    print(object_points)
    print(objectPointErrors)
    print("--- DATA END ---")
    print("Save Number:", save_number)
    print("todo: implement saving")
    # todo: save the file to points-001.json
    

@socketio.on("load-points")
def load_points(data):
    writer.reset()
    save_number = data["saveNumber"]
    object_points = None # todo: load from json
    print("todo: implement loading")

@socketio.on("reset-simulation-gyzmos")
def reset_simulation_gyzmos(data):
    writer.reset()
    print("sim: reset gyzmos")

@socketio.on("load-simulation-poses")
def load_simulation_poses(data):
    # camera_rotations = [
    #     np.array([
    #         [-0.8618331, -0.14180309,  0.486966],
    #         [ 7.450581e-9, 0.9601211, 0.2795845],
    #         [-0.5071923, 0.24095514, -0.8274641]
    #     ]),
    #     np.array([
    #         [-0.850157,  0.09008469, -0.5187658],
    #         [1.8626451e-8, 0.9852552, 0.17109145],
    #         [0.52652943, 0.14545457, -0.83762157]
    #     ]),
    #     np.array([
    #         [0.83510876, 0.074800774, -0.54497546],
    #         [1.1175871e-8, 0.9907115, 0.13598043],
    #         [0.5500849, -0.11355846, 0.8273518]
    #     ]),
    #     np.array([
    #         [0.74628246, -0.15128975, 0.6482082],
    #         [1.4901161e-8, 0.97382754, 0.22728826],
    #         [-0.6656293, -0.16962124, 0.72675043]
    #     ])
    # ]

    # camera_translations = [
    #     np.array([[2.0251737], [2.158235], [-3.0000007]]),
    #     np.array([[-2.16109], [2.151783], [-3.0000005]]),
    #     np.array([[-2.0576675], [2.1107368], [3.0000007]]),
    #     np.array([[2.3062143], [2.1781263], [3.0000005]])
    # ]

    # camera_translations[0][1] += 1

    TEST_CONVERSION = 2

    # SIM DATA
    scale_factor = 0.21317899478641678
    camera_rotations = []
    camera_translations = []
    for i in range(4):
        R, t = reader.get_R_t(i)
        camera_rotations.append(R)
        camera_translations.append(np.asarray(t).reshape(3, 1))

    if TEST_CONVERSION == 0:
        print("IDK this ..")
        # flip = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
        global permutation_index
        global permutation_index_2
        flip = all_48_transforms[permutation_index]
        t_flip = flip_translations[permutation_index_2]
        permutation_index += 1
        permutation_index_2 += 1
        print("--- --- ---")
        print(flip)
        print(t_flip)
        print("--- --- ---")

        # camera_rotations = [flip  @ R for R in camera_rotations]
        # camera_translations = [flip  @ t for t in camera_translations]

        # flip_t = np.array([[1], [-1], [1]]) 
        camera_translations = [t_flip * t for t in camera_translations]


        # cameras = Cameras.instance()
        # scale_factor = cameras.cameras.exposure[0]/100
        # print(scale_factor)
        camera_poses = []

        # Use first camera as origin
        R0_inv = camera_rotations[0].T
        t0 = camera_translations[0]

        camera_poses.append({
            "R": np.eye(3),
            "t": np.zeros((3, 1))
        })

        for R, t in zip(camera_rotations[1:], camera_translations[1:]):
            R_rel = R0_inv @ R
            t_rel = R0_inv @ (t - t0)
            camera_poses.append({
                "R": R_rel,
                "t": t_rel * scale_factor,
            })
    
    elif TEST_CONVERSION == 1:
        print("CAMERA POSES FROM SIMULATION:")

        flip_r= np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
        camera_rotations = [flip_r @ R for R in camera_rotations]

        flip_t = np.array([[1], [-1], [1]]) 
        camera_translations = [flip_t * t for t in camera_translations]


        camera_poses = []
        for R, t in zip(camera_rotations[0:], camera_translations[0:]):
            # print(f"\nCamera {"X"}:")
            # print(f"Rotation Vector (rvec): \n{R}")
            # print(f"Translation Vector (tvec): \n{t}")
            # print(f"--- --- --- --- ---")

            camera_poses.append({
                "R": R,
                "t": t * scale_factor,
            })


    elif TEST_CONVERSION == 2:
        print("CAMERA POSES FROM SIMULATION 1st Camera ORIGIN: UTILS+")

        # flip_r= np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
        # camera_rotations = [flip_r @ R for R in camera_rotations]
        # flip_t = np.array([[1], [-1], [1]]) 
        # camera_translations = [flip_t * t for t in camera_translations]

        camera_rotations = [rotationToOpenCV(R) for R in camera_rotations]
        camera_translations = [translationToOpenCV(t) for t in camera_translations]

        # flip_t = np.array([[1], [1], [1]]) 
        # camera_translations = [flip_t * t for t in camera_translations]

        
        # # Use first camera as origin
        # R0_inv = camera_rotations[0].T
        # t0 = camera_translations[0]

        # camera_poses = []
        # camera_poses.append({
        #     "R": np.eye(3),
        #     "t": np.zeros((3, 1))
        # })

        # # offset_local = np.array([[0.0], [0.0], [0.0]])
        # for R, t in zip(camera_rotations[1:], camera_translations[1:]):
        #     R_rel = R0_inv @ R
        #     t_rel = R0_inv @ (t - t0)
        #     # offset_world = R_rel @ offset_local
        #     # offset_first_camera_space = np.array([[0.0], [0.0], [0.0]])
        #     # + offset_world + offset_first_camera_space
        #     camera_poses.append({
        #         "R": R_rel,
        #         "t": t_rel,
        #     })

        camera_poses = []
        for R, t in zip(camera_rotations, camera_translations):
            camera_poses.append({
                "R": R,
                "t": t,
            })

    elif TEST_CONVERSION == 3:
        R0 = camera_rotations[0]
        t0 = camera_translations[0]

        camera_poses = [{
            "R": np.eye(3),
            "t": np.zeros((3, 1))
        }]

        for R, t in zip(camera_rotations[1:], camera_translations[1:]):
            R_rel = R0.T @ R
            t_rel = R0.T @ (t - t0)
            camera_poses.append({
                "R": R_rel,
                "t": t_rel,
            })

    socketio.emit("camera-pose", {"camera_poses": camera_pose_to_serializable(camera_poses)})

@socketio.on("test-diff")
def test_diff(data):
    print("test diff")

    best_ba = [{"R":[[1,0,0],[0,1,0],[0,0,1]],"t":[0,0,0]},{"R":[[0.42875885812009135,-0.2421588305912347,0.8703590881643284],[0.15541792495722054,0.9688139044292223,0.19298934474886892],[-0.8899500605008954,0.05252351236628881,0.4530233663544974]],"t":[-0.6591285361608913,-0.043671214881990744,0.6000534661871276]},{"R":[[-0.9715083568151248,-0.05296505526180127,0.23101128881396812],[0.04737919632251508,0.9116328169893706,0.4082656227798973],[-0.23222128324985378,0.4075785935364358,-0.883149458300605]],"t":[-0.08831869133856607,-0.15379236773129423,1.4564293521916798]},{"R":[[-0.3191704436507617,0.2547489563377422,-0.9128160806781435],[-0.22120275673186598,0.9165629298600755,0.3331392141741001],[0.921520248455778,0.3082456242205964,-0.23618862554080794]],"t":[0.6804468494579395,-0.17150260126833697,0.9910185755251584]}]
    best_sim = [{"R":[[1,0,0],[0,1,0],[0,0,1]],"t":[[0],[0],[0]]},{"R":[[0.4656417778858568,-0.15141140477402448,0.8719247633266392],[0.2474249153086674,0.9682380241129767,0.03600188167583074],[-0.8496817752010908,0.19897184893955533,0.4883149868460297]],"t":[[-0.7691200454836932],[-0.1252278417108802],[0.43496449023989414]]},{"R":[[-0.9987232229486054,-0.006869804536422386,0.05005146069699151],[0.014124789814411454,0.9132335321000797,0.40719163285259885],[-0.04850596423297571,0.4073786221345159,-0.9119704430178306]],"t":[[-0.10138217967165271],[-0.4218997446150533],[1.4850625338761347]]},{"R":[[-0.3055687821751185,0.2164171498496792,-0.9272495866780162],[-0.2662120125665055,0.9155746020344826,0.30142058962106866],[0.9141986714858295,0.338949660947792,-0.2221581313993255]],"t":[[0.700370730597649],[-0.30377512598650946],[1.0280273635002777]]}]

    index = 0
    both = [best_ba, best_sim]


    for _ in range(1000):
        index = (index + 1) % 2
        camera_poses = both[index]
        socketio.emit("camera-pose", {"camera_poses": camera_poses})
        eventlet.sleep(0.01)
    print("done")

@socketio.on("triangulate-test")
def triangulate_test(data):
    print("running triangulate-test")
    cameras = Cameras.instance()
    camera_poses = data["cameraPoses"]

    debug_frames = []
    for cam_index in range(4):
        K = cameras.get_camera_params(0)["intrinsic_matrix"]
        dist_coeffs = np.zeros((4,1)) 

        R_cam = np.array(camera_poses[cam_index]["R"])
        t_cam  = np.array(camera_poses[cam_index]["t"])

        R_world_to_cam = R_cam.T
        t_world_to_cam = -R_world_to_cam @ t_cam
        rvec, _ = cv.Rodrigues(R_world_to_cam)
        tvec = t_world_to_cam

        # 1. Define axis to visualize (length = 0.05m)
        axis_points = np.float32([
            [0, 0, 0],      # Origin
            [0.3, 0, 0],   # X
            [0, 0.4, 0],   # Y
            [0, 0, 0.5],   # Z
            # [0.3, -0.4, 0.1],
            # [0.2, -0.4, 0.4],
            # [0.7, -0.4, -0.6]
        ])

        # 2. Project axis points
        projected_axis, _ = cv.projectPoints(axis_points, rvec, tvec, K, dist_coeffs)

        # ---- Drawing Section ----
        frame = cameras.cameras.bevy_cams[cam_index].read_frame()   # Get a frame from Camera 0
        frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)                # Ensure correct color format

        # Extract points
        origin = tuple(projected_axis[0].ravel().astype(int))
        x_end  = tuple(projected_axis[1].ravel().astype(int))
        y_end  = tuple(projected_axis[2].ravel().astype(int))
        z_end  = tuple(projected_axis[3].ravel().astype(int))

        # p1  = tuple(projected_axis[4].ravel().astype(int))
        # p2  = tuple(projected_axis[5].ravel().astype(int))
        # p3  = tuple(projected_axis[6].ravel().astype(int))

        # 3. Draw axes from origin
        cv.line(frame, origin, x_end, (0,0,255), 1)  # X - Red
        cv.line(frame, origin, y_end, (0,255,0), 1)  # Y - Green
        cv.line(frame, origin, z_end, (255,0,0), 1)  # Z - Blue

        # # draw bs
        # cv.line(frame, p1, p2, (0,0,255), 1)  # X - Red
        # cv.line(frame, p2, p3, (0,255,0), 1)  # Y - Green
        # cv.line(frame, p3, p1, (255,0,0), 1)  # Z - Blue

        debug_frames.append(frame)
    # Show the frame (for debugging, or send it somewhere)
    joined_frames = np.hstack(debug_frames)
    cv.imshow("Pose Debug", joined_frames)
    cv.waitKey(0)
    cv.destroyAllWindows()
    


@socketio.on("triangulate-points")
def live_mocap(data):
    cameras = Cameras.instance()
    start_or_stop = data["startOrStop"]
    camera_poses = data["cameraPoses"]
    cameras.to_world_coords_matrix = data["toWorldCoordsMatrix"]

    
    # t0 = np.array(camera_poses[0]["t"])
    # t1 = np.array(camera_poses[1]["t"])
    # distance = np.linalg.norm(t0 - t1)
    # print(f"dist: {distance}")
    # print(f"ratio: {distance/4.186269175733933}")

    if (start_or_stop == "start"):
        cameras.start_trangulating_points(camera_poses)
        return
    elif (start_or_stop == "stop"):
        cameras.stop_trangulating_points()


if __name__ == '__main__':
    socketio.run(app, port=3001, debug=False)