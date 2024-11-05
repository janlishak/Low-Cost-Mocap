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

@socketio.on("calculate-camera-pose")
def calculate_camera_pose(data):
    cameras = Cameras.instance()
    image_points = np.array(data["cameraPoints"])
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
    actual_distance = 0.15
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


@socketio.on("triangulate-points")
def live_mocap(data):
    cameras = Cameras.instance()
    start_or_stop = data["startOrStop"]
    camera_poses = data["cameraPoses"]
    cameras.to_world_coords_matrix = data["toWorldCoordsMatrix"]

    if (start_or_stop == "start"):
        cameras.start_trangulating_points(camera_poses)
        return
    elif (start_or_stop == "stop"):
        cameras.stop_trangulating_points()


if __name__ == '__main__':
    socketio.run(app, port=3001, debug=False)