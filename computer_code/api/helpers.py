import numpy as np
from scipy import linalg, optimize, signal
import cv2 as cv
from scipy.spatial.transform import Rotation
import copy
import json
import os
import time
import numpy as np
import cv2 as cv
from KalmanFilter import KalmanFilter
import threading
# from pseyepy import Camera
from Singleton import Singleton

import serial
import threading
import eventlet

from mem import BevyCamera, GreenExampleCamera

class Camera:
    RES_SMALL = "RES_SMALL"
    def __init__(self, fps=60, resolution=RES_SMALL, gain=34, exposure=100):
        self.camera_count = 4
        self.exposure = [exposure] * 4
        self.gain = [gain] * 4
        self.bevy_cams = []

        for i in range(self.camera_count):
            bevy_camera= BevyCamera(f"cam{i}_frame")
            self.bevy_cams.append(bevy_camera)

    def read(self, indexes=None):
        if indexes is None:
            indexes = range(self.camera_count)
        frames = [self.bevy_cams[i].read_frame() for i in indexes]
        return frames, None

@Singleton
class Serial:
    def __init__(self):
        self.serialLock = threading.Lock()
        self.ser = None
        # self.ser = serial.Serial("/dev/cu.usbserial-02X2K2GE", 1000000, write_timeout=1)

    def write(self, data):
        if self.ser != None:
            with self.serialLock:
                self.ser.write(data)
                time.sleep(0.001)


@Singleton
class Cameras:
    def __init__(self):
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, "camera-params.json")
        f = open(filename)
        self.camera_params = json.load(f)

        self.cameras = Camera(fps=60, resolution=Camera.RES_SMALL, gain=34, exposure=100)
        self.num_cameras = len(self.cameras.exposure)
        # print(self.num_cameras)

        self.is_capturing_points = False
        self.is_triangulating_points = False
        self.camera_poses = None
        self.is_locating_objects = False
        self.to_world_coords_matrix = None
        self.drone_armed = []
        self.num_objects = None
        self.kalman_filter = None
        self.socketio = None
        self.serial = Serial.instance()

        self.old_format = False

        global cameras_init
        cameras_init = True

    def set_socketio(self, socketio):
        self.socketio = socketio
    
    def set_num_objects(self, num_objects):
        self.num_objects = num_objects
        self.drone_armed = [False for i in range(0, self.num_objects)]
    
    def edit_settings(self, exposure, gain):
        self.cameras.exposure = [exposure] * self.num_cameras
        self.cameras.gain = [gain] * self.num_cameras

    def _camera_read(self):
        frames, _ = self.cameras.read([0,1,2,3])

        for i in range(0, self.num_cameras):
            frames[i] = np.rot90(frames[i], k=self.camera_params[i]["rotation"])
            # frames[i] = make_square(frames[i])
            frames[i] = cv.undistort(frames[i], self.get_camera_params(i)["intrinsic_matrix"], self.get_camera_params(i)["distortion_coef"])
            frames[i] = cv.GaussianBlur(frames[i],(9,9),0)
            kernel = np.array([[-2,-1,-1,-1,-2],
                               [-1,1,3,1,-1],
                               [-1,3,4,3,-1],
                               [-1,1,3,1,-1],
                               [-2,-1,-1,-1,-2]])
            frames[i] = cv.filter2D(frames[i], -1, kernel)
            frames[i] = cv.cvtColor(frames[i], cv.COLOR_RGB2BGR)

        if (self.is_capturing_points):
            image_points = []
            for i in range(0, self.num_cameras):
                frames[i], single_camera_image_points = self._find_dot(frames[i])
                image_points.append(single_camera_image_points)
            
            if (any(np.all(point[0] != [None,None]) for point in image_points)):
                if self.is_capturing_points and not self.is_triangulating_points:
                    if all(np.all(point[0] != [None, None]) for point in image_points):
                        if self.old_format:
                            self.socketio.emit("image-points", [x[0] for x in image_points]) # if multiple points detected only send the first
                        else:
                            self.socketio.emit("image-points", image_points) # send all points
                elif self.is_triangulating_points:
                    # errors, object_points, fraes = find_point_correspondance_and_object_points(image_points, self.camera_poses, frames)
                    return find_epilines(image_points, self.camera_poses, frames)

                    # convert to world coordinates
                    for i, object_point in enumerate(object_points):
                        new_object_point = np.array([[-1,0,0],[0,-1,0],[0,0,1]]) @ object_point
                        new_object_point = np.concatenate((new_object_point, [1]))
                        new_object_point = np.array(self.to_world_coords_matrix) @ new_object_point
                        new_object_point = new_object_point[:3] / new_object_point[3]
                        new_object_point[1], new_object_point[2] = new_object_point[2], new_object_point[1]
                        object_points[i] = new_object_point

                    objects = []
                    filtered_objects = []
                    if self.is_locating_objects:
                        objects = locate_objects(object_points, errors)
                        filtered_objects = self.kalman_filter.predict_location(objects)
                        
                        if len(filtered_objects) != 0:
                            for filtered_object in filtered_objects:
                                if self.drone_armed[filtered_object['droneIndex']]:
                                    filtered_object["heading"] = round(filtered_object["heading"], 4)

                                    serial_data = { 
                                        "pos": [round(x, 4) for x in filtered_object["pos"].tolist()] + [filtered_object["heading"]],
                                        "vel": [round(x, 4) for x in filtered_object["vel"].tolist()]
                                    }
                                    self.serial.write(f"{filtered_object['droneIndex']}{json.dumps(serial_data)}".encode('utf-8'))
                            
                        for filtered_object in filtered_objects:
                            filtered_object["vel"] = filtered_object["vel"].tolist()
                            filtered_object["pos"] = filtered_object["pos"].tolist()
                    

                    # DEFAULT_TRIANGLE_POINTS = [[-0.1, 0, 0],[0.1, 0, 0], [0, 0, 0.1]]
                    # if(len(object_points)>0):
                    #     DEFAULT_TRIANGLE_POINTS[2] = object_points[0].tolist()
                    # self.socketio.emit("triangle-points", {'triangle_points': DEFAULT_TRIANGLE_POINTS})

                    self.socketio.emit("object-points", {
                        "object_points": object_points.tolist(), 
                        "errors": errors.tolist(), 
                        "objects": [{k:(v.tolist() if isinstance(v, np.ndarray) else v) for (k,v) in object.items()} for object in objects], 
                        "filtered_objects": filtered_objects
                    })
        
        return frames


    def get_frames(self):
        frames = self._camera_read()
        # frames = [add_white_border(frame, 5) for frame in frames]

        return np.hstack(frames)

    def _find_dot(self, img):
        # img = cv.GaussianBlur(img,(5,5),0)
        grey = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
        grey = cv.threshold(grey, 255*0.2, 255, cv.THRESH_BINARY)[1]
        contours,_ = cv.findContours(grey, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        img = cv.drawContours(img, contours, -1, (0,255,0), 1)

        image_points = []
        for contour in contours:
            moments = cv.moments(contour)
            if moments["m00"] != 0:
                center_x = int(moments["m10"] / moments["m00"])
                center_y = int(moments["m01"] / moments["m00"])
                cv.putText(img, f'({center_x}, {center_y})', (center_x,center_y - 15), cv.FONT_HERSHEY_SIMPLEX, 0.3, (100,255,100), 1)
                cv.circle(img, (center_x,center_y), 1, (100,255,100), -1)
                image_points.append([center_x, center_y])

        if len(image_points) == 0:
            image_points = [[None, None]]

        return img, image_points

    def start_capturing_points(self):
        self.is_capturing_points = True

    def stop_capturing_points(self):
        self.is_capturing_points = False

    def start_trangulating_points(self, camera_poses):
        self.is_capturing_points = True
        self.is_triangulating_points = True
        self.camera_poses = camera_poses
        self.kalman_filter = KalmanFilter(self.num_objects)

    def stop_trangulating_points(self):
        self.is_capturing_points = False
        self.is_triangulating_points = False
        self.camera_poses = None

    def start_locating_objects(self):
        self.is_locating_objects = True

    def stop_locating_objects(self):
        self.is_locating_objects = False
    
    def get_camera_params(self, camera_num):
        return {
            "intrinsic_matrix": np.array(self.camera_params[camera_num]["intrinsic_matrix"]),
            "distortion_coef": np.array(self.camera_params[camera_num]["distortion_coef"]),
            "rotation": self.camera_params[camera_num]["rotation"]
        }
    
    def set_camera_params(self, camera_num, intrinsic_matrix=None, distortion_coef=None):
        if intrinsic_matrix is not None:
            self.camera_params[camera_num]["intrinsic_matrix"] = intrinsic_matrix
        
        if distortion_coef is not None:
            self.camera_params[camera_num]["distortion_coef"] = distortion_coef


def calculate_reprojection_errors(image_points, object_points, camera_poses):
    errors = np.array([])
    for image_points_i, object_point in zip(image_points, object_points):
        error = calculate_reprojection_error(image_points_i, object_point, camera_poses)
        if error is None:
            continue
        errors = np.concatenate([errors, [error]])

    return errors


def calculate_reprojection_error(image_points, object_point, camera_poses):
    cameras = Cameras.instance()

    image_points = np.array(image_points)
    none_indicies = np.where(np.all(image_points == None, axis=1))[0]
    image_points = np.delete(image_points, none_indicies, axis=0)
    camera_poses = np.delete(camera_poses, none_indicies, axis=0)

    if len(image_points) <= 1:
        return None

    image_points_t = image_points.transpose((0,1))

    errors = np.array([])
    for i, camera_pose in enumerate(camera_poses):
        if np.all(image_points[i] == None, axis=0):
            continue
        projected_img_points, _ = cv.projectPoints(
            np.expand_dims(object_point, axis=0).astype(np.float32), 
            np.array(camera_pose["R"], dtype=np.float64), 
            np.array(camera_pose["t"], dtype=np.float64), 
            cameras.get_camera_params(i)["intrinsic_matrix"], 
            np.array([])
        )
        projected_img_point = projected_img_points[:,0,:][0]
        errors = np.concatenate([errors, (image_points_t[i]-projected_img_point).flatten() ** 2])
    
    return errors.mean()


def bundle_adjustment(image_points, camera_poses, socketio):
    print("BA", camera_poses)
    cameras = Cameras.instance()

    def params_to_camera_poses(params):
        focal_distances = []
        num_cameras = int((params.size-1)/7)+1
        camera_poses = [{
            "R": np.eye(3),
            "t": np.array([0,0,0], dtype=np.float32)
        }]
        focal_distances.append(params[0])
        for i in range(0, num_cameras-1):
            focal_distances.append(params[i*7+1])
            camera_poses.append({
                "R": Rotation.as_matrix(Rotation.from_rotvec(params[i*7 + 2 : i*7 + 3 + 2])),
                "t": params[i*7 + 3 + 2 : i*7 + 6 + 2]
            })

        return camera_poses, focal_distances

    def residual_function(params):
        camera_poses, focal_distances = params_to_camera_poses(params)
        for i in range(0, len(camera_poses)):
            intrinsic = cameras.get_camera_params(i)["intrinsic_matrix"]
            intrinsic[0, 0] = focal_distances[i]
            intrinsic[1, 1] = focal_distances[i]
            # cameras.set_camera_params(i, intrinsic)
        object_points = triangulate_points(image_points, camera_poses)
        errors = calculate_reprojection_errors(image_points, object_points, camera_poses)
        errors = errors.astype(np.float32)
        socketio.emit("camera-pose", {"camera_poses": camera_pose_to_serializable(camera_poses)})
        eventlet.sleep(0)        
        return errors

    focal_distance = cameras.get_camera_params(0)["intrinsic_matrix"][0,0]
    init_params = np.array([focal_distance])
    for i, camera_pose in enumerate(camera_poses[1:]):
        rot_vec = Rotation.as_rotvec(Rotation.from_matrix(camera_pose["R"])).flatten()
        focal_distance = cameras.get_camera_params(i)["intrinsic_matrix"][0,0]
        init_params = np.concatenate([init_params, [focal_distance]])
        init_params = np.concatenate([init_params, rot_vec])
        init_params = np.concatenate([init_params, camera_pose["t"].flatten()])

    print(init_params)


    res = optimize.least_squares(
        residual_function, init_params, verbose=2, loss="cauchy", ftol=1E-2
    )
    return params_to_camera_poses(res.x)[0]
    

def triangulate_point(image_points, camera_poses):
    image_points = np.array(image_points)
    cameras = Cameras.instance()
    none_indicies = np.where(np.all(image_points == None, axis=1))[0]
    image_points = np.delete(image_points, none_indicies, axis=0)
    camera_poses = np.delete(camera_poses, none_indicies, axis=0)

    if len(image_points) <= 1:
        return [None, None, None]

    Ps = [] # projection matricies

    for i, camera_pose in enumerate(camera_poses):
        RT = np.c_[camera_pose["R"], camera_pose["t"]]
        P = cameras.camera_params[i]["intrinsic_matrix"] @ RT
        Ps.append(P)

    # https://temugeb.github.io/computer_vision/2021/02/06/direct-linear-transorms.html
    def DLT(Ps, image_points):
        A = []

        for P, image_point in zip(Ps, image_points):
            A.append(image_point[1]*P[2,:] - P[1,:])
            A.append(P[0,:] - image_point[0]*P[2,:])
            
        A = np.array(A).reshape((len(Ps)*2,4))
        B = A.transpose() @ A
        U, s, Vh = linalg.svd(B, full_matrices = False)
        object_point = Vh[3,0:3]/Vh[3,3]

        return object_point

    object_point = DLT(Ps, image_points)

    return object_point


def triangulate_points(image_points, camera_poses):
    object_points = []
    for image_points_i in image_points:
        object_point = triangulate_point(image_points_i, camera_poses)
        object_points.append(object_point)
    
    return np.array(object_points)

# my shit

def skew(t):
    return np.array([
        [0, -t[2], t[1]],
        [t[2], 0, -t[0]],
        [-t[1], t[0], 0]
    ])


def draw_lines(img1, img2, lines, pts1, pts2):
    h, w = img1.shape[:2]
    for r, pt1, pt2 in zip(lines, pts1, pts2):
        color = tuple(np.random.randint(0, 255, 3).tolist())
        x0, y0 = map(int, [0, -r[2] / r[1]])
        x1, y1 = map(int, [w, -(r[2] + r[0] * w) / r[1]])
        cv.line(img2, (x0, y0), (x1, y1), color, 1)
        cv.circle(img1, tuple(np.round(pt1).astype(int)), 5, color, -1)
        cv.circle(img2, tuple(np.round(pt2).astype(int)), 5, color, -1)
    return img1, img2


def find_epilines(image_points, camera_poses, frames):
    cameras = Cameras.instance()
    K = np.array(cameras.get_camera_params(0)["intrinsic_matrix"])

    # Camera-to-world poses
    R0_c2w = np.array(camera_poses[0]["R"])
    t0_c2w = np.array(camera_poses[0]["t"])
    R1_c2w = np.array(camera_poses[1]["R"])
    t1_c2w = np.array(camera_poses[1]["t"])

    # Convert to world-to-camera
    R0 = R0_c2w.T
    t0 = -R0 @ t0_c2w
    R1 = R1_c2w.T
    t1 = -R1 @ t1_c2w

    # Relative pose from cam0 to cam1 (in cam0 frame)
    R_rel = R1 @ R0.T
    t_rel = t1 - R_rel @ t0

    # Compute essential and fundamental matrices
    E = skew(t_rel.flatten()) @ R_rel
    F = np.linalg.inv(K).T @ E @ np.linalg.inv(K)

    # Prepare matched points
    pts0 = np.array(image_points[0], dtype=np.float32).reshape(-1, 1, 2)
    pts1 = np.array(image_points[1], dtype=np.float32).reshape(-1, 1, 2)

    # Compute epilines in cam1 for pts0
    lines1 = cv.computeCorrespondEpilines(pts0, 1, F).reshape(-1, 3)
    frames[0], frames[1] = draw_lines(frames[0], frames[1], lines1, pts0.reshape(-1, 2), pts1.reshape(-1, 2))

    # Compute epilines in cam0 for pts1
    lines0 = cv.computeCorrespondEpilines(pts1, 2, F).reshape(-1, 3)
    frames[1], frames[0] = draw_lines(frames[1], frames[0], lines0, pts1.reshape(-1, 2), pts0.reshape(-1, 2))

    return frames

    # lines = np.array([[0.01, -1.0, 100], [-0.02, -1.0, 300]], dtype=np.float32)

    # # Simulate corresponding points
    # pts1 = np.array([[100, 200], [300, 400]], dtype=np.float32)
    # pts2 = np.array([[120, 210], [320, 410]], dtype=np.float32)

    # frames[0], frames[1], = drawlines(frames[0], frames[1], lines, pts1, pts2)

    # cameras = Cameras.instance()
    # K = np.array(cameras.get_camera_params(0)["intrinsic_matrix"])

    # # Get poses for camera 0 and 1
    # R1 = np.array(camera_poses[0]["R"])
    # t1 = np.array(camera_poses[0]["t"])
    # R2 = np.array(camera_poses[1]["R"])
    # t2 = np.array(camera_poses[1]["t"])

    # # Relative pose from cam 0 to cam 1
    # R = R2 @ R1.T
    # t = t2 - R @ t1
    # t_x = skew(t.flatten())

    # # Compute E and F
    # E = t_x @ R
    # F = np.linalg.inv(K).T @ E @ np.linalg.inv(K)

    # # Prepare point data
    # pts0 = np.array(image_points[0], dtype=np.float32).reshape(-1, 1, 2)
    # pts1 = np.array(image_points[1], dtype=np.float32).reshape(-1, 1, 2)

    # # Epilines in cam1 for points in cam0
    # lines1 = cv.computeCorrespondEpilines(pts0, 1, F).reshape(-1, 3)
    # frames[0], frames[1] = drawlines(frames[0], frames[1], lines1, pts0.reshape(-1, 2), pts1.reshape(-1, 2))

    # # Epilines in cam0 for points in cam1
    # lines0 = cv.computeCorrespondEpilines(pts1, 2, F).reshape(-1, 3)
    # frames[1], frames[0] = drawlines(frames[1], frames[0], lines0, pts1.reshape(-1, 2), pts0.reshape(-1, 2))

   

    # OLD VERSION 
    #
    # for image_points_i in image_points:
    #     try:
    #         image_points_i.remove([None, None])
    #     except:
    #         pass

    # # [object_points, possible image_point groups, image_point from camera]
    # correspondances = [[[i]] for i in image_points[0]]


    # Ps = [] # projection matricies
    # for i, camera_pose in enumerate(camera_poses):
    #     RT = np.c_[camera_pose["R"], camera_pose["t"]]
    #     P = cameras.camera_params[i]["intrinsic_matrix"] @ RT
    #     Ps.append(P)

    # root_image_points = [{"camera": 0, "point": point} for point in image_points[0]]

    # for i in range(1, len(camera_poses)):
    #     epipolar_lines = []
    #     for root_image_point in root_image_points:
    #         F = cv.sfm.fundamentalFromProjections(Ps[root_image_point["camera"]], Ps[i])
    #         line = cv.computeCorrespondEpilines(np.array([root_image_point["point"]], dtype=np.float32), 1, F)
    #         epipolar_lines.append(line[0,0].tolist())
    #         frames[i] = drawlines(frames[i], line[0])
    # return frames

def find_point_correspondance_and_object_points(image_points, camera_poses, frames):
    cameras = Cameras.instance()

    for image_points_i in image_points:
        try:
            image_points_i.remove([None, None])
        except:
            pass

    # [object_points, possible image_point groups, image_point from camera]
    correspondances = [[[i]] for i in image_points[0]]

    Ps = [] # projection matricies
    for i, camera_pose in enumerate(camera_poses):
        RT = np.c_[camera_pose["R"], camera_pose["t"]]
        P = cameras.camera_params[i]["intrinsic_matrix"] @ RT
        Ps.append(P)

    root_image_points = [{"camera": 0, "point": point} for point in image_points[0]]

    for i in range(1, len(camera_poses)):
        epipolar_lines = []
        for root_image_point in root_image_points:
            F = cv.sfm.fundamentalFromProjections(Ps[root_image_point["camera"]], Ps[i])
            line = cv.computeCorrespondEpilines(np.array([root_image_point["point"]], dtype=np.float32), 1, F)
            epipolar_lines.append(line[0,0].tolist())
            frames[i] = drawlines(frames[i], line[0])

        not_closest_match_image_points = np.array(image_points[i])
        points = np.array(image_points[i])

        for j, [a, b, c] in enumerate(epipolar_lines):
            distances_to_line = np.array([])
            if len(points) != 0:
                distances_to_line = np.abs(a*points[:,0] + b*points[:,1] + c) / np.sqrt(a**2 + b**2)

            possible_matches = points[distances_to_line < 15].copy()

            # Commenting out this code produces more points, but more garbage points too
            # delete closest match from future consideration
            # if len(points) != 0:
            #     points = np.delete(points, np.argmin(distances_to_line), axis=0)

            # sort possible matches from smallest to largest
            distances_to_line = distances_to_line[distances_to_line < 15]
            possible_matches_sorter = distances_to_line.argsort()
            possible_matches = possible_matches[possible_matches_sorter]
    
            if len(possible_matches) == 0:
                for possible_group in correspondances[j]:
                    possible_group.append([None, None])
            else:
                not_closest_match_image_points = [row for row in not_closest_match_image_points.tolist() if row != possible_matches.tolist()[0]]
                not_closest_match_image_points = np.array(not_closest_match_image_points)
                
                new_correspondances_j = []
                for possible_match in possible_matches:
                    temp = copy.deepcopy(correspondances[j])
                    for possible_group in temp:
                        possible_group.append(possible_match.tolist())
                    new_correspondances_j += temp
                correspondances[j] = new_correspondances_j

        for not_closest_match_image_point in not_closest_match_image_points:
            root_image_points.append({"camera": i, "point": not_closest_match_image_point})
            temp = [[[None, None]] * i]
            temp[0].append(not_closest_match_image_point.tolist())
            correspondances.append(temp)

    object_points = []
    errors = []
    for image_points in correspondances:
        object_points_i = triangulate_points(image_points, camera_poses)

        if np.all(object_points_i == None):
            continue

        errors_i = calculate_reprojection_errors(image_points, object_points_i, camera_poses)

        object_points.append(object_points_i[np.argmin(errors_i)])
        errors.append(np.min(errors_i))

    return np.array(errors), np.array(object_points), frames


def locate_objects(object_points, errors):
    dist1 = 0.095
    dist2 = 0.15

    distance_matrix = np.zeros((object_points.shape[0], object_points.shape[0]))
    already_matched_points = []
    objects = []

    for i in range(0, object_points.shape[0]):
        for j in range(0, object_points.shape[0]):
            distance_matrix[i,j] = np.sqrt(np.sum((object_points[i] - object_points[j])**2))

    for i in range(0, object_points.shape[0]):
        if i in already_matched_points:
            continue
        
        distance_deltas = np.abs(distance_matrix[i] - dist1)
        num_matches = distance_deltas < 0.025
        matches_index = np.where(distance_deltas < 0.025)[0]
        if np.sum(num_matches) >= 2:
            for possible_pair in cartesian_product(matches_index, matches_index):
                pair_distance = np.sqrt(np.sum((object_points[possible_pair[0]] - object_points[possible_pair[1]])**2))
                
                # if the pair isnt the correct distance apart
                if np.abs(pair_distance - dist2) > 0.025:
                    continue

                best_match_1_i = possible_pair[0]
                best_match_2_i = possible_pair[1]

                already_matched_points.append(i)
                already_matched_points.append(best_match_1_i)
                already_matched_points.append(best_match_2_i)

                location = (object_points[best_match_1_i]+object_points[best_match_2_i])/2
                error = np.mean([errors[i], errors[best_match_1_i], errors[best_match_2_i]])

                heading_vec = object_points[best_match_1_i] - object_points[best_match_2_i]
                heading_vec /= linalg.norm(heading_vec)
                heading = np.arctan2(heading_vec[1], heading_vec[0])

                heading = heading - np.pi if heading > np.pi/2 else heading
                heading = heading + np.pi if heading < -np.pi/2 else heading

                # determine drone index based on which side third light is on
                drone_index = 0 if (object_points[i] - location)[1] > 0 else 1

                objects.append({
                    "pos": location,
                    "heading": -heading,
                    "error": error,
                    "droneIndex": drone_index
                })

                break
    
    return objects


def numpy_fillna(data):
    data = np.array(data, dtype=object)
    # Get lengths of each row of data
    lens = np.array([len(i) for i in data])

    # Mask of valid places in each row
    mask = np.arange(lens.max()) < lens[:,None]

    # Setup output array and put elements from data into masked positions
    out = np.full((mask.shape[0], mask.shape[1], 2), [None, None])
    out[mask] = np.concatenate(data)
    return out
        

def drawlines(img1,lines):
    r,c,_ = img1.shape
    for r in lines:
        color = tuple(np.random.randint(0,255,3).tolist())
        x0,y0 = map(int, [0, -r[2]/r[1] ])
        x1,y1 = map(int, [c, -(r[2]+r[0]*c)/r[1] ])
        img1 = cv.line(img1, (x0,y0), (x1,y1), color,1)
    return img1


def make_square(img):
    x, y, _ = img.shape
    size = max(x, y)
    new_img = np.zeros((size, size, 3), dtype=np.uint8)
    ax,ay = (size - img.shape[1])//2,(size - img.shape[0])//2
    new_img[ay:img.shape[0]+ay,ax:ax+img.shape[1]] = img

    # Pad the new_img array with edge pixel values
    # Apply feathering effect
    feather_pixels = 8
    for i in range(feather_pixels):
        alpha = (i + 1) / feather_pixels
        new_img[ay - i - 1, :] = img[0, :] * (1 - alpha)  # Top edge
        new_img[ay + img.shape[0] + i, :] = img[-1, :] * (1 - alpha)  # Bottom edge


    return new_img


def camera_pose_to_serializable(camera_poses):
    for i in range(0, len(camera_poses)):
        camera_poses[i] = {k: v.tolist() for (k, v) in camera_poses[i].items()}

    return camera_poses

def cartesian_product(x, y):
    return np.array([[x0, y0] for x0 in x for y0 in y])

def add_white_border(image, border_size):
    height, width = image.shape[:2]
    bordered_image = cv.copyMakeBorder(image, border_size, border_size, border_size, border_size, cv.BORDER_CONSTANT, value=[255, 255, 255])
    return bordered_image