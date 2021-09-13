import cv2
from cv2 import aruco
import numpy as np
from pathlib import Path
import shutil
from tqdm import tqdm
import yaml
from pymavlink import mavutil
import math 
import time


def permutation(arr, n):
    # Set to check the count
    # of non-repeating elements
    s = set()
    maxEle = 0
    for i in range(n):
        # Insert all elements in the set
        s.add(arr[i])
        # Calculating the max element
        maxEle = max(maxEle, arr[i])
    if (maxEle != n):
        return False
    # Check if set size is equal to n
    if (len(s) == n):
        return True
    return False


def rgb2gray(rgb):
    return np.dot(rgb[..., :3], [0.2989, 0.5870, 0.1140])


def recreate_folder_data(path):

    main_path = Path(path)

    if main_path.exists() and main_path.is_dir():
        shutil.rmtree(main_path)
    main_path.mkdir(parents=True, exist_ok=True)


def detect_good_board_data(image, arucoParams, aruco_dict, horizontal_tag_ammount, vertical_tag_ammount, data_ammount, path, show_detections=False):

    name = path + str(data_ammount)+".jpg"
    img_gray = rgb2gray(image).astype(np.uint8)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        img_gray, aruco_dict, parameters=arucoParams)

    if len(corners) != horizontal_tag_ammount*vertical_tag_ammount:
        #print ("board is not detected correctly")
        pass
    elif corners == None or corners == []:
        #print ("pass")
        pass
    else:
        ids_list = ids.T[0]+1
        n = len(ids_list)
        print(n)
        if (permutation(ids_list, n)):
            print(name)
            cv2.imwrite(name, image)
            data_ammount = data_ammount+1
        img_aruco = aruco.drawDetectedMarkers(
            image.copy(), corners, ids, (0, 255, 0))
        if show_detections:
            cv2.imshow("image with board", img_aruco)
            cv2.waitKey(10)
        else:
            print("No", ids_list)
    return data_ammount


def calib_camera(arucoParams, aruco_dict, horizontal_tag_ammount, vertical_tag_ammount, markerLength, markerSeparation, path, file_name="calibration"):
    # create arUco board
    board = aruco.GridBoard_create(
        horizontal_tag_ammount, vertical_tag_ammount, markerLength, markerSeparation, aruco_dict)

    main_path = Path(path)

    img_list = []
    calib_fnms = main_path.glob('*.jpg')
    print('Using ...', calib_fnms, end='')
    for idx, fn in enumerate(calib_fnms):
        print(idx, '', end='')
        img = cv2.imread(str(fn))
        img_list.append(img)
        h, w, c = img.shape

    print('Calibration images')

    counter, corners_list, id_list = [], [], []
    first = True
    for im in tqdm(img_list):
        img_gray = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            img_gray, aruco_dict, parameters=arucoParams)
        if first == True:
            corners_list = corners
            id_list = ids
            first = False
        else:
            corners_list = np.vstack((corners_list, corners))
            id_list = np.vstack((id_list, ids))
        img_aruco = aruco.drawDetectedMarkers(im, corners, ids, (0, 255, 0))
        counter.append(len(ids))
    print('Found {} unique markers'.format(np.unique(ids)))

    counter = np.array(counter)
    print("Calibrating camera .... Please wait...")
    #mat = np.zeros((3,3), float)
    ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraAruco(
        corners_list, id_list, counter, board, img_gray.shape, None, None)

    print("Camera matrix is \n", mtx,
          "\n And is stored in calibration.yaml file along with distortion coefficients : \n", dist)

    data = {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(
        dist).tolist(), 'rvecs': np.asarray(rvecs).tolist(), 'tvecs': np.asarray(tvecs).tolist()}
    with open(file_name+".yaml", "w") as f:
        yaml.dump(data, f)


def update_landing_gps(drone, initialLocation, angle_x, angle_y, tvec,id_target, markerLength, rvec, time_capture=int(time.time()*1e6),dist_to_marker=0, hertz=5):
  
    globalPosInt = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=False)

    if globalPosInt:
        tvec[2]= globalPosInt.relative_alt
        time_capture = globalPosInt.time_boot_ms * 1000
        angle_x = math.atan2(tvec[0],tvec[2])
        angle_y = math.atan2(tvec[1],tvec[2])
        angle_x = math.atan2(tvec[0],dist_to_marker)
        angle_y = math.atan2(tvec[1],dist_to_marker)
        target_size_xy = math.atan2(markerLength,dist_to_marker)# Because is a square
        landing_target_send(drone,angle_x,angle_y,int(abs(dist_to_marker)/1000), time_capture, 0,0)
        distance_message(drone,int(dist_to_marker))
        print(math.degrees(angle_x),math.degrees(angle_y))


def update_landing_dist(drone, initialLocation, angle_x, angle_y, tvec,id_target, markerLength, rvec, time_capture=int(time.time()*1e6),dist_to_marker=0, hertz=5):

    dist_to_marker = math.sqrt(tvec[0]**2+tvec[1]**2+tvec[2]**2 )

    angle_x = math.atan2(tvec[0],dist_to_marker)
    angle_y = math.atan2(tvec[1],dist_to_marker)
    target_size_xy = math.atan2(markerLength,dist_to_marker)# Because is a square
    landing_target_send(drone,angle_x,angle_y,int(abs(dist_to_marker)/1000), time_capture, 0,0)
    distance_message(drone,int(dist_to_marker))
    print(math.degrees(angle_x),math.degrees(angle_y))

def update_landing(drone, initialLocation, angle_x, angle_y, tvec,id_target, markerLength, rvec, time_capture=int(time.time()*1e6),dist_to_marker=0, hertz=5):

    

    dist_to_marker = math.sqrt(tvec[0]**2+tvec[1]**2+tvec[2]**2 )

    angle_x = math.atan2(tvec[0],tvec[2])
    angle_y = math.atan2(tvec[1],tvec[2])
    target_size_xy = math.atan2(markerLength,dist_to_marker)# Because is a square
    landing_target_send(drone,angle_x,angle_y,int(abs(dist_to_marker)/1000), time_capture, 0,0)
    distance_message(drone,int(dist_to_marker))

def landing_target_send(drone,x,y,z, time_usec=0, target_num=0, target_size_xy=0):
    #print(x,y,z, time_usec, target_num, target_size_xy)
    drone.mav.landing_target_send(
        # globalPosInt.time_boot_ms * 1000,  # time in us since system boot
        time_usec,
        target_num,	
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        x,
        y,
        -z,  # distance drone-target
        target_size_xy,          # Target x-axis size, in radians
        target_size_xy          # Target y-axis size, in radians
    )
    
def landing_message(drone,x,y,z, time_usec=0, target_num=0):
    msg = drone.mav.message_factory.landing_target_encode(
        time_usec,          # time target data was processed, as close to sensor capture as possible
        target_num,          # target num, not used
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame, not used
        x,          # X-axis angular offset, in radians
        y,          # Y-axis angular offset, in radians
        z,          # distance, in meters
        0,          # Target x-axis size, in radians
        0,          # Target y-axis size, in radians
        0,          # x	float	X Position of the landing target on MAV_FRAME
        0,          # y	float	Y Position of the landing target on MAV_FRAME
        0,          # z	float	Z Position of the landing target on MAV_FRAME
        (1,0,0,0),  # q	float[4]	Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        2,          # type of landing target: 2 = Fiducial marker
        1,          # position_valid boolean
    )
    
    drone.mav.send_mavlink(msg)
    drone.mav.flush()

def distance_message(drone,dist):
    
    drone.mav.distance_sensor_send(
        0,          # time since system boot, not used
        1,          # min distance cm
        4000,      # max distance cm
        dist,       # current distance, must be int
        0,          # type = laser?
        0,          # onboard id, not used
        mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # must be set to MAV_SENSOR_ROTATION_PITCH_270 for mavlink rangefinder, represents downward facing
        0           # covariance, not used
    )
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).


def rotationMatrixToEulerAngles(R):

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])
