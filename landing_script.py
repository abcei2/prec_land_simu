import asyncio

import pygazebo
import io
import PIL
from PIL import Image
import numpy as np
import cv2

from pymavlink import mavutil
from cv2 import aruco
import yaml
import time
import math 

import threading
from aruco_utils import update_landing, rgb2gray

CONNECT_MAVLINK = True

#--------------- MAVLINK INIT SECTION -------------#
print("[INFO] connecting Mavlink...")
global hertz
if CONNECT_MAVLINK:
    drone = mavutil.mavlink_connection("udp:localhost:14551")
    hertz = 10 # betwwen 10 and 50 Hz
    ht = drone.wait_heartbeat()
    print(ht.to_json())
    initialLocation = drone.location()
else:
    fps = 10
    hertz = fps
#--------------- ARUCO TAG  INIT SECTION -------------#

# For validating results, show aruco board to camera.
aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_6X6_1000 )

arucoParams = aruco.DetectorParameters_create()

with open('calibration.yaml') as f:
    print("LOADED")
    loadeddict = yaml.load(f)

mtx = loadeddict.get('camera_matrix')
dist = loadeddict.get('dist_coeff')
mtx = np.array(mtx)
dist = np.array(dist)

global time_0, bussy_task
time_0 = time.time()

bussy_task = False


class BackgroundTasks(threading.Thread):

    def __init__(self,img):
        super(BackgroundTasks, self).__init__()
        self.image=img
    def run(self,*args,**kwargs):

        global time_0, bussy_task, hertz
        img=self.image
        if time.time() >= time_0 + 1.0/hertz:      
            img=Image.frombytes('RGB', (640,480), img, 'raw')
            img=np.array(img) 
            time_0 = time.time()
            im_gray = rgb2gray(img).astype(np.uint8)
            h,  w = im_gray.shape[:2]
            corners, ids, rejectedImgPoints = aruco.detectMarkers(im_gray, aruco_dict, parameters=arucoParams)
        
            img_aruco = img.copy()
            if len(corners)==0:
                pass
            else:
                img_aruco = aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))
                try:
                    if ids == None:
                        img_aruco = image.copy()
                        return
                except:
                    pass
                bigger_corner=[]
                bigger_id=0
                markerLength = 0
                for corner, id1 in zip(corners,ids):
                    bigger_corner = corner
                    bigger_id=id1
                    if id1==21: 
                        if markerLength>30 or markerLength==0:
                            markerLength=29.2
                    
                    
                    if id1==22: 
                        if markerLength>58.5 or markerLength==0:
                            markerLength=58.5

                    if id1==23: 
                        if markerLength>117.8  or markerLength==0:
                            markerLength=117.8 
                  
                     
                if markerLength ==0:
                    bussy_task = False
                    return                    
                
                rvec, tvec, _obj_points = aruco.estimatePoseSingleMarkers(corner, markerLength, mtx, dist)
                tvec = tvec[0][0]
                dist_to_marker_2d=math.sqrt(tvec[0]**2+tvec[1]**2)
                dist_to_marker = math.sqrt(tvec[0]**2+tvec[1]**2+tvec[2]**2 )
                angle_x = math.atan2(tvec[0],tvec[2])
                angle_y = math.atan2(tvec[1],tvec[2])

                print("id:",bigger_id[0],"x:",math.degrees(angle_x),"y:",math.degrees(angle_y), "dist_2d:",dist_to_marker_2d, "dist_3d:",dist_to_marker)
                 
                if CONNECT_MAVLINK:
                    update_landing(drone,initialLocation,angle_x,angle_y,tvec,dist_to_marker,hertz)
            
          
            if DISPLAY_RESULTS:        
                cv2.imshow("Aruco", img_aruco)
                cv2.waitKey(10)
        else:
            pass

        bussy_task = False



def retrieve_image(image):
    global bussy_task
    if not bussy_task:
        bussy_task = True                
        t = BackgroundTasks(image)
        t.start()
    

async def publish_loop():
    manager = await pygazebo.connect()
    
    publish_loop.is_waiting = True  

    subscriber=manager.subscribe('/gazebo/default/iris_demo/iris_demo/gimbal_small_2d/tilt_link/camera/image',
                    'gazebo.msgs.Image',
                    retrieve_image)
    returned = await subscriber.wait_for_connection()
    while (publish_loop.is_waiting):
        await asyncio.sleep(1.0)

   
loop = asyncio.get_event_loop()
loop.run_until_complete(publish_loop())