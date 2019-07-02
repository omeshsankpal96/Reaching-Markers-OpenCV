import os
import cv2
from cv2 import aruco
import numpy as np
import yaml
from picamera.array import PiRGBArray
from picamera import PiCamera
import time, math
import RPi.GPIO as GPIO
from AlphaBot import AlphaBot

with open('calibration.yaml') as f:
    loadeddict = yaml.load(f)

camera_matrix = loadeddict.get('camera_matrix')
dist_coeffs = loadeddict.get('dist_coeff')

camera_matrix = np.matrix(camera_matrix)
dist_coeffs = np.matrix(dist_coeffs)

image_size = (640, 480)
map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, None, image_size, cv2.CV_16SC2)

aruco_dict = aruco.Dictionary_get( aruco.DICT_6X6_1000 )

markerLength = 10

arucoParams = aruco.DetectorParameters_create()

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 90
rawCapture = PiRGBArray(camera, size=(640, 480))
Ab = AlphaBot();
Ab.stop();
# Initialization
cntl = 8;
cntr = 7;
EncR = 0.0;
EncL = 0.0;
xx = 0;
yy = 0;
theta = 0;
wheel_R = 1.25
wheel_Dia = 2*wheel_R;                 # radius of the wheels
wheel_circum = math.pi*wheel_Dia;     # circumference of the wheels

# Left encoder increment
def updateEncoderL(channel):
    global EncL;
    EncL += 1;
    #print 'EncL'


# Right encoder increment
def updateEncoderR(channel):
    global EncR;
    EncR += 1;
    #print 'EncR'


# Function to define x and y co-ordinates
def get_x_y(avg_value, r_enc, l_enc):
    global theta, xx, yy, theta_dot, wheel_circum;
    print("Initial Co-ordinates(x,y): ({0}, {1})" .format(xx, yy))
    enc_val = (r_enc+l_enc)/2
    xx = ((enc_val*wheel_circum)/40)*math.cos(theta) + xx 
    yy = ((enc_val*wheel_circum)/40)*math.sin(theta) + yy 
    theta_dot = 0
    print("Theta: %f" %theta)
    print(" Final Co-ordinates(x,y): ({0}, {1})" .format(xx, yy))
    

# Function to define the rotation value: theta
def get_theta(avg_value, r_enc, l_enc, flag):
    global theta, theta_dot
    theta_dot = 4.5*((r_enc+l_enc)/2)
    theta = theta_dot + theta
    theta = theta % 360;
    print("Theta at that instant: %f" %theta_dot)
    print("Sum of rotations - theta : %f" %theta)


def rotation(angle):
    R0 = EncR
    L0 = EncL
    flag = False
    # Anti-clockwise direction
    if(angle > 0):
        enc_val = angle*0.15 # rotation based on encoder value
        # Actual:angle*0.22   Improvised to reduce the error
        while((EncL-L0)+(EncR-R0))/2 < enc_val:
            Ab.setMotor(-35, -35);
        print('r_enc = {0} l_enc = {1}' .format((EncR-R0), (EncL-L0)));
        flag = False
        
    elif(angle < 0):# Clockwise direction
        angle = angle*-1
        enc_val = angle*0.18 # rotation based on encoder value
        # Actual angle: angle*0.22    Improvised to reduce the error
        while((EncL-L0)+(EncR-R0))/2 < enc_val:
            Ab.setMotor(35, 35);
        print('r_enc = {0} l_enc = {1}' .format((EncR-R0), (EncL-L0)));
        flag = True
    Ab.stop()
    time.sleep(0.5)
    get_theta(enc_val, (EncR-R0), (EncL-L0), flag)
    print('r_enc = {0} l_enc = {1}' .format((EncR-R0), (EncL-L0)));
    time.sleep(0.5)

def distance(distance):
    global wheel_circum;
    distance = distance*0.5;
    enc_val= (40/wheel_circum)*distance # Distance based on encoder values
    R0 = EncR
    L0 = EncL
    while(EncR-R0) < enc_val:
       if ((EncL-L0)+(EncR-R0))/2 < enc_val:
           Ab.setMotor(-38, 40);
    Ab.stop()
    time.sleep(0.5)
    get_x_y(enc_val, (EncR-R0), (EncL-L0))
    print('r_enc = {0} l_enc = {1}' .format((EncR-R0), (EncL-L0)));
    time.sleep(0.5)


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False);
GPIO.setup(cntr, GPIO.IN);
GPIO.setup(cntl, GPIO.IN);
GPIO.add_event_detect(cntr, GPIO.BOTH, updateEncoderR)
GPIO.add_event_detect(cntl, GPIO.BOTH, updateEncoderL)


time.sleep(0.1)

Ab.stop();

R_flip = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] = -1.0
R_flip[2,2] = -1.0

font = cv2.FONT_HERSHEY_PLAIN
id_to_find = 0
while(True): 
    rotation(3);
    time.sleep(0.1);

    camera.capture(rawCapture, format="bgr")
    img = rawCapture.array
    rawCapture.truncate(0)
    imgRemapped = cv2.remap(img, map1, map2, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
    imgRemapped_gray = cv2.cvtColor(imgRemapped, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(imgRemapped_gray, aruco_dict, parameters=arucoParams)
    
    print(ids)
    if np.all(ids == id_to_find):
        EncR = 0;
        EncL = 0
        Ab.stop();
        ret = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
        #ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
        rvec = ret[0][0,0,:]
        tvec = ret[1][0,0,:]
        
        aruco.drawDetectedMarkers(frame, corners)
        aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)
        
        str_position = "MARKER Position x=%4.0f y=%4.0f z=%4.0f"%(tvec[0], tvec[1], tvec[2])
        cv2.putText(frame, strPosition, (0,100), font, 1, (0,255,0), 2, cv2.LINE_AA)
        destination = float(tvec[2])

        
        enc_val = (destination/4) +4
        distance(enc_val)
        print (ids)

        id_to_find = id_to_find + 1;
        print(id_to_find)
        time.sleep(2);
        
    else:
        imgWithAruco = imgRemapped 

    cv2.imshow("aruco", imgWithAruco)
    if id_to_find > 7:
        break
    if cv2.waitKey(2) & 0xFF == ord('q'):
        break
