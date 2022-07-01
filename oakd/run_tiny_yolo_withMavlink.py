#!/usr/bin/env python3

"""
The code is the same as for Tiny Yolo V3 and V4, the only difference is the blob file
- Tiny YOLOv3: https://github.com/david8862/keras-YOLOv3-model-set
- Tiny YOLOv4: https://github.com/TNTWEN/OpenVINO-YOLOV4
"""

from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time

import threading
#-------------------------------------------------set get gps-------------------------------------------------------------------------------------------
import os, socket, select, subprocess, struct
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

#global mav_master
#global mav_modes

cur_uav_latitude = 0
cur_uav_longitude = 0
cur_uav_altitude = 0
cur_uav_satellites = 0
cur_uav_gps_ground_speed = 0
noGCS_flag=1
gps_lat=0
gps_long=0
lat_aver = 0
long_aver=0

class nothing(object):
    def __init__(self):
        return
    def write(self, data):
        return len(data)
    def read(self):
        return []

def init_mav():
    compid = 1
    #mav_master = mavutil.mavlink_connection(device="/dev/ttyACM0", baud=57600, source_system=255)
    mav_master = mavutil.mavlink_connection(device="tcp:127.0.0.1:5762", baud=115200, source_system=255)
    print ("Waiting for APM heartbeat")
    while True:
        hb = mav_master.recv_match(type='HEARTBEAT', blocking=True)
        if hb.type != mavutil.mavlink.MAV_TYPE_GCS:
            print ('Heartbeat from APM system %d' % mav_master.target_system)
            break
    mav_modes = mav_master.mode_mapping()
    inject_mav = mavlink.MAVLink(nothing(), mav_master.target_system, compid)
    return (mav_master, mav_modes)

def gps_get():
    global cur_uav_latitude
    global cur_uav_longitude
    global cur_uav_altitude
    global cur_uav_satellites
    global cur_uav_gps_ground_speed
    global noGCS_flag
    gps_pos_flag=0
    # receive message from UAV
    while True:
        cur_ts = time.time()
        msg = mav_master.recv_msg()
        if msg is not None and msg.get_type() != "BAD_DATA":
            msg_id = msg.get_msgId()
            #print ('(msg_id from UAV:%d)' % (msg_id))

            #if msg.get_type() == "GLOBAL_POSITION_INT":
            if msg_id == 33:
                cur_uav_altitude = msg.relative_alt/1000.0
                #print "Relative Altitude=%.7f (M)" % (cur_uav_altitude)

            #if msg.get_type() == "GPS_RAW_INT":
            if msg_id == 24:
                cur_uav_latitude = msg.lat/10000000.0
                cur_uav_longitude = msg.lon/10000000.0
                cur_uav_satellites = msg.satellites_visible
                cur_uav_gps_ground_speed = msg.vel/100.0
                gps_pos_flag=1

        if noGCS_flag == 1:
            noGCS_flag += 1
            #send request_data_stream to uav
            print ("send request_data_stream to UAV [flag : %d]" % (noGCS_flag))
            mav_master.mav.request_data_stream_send(mav_master.target_system, mav_master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10 ,1)
        #print("cuv_lat=", cur_uav_latitude , " cuv_long=",cur_uav_longitude)
        continue
    #return(cur_uav_latitude,cur_uav_longitude)

mav_master, mav_modes = init_mav()
#-------------------------------------------------set get gps-------------------------------------------------------------------------------------------
t = threading.Thread(target=gps_get)
t.start()

# clear file content
with open("test_gps.csv", 'w+', encoding = 'utf-8') as f:
    f.truncate()

# nnPath = '/Users/tangtang/Code/OpenVINO-YOLOV4/frozen_darknet_yolov4_model_openvino_2021.4_6shave.blob'
# nnPath = '/Users/tangtang/Code/oakd/tiny-yolo-v4_openvino_2021.2_6shave.blob'
# nnPath = '/Users/tangtang/Code/tensorflow-yolov4-tiny/yolov4-tiny_openvino_2021.4_6shave.blob'
nnPath = './yolov4-tiny-pier_last_openvino_2021.4_6shave.blob'

if not Path(nnPath).exists():
    import sys
    raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

# tiny yolo v4 label texts
# labelMap = [
#     "person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
#     "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
#     "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant",
#     "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
#     "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
#     "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
#     "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
#     "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
#     "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
#     "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
#     "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
#     "teddy bear",     "hair drier", "toothbrush"
# ]

labelMap = ["pier"]

syncNN = True

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
detectionNetwork = pipeline.create(dai.node.YoloDetectionNetwork)
xoutRgb = pipeline.create(dai.node.XLinkOut)
nnOut = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")
nnOut.setStreamName("nn")

# Properties
camRgb.setPreviewSize(416, 416)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
camRgb.setFps(40)

# Network specific settings
detectionNetwork.setConfidenceThreshold(0.5)
detectionNetwork.setNumClasses(1)
detectionNetwork.setCoordinateSize(4)
detectionNetwork.setAnchors(np.array([10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319]))
detectionNetwork.setAnchorMasks({"side26": np.array([1, 2, 3]), "side13": np.array([3, 4, 5])})
detectionNetwork.setIouThreshold(0.5)
detectionNetwork.setBlobPath(nnPath)
detectionNetwork.setNumInferenceThreads(2)
detectionNetwork.input.setBlocking(False)

# Linking
camRgb.preview.link(detectionNetwork.input)
if syncNN:
    detectionNetwork.passthrough.link(xoutRgb.input)
else:
    camRgb.preview.link(xoutRgb.input)

detectionNetwork.out.link(nnOut.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queues will be used to get the rgb frames and nn data from the outputs defined above
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    qDet = device.getOutputQueue(name="nn", maxSize=4, blocking=False)

    frame = None
    detections = []
    startTime = time.monotonic()
    counter = 0
    color2 = (255, 255, 255)

    gps_lat_aver = []  # aver gps position
    gps_long_aver = []  
    # save video set
    fourcc=cv2.VideoWriter_fourcc(*'XVID')
    save_original=cv2.VideoWriter("original video.avi",fourcc,30,(416, 416))
    save_detection=cv2.VideoWriter("detection video.avi",fourcc,30,(416, 416))
    # nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
    def frameNorm(frame, bbox):
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)
        
    def displayFrame(name, frame):
        color = (255, 0, 0)
        global lat_aver
        global long_aver
        global cur_uav_latitude
        global cur_uav_longitude
        #save_original.write(frame)
        center_x = (detections[0].xmin+detections[0].xmax)/2 if len(detections)>0 else 0
        for detection in detections:
            bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
            cv2.putText(frame, labelMap[detection.label], (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"{int(detection.confidence * 100)}%", (bbox[0] + 10, bbox[1] + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            
            #print("lat=", cur_uav_latitude , " long=",cur_uav_longitude)

            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
            center_x_tmp = (detection.xmin+detection.xmax)/2
            if abs(center_x-0.5)>abs(center_x_tmp-0.5):
                center_x = center_x_tmp
        if center_x < 0.55 and center_x > 0.45:
            if cur_uav_latitude!=0 and cur_uav_longitude!=0:   
                gps_lat_aver.append(cur_uav_latitude)
                gps_long_aver.append(cur_uav_longitude)

        if (center_x < 0.45 and center_x > 0) or (center_x > 0.55 and center_x < 1) :
            if len(gps_lat_aver)!=0 and len(gps_long_aver)!=0 :
                lat_aver=sum(gps_lat_aver)/len(gps_lat_aver)
                long_aver=sum(gps_long_aver)/len(gps_long_aver)
            if lat_aver!=0 and long_aver!=0:
                #print("lat=", lat_aver," _",len(gps_lat_aver), " long=",long_aver," _",len(gps_long_aver))
                with open("test_gps.csv", 'a', encoding = 'utf-8') as f:
                    f.write("{},{}\n".format(lat_aver,long_aver))
            gps_lat_aver.clear()
            gps_long_aver.clear()
            lat_aver=0
            long_aver=0


        # Show the frame
        cv2.imshow(name, frame)
        # save video
        #save_detection.write(frame)
    while True:
        if syncNN:
            inRgb = qRgb.get()
            inDet = qDet.get()
        else:
            inRgb = qRgb.tryGet()
            inDet = qDet.tryGet()

        if inRgb is not None:
            frame = inRgb.getCvFrame()
            cv2.putText(frame, "NN fps: {:.2f}".format(counter / (time.monotonic() - startTime)),
                        (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color2)

        if inDet is not None:
            detections = inDet.detections
            counter += 1

        if frame is not None:
            displayFrame("rgb", frame)

        if cv2.waitKey(1) == ord('q'):
            break
