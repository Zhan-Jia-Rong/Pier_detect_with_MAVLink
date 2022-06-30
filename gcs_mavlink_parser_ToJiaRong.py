#!/usr/bin/env python3

import os, sys, socket, time, select, subprocess, struct
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

cur_uav_latitude = 0
cur_uav_longitude = 0
cur_uav_altitude = 0
cur_uav_satellites = 0
cur_uav_gps_ground_speed = 0

noGCS_flag = 1

class nothing(object):
    def __init__(self):
        return
    def write(self, data):
        return len(data)
    def read(self):
        return []

def init_mav():
    mav_master = mavutil.mavlink_connection(device="tcp:127.0.0.1:5762", baud=115200, source_system=255)
    print ("Waiting for APM heartbeat")
    while True:
        hb = mav_master.recv_match(type='HEARTBEAT', blocking=True)
        if hb.type != mavutil.mavlink.MAV_TYPE_GCS:
            print ('Heartbeat from APM system %d' % mav_master.target_system)
            break
    mav_modes = mav_master.mode_mapping()
    return (mav_master, mav_modes)

def my_main():
    global cur_uav_latitude
    global cur_uav_longitude
    global cur_uav_altitude
    global cur_uav_satellites
    global cur_uav_gps_ground_speed
    global noGCS_flag

    compid = 1

    # Connect UAV
    mav_master, mav_modes = init_mav()
    inject_mav = mavlink.MAVLink(nothing(), mav_master.target_system, compid)
    print ('mav_master.target_system: %d' % mav_master.target_system)

    while True:
        cur_ts = time.time()

        # receive message from UAV
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

        if noGCS_flag == 1:
            noGCS_flag += 1
            #send request_data_stream to uav
            print ("send request_data_stream to UAV [flag : %d]" % (noGCS_flag))
            mav_master.mav.request_data_stream_send(mav_master.target_system, mav_master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10 ,1)
        #print("lat=", cur_uav_latitude , " long=",cur_uav_longitude)
        #print ('Lat: %.7f ' % gps_lat ,'Long: %.7f ' % gps_long)
        #with open("gps_get.csv",'a',encoding='utf-8') as f:
        #    f.write("Lat={},Long={}\n".format(cur_uav_latitude, cur_uav_longitude))
        continue

if __name__ == "__main__":
    my_main()

