import time
from dronekit import Vehicle, connect,  VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from pymavlink import mavwp
from piston_mini_client.failhandlers import SocketError
import math
import os


#to establish connection with sitl via udp
try:
    vehicle = connect('127.0.0.1:14550', wait_ready=True)
    mav1 = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    print "Established connection with SITL"
except SocketError:
    print"No server exist or check the baud rate"
 

mav1.wait_heartbeat()
print"  Got heartbeat"

while (not vehicle.is_armable):
    print"Pre-arm checks....."
    time.sleep(1)
    
print"Pre-arm checks passed "
    
flag=0;

#load waypoints from a txt file
wp=mavwp.MAVWPLoader()
try:
    wp.load("pqr.txt")
    print"file successfully loaded"
except IOError as e:
    print"Error code:",e
    print"waypoint file should be in same folder where python file is "
    flag=1;



def getDistanceFromLatLonInKm(lat1,lon1,lat2,lon2): 
    R = 6371; # Radius of the earth in km
    dLat = deg2rad(lat2-lat1);  # deg2rad below
    dLon = deg2rad(lon2-lon1); 
    a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(deg2rad(lat1)) * math.cos(deg2rad(lat2)) * math.sin(dLon/2) * math.sin(dLon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c # Distance in km
    return d


def deg2rad(deg):
    return deg * (math.pi/180)





if(flag==0):
    
    #print the no. of waypoints
    print"%d" % wp.count()

    #print waypoint info of 2
    #print"%s" % wp.wp(0).x
    
    # Send a clear all waypoints command to the flight controller
    mav1.waypoint_clear_all_send()  
    
    #Send a waypoint_count(N) to the flight controller
    mav1.waypoint_count_send(wp.count())  


    for i in range(wp.count()):
    
            #Listen for MISSION_REQUEST messages from flight controller
            msg = mav1.recv_match(type=['MISSION_REQUEST'],blocking=True)             
            
            #send the appropriate waypoint
            mav1.mav.send(wp.wp(msg.seq))                                                                      
            print 'Sending waypoint {0}'.format(msg.seq)


    #check for GPS_fix
    while vehicle.gps_0.fix_type < 2:
        print "Waiting for GPS...:", vehicle.gps_0.fix_type
        time.sleep(1)
    
    print "Found GPS:",vehicle.gps_0


    # arm
    #format mav1.mav.command_long_send(mav1.target_system, mav1.target_component,command_id, confirmation, arm, param1, param2, param3, param4, param5, param6)
    mav1.mav.command_long_send(mav1.target_system, mav1.target_component,
                          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,
                          0, 0, 0, 0, 0, 0)
    
    
    #if arming fails due to certian mode
    if(not vehicle.armed):
        #change mode to stabilize
        vehicle.mode = VehicleMode("STABILIZE")

        time.sleep(1)
        #now again try to arm
        mav1.mav.command_long_send(mav1.target_system, mav1.target_component,
                          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,
                          0, 0, 0, 0, 0, 0)
    
    
    print "Motors arm"
    


    
    #mode auto (auto mode sets when arm and we have to arm in other mode and then move to auto mode)
    mav1.mav.command_long_send(mav1.target_system, mav1.target_component,
    mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)

    #vehicle2 = connect('127.0.0.1:14550',_initialize=True,wait_ready=True,status_printer=False,heartbeat_timeout=60)
    vehicle2 = connect('127.0.0.1:14550',_initialize=True,wait_ready=True,status_printer=False,heartbeat_timeout=60)

    lat2=wp.wp(1).x
    lon2=wp.wp(1).y
    j=1
    #lat1=vehicle.location.global_relative_frame.lat
     
    while(1):
        
    #time.sleep(1)
        cmds = vehicle2.commands
        cmds.download()
    #time.sleep(1)
        cmds.wait_ready()
        lat1=vehicle2.location.global_relative_frame.lat
        lon1=vehicle2.location.global_relative_frame.lon
        #print lat1
        #use try because lat1 value can be none
        try:
            distance=getDistanceFromLatLonInKm(lat1,lon1,lat2,lon2)*1000
            print distance
        except Exception:
            print ""
        
        if(distance <= 4):
            print "Reached %d waypoint" % j       
            j=j+1
            if(j<wp.count()):
                lat2=wp.wp(j).x
                lon2=wp.wp(j).y
           
        if(j==(wp.count())):
            print"Reached Destination"
            break 


    
    
