import time
import pymavlink
from pymavlink import mavutil

#to establish connection with sitl
mav1 = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print "established connection"
mav1.wait_heartbeat()
print "got heartbeat"


print "address=%s" %(mav1.address)
#print "lat=%d lon=%d alt=%d heading=%s" % (mav1.lat,mav1.lng,mav1.alt,mav1.heading)
#print "message %s" %mav1.messages
print "params %s" %mav1.params
print "target_system %s" %mav1.target_system
print "target_component %s" %mav1.target_component
print "source_system %s" %mav1.source_system
#print "source_component %s" %mav1.source_component
print "flightmode %s" %mav1.flightmode

print "vehicle_type %s" %mav1.vehicle_type
print "mav_type %s" %mav1.mav_type
print "ground_pressure %s" %mav1.ground_pressure
print "ground_temperature %s" %mav1.ground_temperature




