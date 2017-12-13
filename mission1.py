import time
import MAVProxy
from pymavlink import mavwp
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
#from MAVProxy import mavproxy_wp
from MAVProxy.modules import mavproxy_wp



#to establish connection with sitl
mav1 = mavutil.mavlink_connection('udp:127.0.0.1:14550')
mav1.wait_heartbeat()

#load waypoints from a txt file
wp=mavwp.MAVWPLoader()
wp.load("abc.txt")

#wp._read_waypoints_v110("abc.txt")
#print the no. of waypoints
print"%d" % wp.count()

#print waypoint info of 2
print"%s" % wp.wp(2)

mav1.waypoint_clear_all_send()  
mav1.waypoint_count_send(wp.count())  


for i in range(wp.count()):
            msg = mav1.recv_match(type=['MISSION_REQUEST'],blocking=True)             
            mav1.mav.send(wp.wp(msg.seq))                                                                      
            print 'Sending waypoint {0}'.format(msg.seq)

