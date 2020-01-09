#!/usr/bin/env python

#working code for reading angles at 10Hz and distance at 3Hz 

import roslib; roslib.load_manifest('leica_ros_sph')
import rospy
import sys
import time
import math
import GeoCom_mod
#from leica_ros_sph.msg import AngleMeasStamped
#from leica_ros_sph.msg import RadialMeasStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from optparse import OptionParser
from operator import neg
#from math import sin,cos

if GeoCom_mod.COM_OpenConnection('/dev/ttyS0', 115200)[0]:
    sys.exit("Can not open Port... exiting")

prism_type = 7
GeoCom_mod.BAP_SetPrismType(prism_type)
GeoCom_mod.AUT_LockIn()
GeoCom_mod.TMC_SetEdmMode(9) #EDM_CONT_FAST = 9, // Fast repeated measurement (geocom manual p.91)
GeoCom_mod.TMC_DoMeasure()
time.sleep(2)
print "Leica TS is set up"

# Set up ROS:
rospy.init_node('leica_node')
pub_carth = rospy.Publisher("/leica/pose", PoseStamped, queue_size=1) #for px4
pub_point = rospy.Publisher('/leica/worldpose',PoseStamped, queue_size=1) #original total station world frame
print "ROS-node is set up"

loop_count = 1

while not rospy.is_shutdown():

    try:
        [error, RC, coord] = GeoCom_mod.TMC_GetCoordinate()
        carth_meas = PoseStamped()
        point_meas = PoseStamped()
        ###   DEBUG   ###
        #print 'error = ', error   
        #print 'RC = ', RC   
        print 'coord = ', coord   
        ###   DEBUG   ###     
        if RC==0 or RC==1284 or RC==1285:
            if coord[0]!=0 and coord[1]!=0 and coord[2]!=0:
                if RC==0:
                    print 'Successfully received coordinates \n'
                if RC==1284:
                    print '!!!!!WARNING!!!!! Accuracy could not be guaranteed \n'
                if RC==1285:
                    print '!!!!!WARNING!!!!! Distance could not be guaranteed \n'
                
		#should be ENU - XYZ
                if (loop_count<10):
                    x_offset =  float(coord[1])
                    y_offset =  float(coord[0])
                    z_offset =  float(coord[2])
                carth_x =   float(coord[1]) + (x_offset*-1) 
                carth_y =  float(coord[0]) + (y_offset*-1) 
                carth_h =  -float(coord[2]) - (z_offset*-1) 
      
                point_x =   float(coord[0])  #East
                point_y =   float(coord[1])  #North
                point_z =  float(coord[2])  #Up

            else:
                print '!!!!!WARNING!!!!! NO DATA!!! \n'
            
        carth_meas.header.stamp = rospy.Time.now()
        carth_meas.pose.position.x = carth_x
        carth_meas.pose.position.y = carth_y
        carth_meas.pose.position.z = carth_h
        carth_meas.header.seq = loop_count
        carth_meas.header.frame_id = 'fcu'
        pub_carth.publish(carth_meas)

        point_meas.header.stamp = rospy.Time.now()
        point_meas.pose.position.x = point_x
        point_meas.pose.position.y = point_y
        point_meas.pose.position.z = point_z
        point_meas.header.seq = loop_count
        point_meas.header.frame_id = 'fcu'
        pub_point.publish(point_meas)

        loop_count = loop_count + 1
        
    except ValueError:
        print "Non numeric value recieved!"

    except:
        print "No measurement or drop."
        # Short break in case the problem was related to the serial connection.
        time.sleep(0.3)
        # Then restart the measurement
        GeoCom_mod.TMC_DoMeasure()
        print "Restarted measurements"
    
# Closing serial connection, when execution is stopped
GeoCom_mod.COM_CloseConnection()

