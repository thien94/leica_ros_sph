#!/usr/bin/env python

import roslib; roslib.load_manifest('leica_ros_sph')
import rospy
import sys
import time
import math
import GeoCom_mod

from geometry_msgs.msg import PointStamped
from optparse import OptionParser
from operator import neg

# Handling options
usage = "usage: rosrun leica_interface %prog [options]"
parser = OptionParser(usage=usage)
parser.set_defaults(port="/dev/ttyUSB0",baudrate=115200, debug=False)
parser.add_option("-p", "--port", action="store", type="string", dest="port", help="specify used port [default: %default]")
parser.add_option("-b", "--baudrate", action="store", type="int", dest="baudrate", help="specify used baudrate [default: %default]")
parser.add_option("-d", "--debug", action="store_true", dest="debug", help="print debug information")
(options, args) = parser.parse_args()

if GeoCom_mod.COM_OpenConnection(options.port, options.baudrate )[0]:
    sys.exit("Can not open Port... exiting")

prism_type = 7
GeoCom_mod.BAP_SetPrismType(prism_type)
GeoCom_mod.AUT_LockIn()
GeoCom_mod.TMC_SetEdmMode(9) #EDM_CONT_FAST = 9, // Fast repeated measurement (geocom manual p.91)
GeoCom_mod.TMC_DoMeasure()
time.sleep(2)
print "Leica is set up"

# Set up ROS:
rospy.init_node('leica_node')
point_pub = rospy.Publisher('/leica/worldposition',PointStamped, queue_size=1) #prism location in original total station world frame
point_msg = PointStamped()
print "ROS-node is set up"

loop_count = 1

while not rospy.is_shutdown():

    try:
        [error, RC, coord] = GeoCom_mod.TMC_GetCoordinate()
        
        if options.debug: rospy.loginfo( 'Error: '+ str(error) )   
        if options.debug: rospy.loginfo( 'Return Code: '+ str(RC) )  
        if options.debug: rospy.loginfo( 'Received: '+ str(coord) )   
     
        if RC==0:
            rospy.loginfo ('Valid data:'+str(coord))
            #should be ENU - XYZ
            point_x =   float(coord[0])  #East
            point_y =   float(coord[1])  #North
            point_z =  float(coord[2])   #Up
            
        elif RC==1284:
            rospy.logwarn( 'Accuracy could not be guaranteed \n' )
            print ('Still sending data:'+str(coord))
            point_x =   float(coord[0])  #East
            point_y =   float(coord[1])  #North
            point_z =  float(coord[2])   #Up
                        
        elif RC==1285:
            rospy.logwarn('No valid distance measurement! \n')
            
        else:
            rospy.logwarn( '\n'+'ERROR, Return code: '+str(RC)+'\n')
        
        point_msg.header.seq = loop_count  
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = 'world'
        point_msg.point.x = point_x
        point_msg.point.y = point_y
        point_msg.point.z = point_z
        point_pub.publish(point_msg)

        loop_count = loop_count + 1
        
    except ValueError:
        rospy.logwarn( "Non numeric value recieved!" )

    except:
        rospy.logfatal( "No measurement or drop." )
        # Short break in case the problem was related to the serial connection.
        time.sleep(0.2)
        # Then restart the measurement
        GeoCom_mod.TMC_DoMeasure()
        rospy.loginfo( "Restarted measurements" )
    
# Closing serial connection, when execution is stopped
GeoCom_mod.COM_CloseConnection()

