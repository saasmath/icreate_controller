#! /usr/bin/env python
import roslib; roslib.load_manifest('icreate_controller')
import rospy
from icreate_controller.srv import *
from irobot_create_2_1.srv import *
from irobot_create_2_1.msg import *

"""
  icreate watcher node:
    watches the icreate driver's sensor packet and waits until emergency conditions
    are hit, then it sends a kill signal to the main icreate controller node and brakes
    the icreate
    current emergency condtions:
      -either wheel lifts off the ground
      -either of the front and side cliff sensors go off
      -icreate is moving without controller node online
"""

#stop icreate by calling brake service on icreate driver
def _iCreateDriverBrake():
  rospy.wait_for_service('brake')
  try:
    #call service with parameters
    srvc = rospy.ServiceProxy('brake', Brake)
    response = srvc(True)
    return response.success
  except rospy.ServiceException, e:
    print "Failed to call brake: %s" %e  

#stop icreate node program by calling icreate_shutdown service on node
def _iCreateNodeShutdown(reason):
  try:
    #call service with parameters
    srvc = rospy.ServiceProxy('icreate_shutdown', iCreateShutdown)
    response = srvc(reason)
    _iCreateDriverBrake()
  except rospy.ServiceException, e:
    pass 

#callback for sensor data from icreate
def _sensorCallback(data):
  #emergency situations
  if(data.wheeldropLeft or data.wheeldropRight):
    _iCreateNodeShutdown("iCreate wheels have dropped")
  elif(data.cliffLeft or data.cliffFronLeft or data.cliffFrontRight or data.cliffRight):
    _iCreateNodeShutdown("iCreate cliff sensors activated")

if __name__ == '__main__':
  #start up ros node and subscriber
  rospy.init_node('iCreateWatcher')
  sensorWatch = rospy.Subscriber("sensorPacket",SensorPacket,_sensorCallback)
  while not rospy.is_shutdown():
    #check if icreate node is  active, if not then brake
    try:
      rospy.wait_for_service('icreate_shutdown',2)
    except rospy.ROSException, e:
      _iCreateDriverBrake()
    
