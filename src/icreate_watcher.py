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
  rospy.wait_for_service('icreate_shutdown')
  try:
    #call service with parameters
    srvc = rospy.ServiceProxy('icreate_shutdown', iCreateShutdown)
    response = srvc(reason)
    rospy.sleep(2)
  except rospy.ServiceException, e:
    print "Failed to call icreate_shutdown: %s" %e  

#callback for sensor data from icreate
def _sensorCallback(data):
  if(data.wheeldropLeft == True or data.wheeldropRight == True):
    _iCreateNodeShutdown("iCreate wheels have dropped")
    _iCreateDriverBrake()

if __name__ == '__main__':
  #start up ros node and subscriber
  rospy.init_node('iCreateWatcher')
  sensorWatch = rospy.Subscriber("sensorPacket",SensorPacket,_sensorCallback)
  while not rospy.is_shutdown():
    #check if icreate node is  active, if not then brake
    try:
      rospy.wait_for_service('icreate_shutdown',5)
    except rospy.ROSException, e:
      _iCreateDriverBrake()
    
