#! /usr/bin/env python
import roslib; roslib.load_manifest('icreate_controller')
import rospy
from icreate import iCreate

def sensorCallback(icreate,key,val):
  if(key == "current"):
    print "Current: %s" %val
    
def bump(icreate):
  return ((icreate.sensor("bumpLeft") or icreate.sensor("bumpRight")) == True)

if __name__== '__main__':
  create = iCreate(sensorCallback)
  #create.move(100)
  while not rospy.is_shutdown():
    create.moveUntil(bump)
    create.turnAngle(90)


