#! /usr/bin/env python
import roslib; roslib.load_manifest('icreate_controller')
import rospy
from icreate import *

"""
  icreate_controller bump demo:
    A simple program to show how to control the robot's movements
    and use its sensor data. The robot continously moves forward
    until it bumps into something, then it backs up and turns the
    opposite direction of the bump.
"""

#callback function sent to get realtime sensor data
def sensorCallback(icreate,name,val):
  #check if the bump sensors was changed
  if(name == "bumpLeft" or name == "bumpRight"):
    if(val):
      print name,"was hit!"

if __name__== '__main__':
  create = iCreate(sensorCallback) #initialize the icreate with the callback
  while not rospy.is_shutdown(): #run until an interupt(ctrl+c) is heard
    #use moveUntil to move the robot until an input the input function returns true
    #in this case a lambda function is used to check if either of the bumpers gets hit
    create.moveUntil(
      lambda(icreate):
        ((icreate.sensor("bumpLeft") or icreate.sensor("bumpRight")) == True))
    side = 1 if create.sensor("bumpLeft")==True else -1 #get the side of bumper hit
    create.moveDistance(-.25) #move back .25m
    create.turnAngle(side*90) #turn 90 degrees against the side of hit
    

