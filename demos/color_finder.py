#! /usr/bin/env python
import roslib; roslib.load_manifest('icreate_controller')
import rospy
from icreate import *

"""
  icreate_controller color finder demo:
    A program/tool to show how to get and use images from
    the robot. It keeps track of the color in the top
    leftmost pixel comming from the camera and shows it
    as a circle in that corner. Then when you want to keep
    track of a certain color, hit the left bumper to print 
    the color. Hitting the right bumper inserts a spacer in
    the print. Its a great tool to find out what a certain
    color in your lighting enviroment looks like through 
    the robot so that you can use that value in another program.
"""

#callback function sent to get images as they come from the camera
def imageCallback(icreate,img):
  #create a circle in the corner of the image with the color
  #of the pixel at 0,0
  cv.Circle(img,(0,0),40,img[0,0],5)
  icreate.color = img[0,0] #save the pixel color
  #show the updated image in the window
  cv.ShowImage("Finder",img) 
  cv.WaitKey(3)
  
#callback function sent to get realtime sensor data
def sensorCallback(icreate,key,val):
  #print color in corner if left bumper is hit
  if(key == "bumpLeft" and val):
    print icreate.color
  #print a couple spaces if right bumper is hit
  elif(key == "bumpRight" and val):
    print "\n\n"
  
if __name__== '__main__':
  create = iCreate(sensorCallback,imageCallback)
  #set a local variable on the create instance
  create.color = (0,0,0) 
  cv.NamedWindow("Finder",1) #use a cv window to show image
  rospy.spin() #this continues the program until an interupt(ctrl+c) is hit
  cv.DestroyAllWindows() #make sure all cv windows are destroyed
  

