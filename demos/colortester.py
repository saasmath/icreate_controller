#! /usr/bin/env python
import roslib; roslib.load_manifest('create_interface')
import rospy
from image_tester import ImageTester
import cv
import time

def icall(img):
  (cols,rows) = cv.GetSize(img)
  if cols > 60 and rows > 60 :
    cv.Circle(img, (50,50), 10, 255)
  cv.ShowImage("Color", img)
  cv.WaitKey(3)


if __name__== '__main__':
  video = ImageTester(icall)
  cv.NamedWindow("Color",1)
  rospy.spin()
  cv.DestroyAllWindows()
