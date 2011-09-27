#! /usr/bin/env python
import roslib; roslib.load_manifest('icreate_controller')
import rospy
from icreate import *

"""
  icreate_controller color finder2 demo:
    A program/tool to show how to get and use images from
    the netbook camera just using ROS itself. The program brings up two
    windows, when the user clicks on the first window that is
    showing the video stream, the program picks out that frame
    and processes it and puts it in the second window. Then when
    the user clicks on the second window it prints out the color
    value of the pixel they are clicking on.
    
    A user editing this demo should change the processImage function
    to do something better than just return the color. Image segmentation
    and flooding can be done here and then outputed to the second window.
    Extra documentation on image processing with open cv can be found at:
    http://opencv.willowgarage.com/documentation/python/index.html
    http://opencv.willowgarage.com/documentation/python/cookbook.html
    http://opencv.willowgarage.com/documentation/python/highgui_user_interface.html
    and various other resources online.
"""
#color_finder class to process images from camera
class color_finder:
  def __init__(self, imageTopic): #input image topic to subscribe to
    #setup ros connections
    rospy.init_node('Color_Finder') #setup program as a ros node
    #subscribe to image topic with a callback that accepts Image
    self.imageSub = rospy.Subscriber(imageTopic,Image,self.imageCallback)     
    self.cvBridge = CvBridge() #image message to cv image bridge
    #image members
    self.colorImage = None
    self.grayImage = None
    self.segImage = None
    
    cv.NamedWindow("Color Finder1",1) #use a cv window to show image
    cv.NamedWindow("Color Finder2",1) #use a cv window to show segmented image frame
    #mouse callbacks for both windows
    cv.SetMouseCallback("Color Finder1", self.window1Callback, None) 
    cv.SetMouseCallback("Color Finder2", self.window2Callback, None) 
    
  #callback function sent to get image messages as they come from the camera node
  def imageCallback(self,image):
    #convert message to cv IPL image in 8-channel bgr scale and 1 channel gray scale
    self.colorImage = self.cvBridge.imgmsg_to_cv(image, "bgr8") #CV_8UC3
    self.grayImage = self.cvBridge.imgmsg_to_cv(image, "mono8") #CV_8UC1
    #show the image in the window
    cv.ShowImage("Color Finder1",self.colorImage) 
    cv.WaitKey(3) 
    
  #function to process the color and gray images from camera
  #and return a new processed image, could be used to implement
  #segmentation in the future
  def processImage(self,color,gray): #input color image, gray image
    return color #currently just returns color back
    #edge = cv.CreateMat(gray.rows, gray.cols, gray.type)
    #cv.Canny(gray,edge,.4,.6)
  
  #first window callback, whenever the user clicks the mouse
  #it creates the specialized image for the frame
  #and projects it on the second window
  def window1Callback(self,event, x, y, flags, param):
    if(event == cv.CV_EVENT_LBUTTONDOWN): #left clicked
      #get processed image from colorImage and grayImage
      self.segImage = self.processImage(self.colorImage,self.grayImage)
      #display processed image
      cv.ShowImage("Color Finder2",self.segImage) 
      cv.WaitKey(3) 
  
  #second window call back prints out color at clicked area
  def window2Callback(self,event, x, y, flags, param):
    if(event == cv.CV_EVENT_LBUTTONDOWN and self.segImage!=None): #left clicked
      print self.segImage[y,x]  
  
if __name__== '__main__':
  cf = color_finder("/camera/image_raw") #call to run main program/node
  rospy.spin() #this continues the program until an interupt(ctrl+c) is hit
  cv.DestroyAllWindows() #make sure all cv windows are destroyed
  

