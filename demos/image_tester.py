import roslib; roslib.load_manifest('icreate_controller')
import rospy
from std_msgs.msg import String
import sys
import time 
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageTester:
  def __init__(self,imageCallBack=None,imageTopic="image_raw"): 
    rospy.init_node('ImageTester')
    self._imagecall = imageCallBack
    self._imageInput = imageTopic
    self._cvBridge = CvBridge()
    self._imageSub = rospy.Subscriber(self._imageInput,Image,self._imagesubcall)
    self.currFrame = None
    
  def _imagesubcall(self,image):
    try:
      cvImage = self._cvBridge.imgmsg_to_cv(image, "bgr8")
      if(self._imagecall!=None):
        self._imagecall(cvImage)
    except CvBridgeError, e:
      print "Failed to convert image: %s" %e
