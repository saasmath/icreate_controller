#! /usr/bin/env python
import roslib; roslib.load_manifest('icreate_controller')
import rospy
from irobot_create_2_1.srv import *
from irobot_create_2_1.msg import *
from std_msgs.msg import String
import sys
import time 
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class iCreate:

  def __init__(self,sensorCallBack=None,imageCallBack=None,imageTopic="image_raw"): 
    rospy.init_node('iCreateController')
    
    self._imagecall = imageCallBack
    self._imageInput = imageTopic
    self._cvBridge = CvBridge()
    self._imageSub = rospy.Subscriber(self._imageInput,Image,self._imagesubcall)
    self.currFrame = None
    
    self._initSensors = False
    self._sensorcall = sensorCallBack
    self._sensorSub = rospy.Subscriber("sensorPacket",SensorPacket,self._sensorsubcall)
    self._sensors = {
      "wheeldropCaster":[],                "wheeldropLeft":[],
      "wheeldropRight":[],                 "bumpLeft":[],
      "bumpRight":[],                      "wall":[],
      "cliffLeft":[],                      "cliffFronLeft":[],
      "cliffFrontRight":[],                "cliffRight":[],
      "virtualWall":[],                    "infraredByte":[],
      "advance":[],                        "play":[],
      "distance":[],                       "angle":[],
      "chargingState":[],                  "voltage":[],
      "current":[],                        "batteryTemperature":[],
      "batteryCharge":[],                  "batteryCapacity":[],
      "wallSignal":[],                     "cliffLeftSignal":[],
      "cliffFrontLeftSignal":[],           "cliffFrontRightSignal":[],
      "cliffRightSignal":[],               "homeBase":[],
      "internalCharger":[],                "songNumber":[],
      "songPlaying":[] }
    while(not self._initSensors):
      pass
  
  def _imagesubcall(self,image):
    try:
      cvImage = self._cvBridge.imgmsg_to_cv(image, "bgr8")
      if(self._imagecall!=None):
        self._imagecall(self,cvImage)
    except CvBridgeError, e:
      print "Failed to convert image: %s" %e
      
  def colorBlobs(self,image):
    print "asd"
    return

  def _sensorsubcall(self,data):
    if(not self._initSensors):
      self._sensors = {
        "wheeldropCaster":data.wheeldropCaster,"wheeldropLeft":data.wheeldropLeft,
        "wheeldropRight":data.wheeldropRight,  "bumpLeft":data.bumpLeft,
        "bumpRight":data.bumpRight,            "wall":data.wall,
        "cliffLeft":data.cliffLeft,            "cliffFronLeft":data.cliffFronLeft,
        "cliffFrontRight":data.cliffFrontRight,"cliffRight":data.cliffRight,
        "virtualWall":data.virtualWall,        "infraredByte":data.infraredByte,
        "advance":data.advance,                "play":data.play,
        "distance":data.distance,              "angle":data.angle,
        "chargingState":data.chargingState,    "voltage":data.voltage,
        "current":data.current,                "batteryTemperature":data.batteryTemperature,
        "batteryCharge":data.batteryCharge,    "batteryCapacity":data.batteryCapacity,
        "wallSignal":data.wallSignal,          "cliffLeftSignal":data.cliffLeftSignal,
        "cliffFrontLeftSignal":data.cliffFrontLeftSignal,"cliffFrontRightSignal":data.cliffFrontRightSignal,
        "cliffRightSignal":data.cliffRightSignal, "homeBase":data.homeBase,
        "internalCharger":data.internalCharger,"songNumber":data.songNumber,
        "songPlaying":data.songPlaying }
      self._initSensors = True
    else:
      self._changeAppend("wheeldropCaster", data.wheeldropCaster )
      self._changeAppend("wall", data.wall )
      self._changeAppend("wheeldropRight", data.wheeldropRight )
      self._changeAppend("bumpRight", data.bumpRight )
      self._changeAppend("voltage", data.voltage )
      self._changeAppend("homeBase", data.homeBase )
      self._changeAppend("cliffFronLeft", data.cliffFronLeft )
      self._changeAppend("wallSignal", data.wallSignal )
      self._changeAppend("internalCharger", data.internalCharger )
      self._changeAppend("angle", data.angle )
      self._changeAppend("wheeldropLeft", data.wheeldropLeft )
      self._changeAppend("batteryTemperature", data.batteryTemperature )
      self._changeAppend("bumpLeft", data.bumpLeft )
      self._changeAppend("current", data.current )
      self._changeAppend("cliffRight", data.cliffRight )
      self._changeAppend("songPlaying", data.songPlaying )
      self._changeAppend("chargingState", data.chargingState )
      self._changeAppend("play", data.play )
      self._changeAppend("cliffLeftSignal", data.cliffLeftSignal )
      self._changeAppend("songNumber", data.songNumber )
      self._changeAppend("distance", data.distance )
      self._changeAppend("cliffLeft", data.cliffLeft )
      self._changeAppend("cliffFrontRight", data.cliffFrontRight )
      self._changeAppend("batteryCharge", data.batteryCharge )
      self._changeAppend("advance", data.advance )
      self._changeAppend("infraredByte", data.infraredByte )
      self._changeAppend("virtualWall", data.virtualWall )
      self._changeAppend("cliffRightSignal", data.cliffRightSignal )
      self._changeAppend("batteryCapacity", data.batteryCapacity )
      self._changeAppend("cliffFrontLeftSignal", data.cliffFrontLeftSignal )
      self._changeAppend("cliffFrontRightSignal", data.cliffFrontRightSignal )
  
  def _changeAppend(self,key,val):
    if self._sensors[key] is not val:
      self._sensors[key]=val
      self._onSensorEvent(key,val)
  
  def _onSensorEvent(self,sensorID,value):
    if(self._sensorcall!=None):
      self._sensorcall(self,sensorID,value)
  
  def sensor(self,key):
      return self._sensors[key] if key in self._sensors else None
      
  def printSensors(self):
    for key, value in self._sensors.iteritems():
      print key,":", value
      
  def brake(self):
    rospy.wait_for_service('brake')
    try:
      srvc = rospy.ServiceProxy('brake', Brake)
      response = srvc(True)
      return response.success
    except rospy.ServiceException, e:
      print "Failed to call brake: %s" %e  
    
  def turn(self,speed):
    rospy.wait_for_service('turn')
    try:
      speed = -500 if speed<-500 else (500 if speed>500 else speed)
      srvc = rospy.ServiceProxy('turn', Turn)
      response = srvc(False,speed)
      return response.success
    except rospy.ServiceException, e:
      print "Failed to call turn: %s" %e
      
  def move(self,speed):
    rospy.wait_for_service('tank')
    try:
      speed = -500 if speed<-500 else (500 if speed>500 else speed)
      srvc = rospy.ServiceProxy('tank', Tank)
      response = srvc(False,speed,speed)
      return response.success
    except rospy.ServiceException, e:
      print "Failed to call move: %s" %e
      
  def turnFor(self,tim,speed):
    self.turn(speed)
    time.sleep(tim)
    self.brake()
  
  def moveFor(self,tim,speed):
    self.move(speed)
    time.sleep(tim)
    self.brake()
    
  def turnAngle(self,angle,speed=130):
    tim = angle*.0240*100.0/speed
    self.turnFor(tim,speed)

  def moveDistance(self,dis,speed=130):
    tim = dis*1000/speed
    self.moveFor(tim,speed)

  def turnUntil(self,cond,speed=130):
    self.turn(speed)
    while (not cond(self)):
      pass
    self.brake()
    
  def moveUntil(self,cond,speed=130):
    self.move(speed)
    while (not cond(self)):
      pass
    self.brake()
