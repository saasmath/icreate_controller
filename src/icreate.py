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

#==============================================================
#iCreate Controller
#==============================================================
class iCreate:
  def __init__(self,sensorCallBack=None,imageCallBack=None,imageTopic="image_raw",imageCallBack=None,imageTopic="/camera/image_raw"): 
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
      self.currFrame = cvImage
      if(self._imagecall!=None):
        self._imagecall(self,cvImage)
    except CvBridgeError, e:
      print "Failed to convert image: %s" %e

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

#==============================================================
#Color Blobs
#==============================================================
def bloblabel(point,div,pdict,label,blob):
  pstack = [point] #main stack for blob points
  while (len(pstack)!=0):
    #add point to main blob and give it a label
    currpoint = pstack.pop()
    pdict[currpoint] = label
    blob.append(currpoint)
    #add valid points around current point to stack
    for x in range(-1,2):
      for y in range(-1,2):
        if((x==0 and y==0)
          or not(pdict.has_key((currpoint[0]+x*div,currpoint[1]+y*div)))
          or label == pdict[(currpoint[0]+x*div,currpoint[1]+y*div)]):
          continue
        else:
          pstack.append((currpoint[0]+x*div,currpoint[1]+y*div))

def blobrect(blob):
  #rectangle properties
  ix=ax=blob[0][0]
  iy=ay=blob[0][1]
  cx=cy=0.0
  #get rectangle bounds
  for p in blob:
    cx += p[0]/len(blob)
    cy += p[1]/len(blob)
    if(p[0]>ax):
      ax = p[0]
    elif(p[0]<ix):
      ix = p[0]
    if(p[1]>ay):
      ay = p[1]
    elif(p[1]<iy):
      iy = p[1]
  return ((ix,iy),(ax,ay),(cx,cy))

def colorblobs(img):
  thres = 30 #maximum channel error for color
  (cols,rows) = cv.GetSize(img) #image size
  colors = {'postit':(151,126,22),'pen':(28,131,116), 'red':(225,15,15)}
  bins = {} #intial color bins
  for k in colors.iterkeys():
    bins[k] = []
    
  div = 2 #use 1/div amount of pixels in image
  for x in range(cols/div):
    for y in range(rows/div):
      col = img[y*div,x*div] #current color
      #put color into color bins if they match under threshold
      for k,v in colors.iteritems():
        diff = max(abs(v[0]-col[2]),abs(v[1]-col[1]),abs(v[2]-col[0]))
        if(diff < thres):
          bins[k].append((x*div,y*div))
  
  filled = {} #color bins with intial labels and extended points
  fill = 2 #range around pixel to add to bin
  for k,v in bins.iteritems():
    filled[k] = {} 
    for p in v:
      #add intial point and points around range
      #into color bin with false label
      for x in range(-fill,fill+1):
        for y in range(-fill,fill+1):
          filled[k][(p[0]+x*div,p[1]+y*div)] = -1
  
  colorblobs = {} #final color blobs bin
  for k,v in filled.iteritems():
    if len(v)==0: #skip empty color bins
      continue
    #intial labels
    currlabel = 0
    labels = [0]
    preblobs = [[]] #inital blobs array
    for p,l in v.iteritems():
      if(not(l in labels)): #if not labeled
        #label point and all close points into blob and increment label
        bloblabel(p,div,v,currlabel,preblobs[currlabel])
        labels.append(currlabel)
        currlabel+=1
        preblobs.append([])
    blobs = [] #final blobs array
    blobthres = 9*3 #minmum number of points in an array
    for b in preblobs:
      if(len(b)>blobthres):
        blobs.append(blobrect(b))
    colorblobs[k] = blobs    
  
  #show points on screen
  for k,v in filled.iteritems():
    ax=ay=0
    ix=cols
    iy=rows
    for p in v:
      if(p[0]>ax):
        ax = p[0]
      elif(p[0]<ix):
        ix = p[0]
      if(p[1]>ay):
        ay = p[1]
      elif(p[1]<iy):
        iy = p[1]
      cv.Circle(img, (p[0],p[1]), 4, (colors[k][2]/2,colors[k][1]/2,colors[k][0]/2))      
  
  #show blobs on screen
  for k,v in colorblobs.iteritems():
    for b in v:
      cv.Rectangle(img, b[0], b[1], (colors[k][2],colors[k][1],colors[k][0]))
    
  cv.ShowImage("Color", img)
  cv.WaitKey(3)
