#! /usr/bin/env python
import roslib; roslib.load_manifest('icreate_controller')
import rospy
from irobot_create_2_1.srv import *
from irobot_create_2_1.msg import *
from std_msgs.msg import String
from icreate_controller.srv import *
import sys
import time 
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#==============================================================
#iCreate Controller
#==============================================================
class iCreate:
  
  #================================  
  def __init__(self,sensorCallBack=None,imageCallBack=None,imageTopic="/camera/image_raw"): 
    """
      initialize a new icreate and start its ROS node along with its subsribers and services
        sensorCallBack-the callback function that is used when new sensor data is received
        imageCallBack-the callback function that is used when new image data is received
        imageTopic-the image topic subcribed to
    """
    #start ros node and services
    rospy.init_node('iCreateController')
    self._shutdownservice = rospy.Service('icreate_shutdown', iCreateShutdown, self._iCreateShutdown)
    self._shutdownFunc = None 
    
    #setup image subscriber
    self._imagecall = imageCallBack
    self._imageInput = imageTopic
    self._cvBridge = CvBridge()
    self._imageSub = rospy.Subscriber(self._imageInput,Image,self._imagesubcall)
    self._currFrame = None
    self._skipframes = False #frame skipping variables
    self._numFrameSkip = 3
    self._currFrameNum = 1
    
    #setup sensor subscriber
    self._initSensors = False
    self._sensorcall = sensorCallBack
    self._sensorSub = rospy.Subscriber("sensorPacket",SensorPacket,self._sensorsubcall)
    #initial empty sensor table
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
      pass #wait till create driver pushes sensor data
  
  #================================  
  def _iCreateShutdown(self,reason):
    """
      iCreate node global shutdown service that can be called as ros service or an internal function, exits program
        reason-either a string or ros service message that dictates the cause of shutdown
    """
    self.brake()
    if(type(reason)==type("")):
      rospy.loginfo(reason)
    else:
      rospy.loginfo(reason.reason)
    if(self._shutdownFunc != None):
      shutdownFunc()
    sys.exit()
  
  #================================      
  def setShutdownFunction(self,func):
    """
      Set the function that will be activated before the iCreate node is shutdown due to an issue
        func-input function that will be set as the ros shutdown function
    """
    self._shutdownFunc = func
  
  #================================  
  def _imagesubcall(self,image): 
    """
      image subscribler callback
    """    
    #skip 30fps down to 10 fps
    if(self._numFrameSkip==self._currFrameNum or not(self._skipframes)):
      self._currFrameNum = 1 #frame skips
      try:
        #turn image message into iplimage
        cvImage = self._cvBridge.imgmsg_to_cv(image, "bgr8")
        self._currFrame = cvImage
        #send image data to exterior callback if it exists
        if(self._imagecall!=None): 
          self._imagecall(self,cvImage)
      except CvBridgeError, e:
        print "Failed to convert image: %s" %e
    else:
      self._currFrameNum += 1
  
  #================================  
  def _sensorsubcall(self,data): 
    """
      sensor subscriber callback
    """
    #setup sensors table on first sensor packet
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
    else: #apply changes to sensor table if the old value has changed
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
  
  #================================
  def _changeAppend(self,key,val): 
    """
      helper method to check changes in sensor table data
    """
    if self._sensors[key] is not val:
      self._sensors[key]=val #set new sensor data in table
      self._onSensorEvent(key,val) #push new data to sensor event callback
   
  #================================ 
  def _onSensorEvent(self,sensorID,value): 
    """
      new sensor data callback
    """
    #send sensor data to exterior callback if it exists
    if(self._sensorcall!=None):
      self._sensorcall(self,sensorID,value)
  
  #================================  
  def sensor(self,sensorname): 
    """
      returns the sensor value for input sensorname, if it isn't a sensor returns None
        sensorname-the name of the sensor used
    """
    return self._sensors[sensorname] if sensorname in self._sensors.keys() else None
  
  #================================  
  def printSensors(self): 
    """
      print sensor and sensor data for all sensors
    """
    for key, value in self._sensors.iteritems():
      print key,":", value
  
  #================================  
  def getCurrentVideoFrame(self):
    """
      return the current frame of video as a cv mat
    """
    return self._currFrame
  
  #================================
  def toggleFrameSkipping(self,toggle):
    """
      If toggle true, then input video is assumed at 30 fps and frames are skipped to make it 10 fps
      If toggle false, then input video is used as is without skipping frames, Starts out false
        toggle-input boolean for skipping frames
    """
    self._skipFrames = toggle
  
  #================================  
  def brake(self): 
    """
      stop icreate by calling brake service on icreate driver
    """
    rospy.wait_for_service('brake')
    try:
      #call service with parameters
      srvc = rospy.ServiceProxy('brake', Brake)
      response = srvc(True)
      return response.success
    except rospy.ServiceException, e:
      print "Failed to call brake: %s" %e  
  
  #================================
  def turn(self,speed): 
    """
      turn icreate by calling turn service on icreate driver with input speed
        speed-icreate velocity, between -500mm/s(counter-clockwise) and 500mm/s(clockwise)
    """
    rospy.wait_for_service('turn')
    try:
      #call service with parameters
      #constrain speed
      speed = -500 if speed<-500 else (500 if speed>500 else speed) 
      srvc = rospy.ServiceProxy('turn', Turn)
      response = srvc(False,int(speed))
      return response.success
    except rospy.ServiceException, e:
      print "Failed to call turn: %s" %e
  
  #================================
  def move(self,speed): 
    """
      move icreate forward or backward by calling tank service on icreate driver using the same speed for both wheels
        speed-icreate velocity, between -500mm/s(backward) and 500mm/s(forward)
    """
    rospy.wait_for_service('tank')
    try:
      #call service with parameters
      #constrain speed
      speed = -500 if speed<-500 else (500 if speed>500 else speed)
      srvc = rospy.ServiceProxy('tank', Tank)
      response = srvc(False,int(speed),int(speed))
      return response.success
    except rospy.ServiceException, e:
      print "Failed to call move: %s" %e
  
  #================================
  def tank(self,left,right): 
    """
      move icreate by calling tank service on icreate driver using the left and right speeds for respective wheels
        left-icreate left wheel velocity, between -500mm/s(backward) and 500mm/s(forward)
        right-icreate right wheel velocity, between -500mm/s(backward) and 500mm/s(forward)
    """
    rospy.wait_for_service('tank')
    try:
      #call service with parameters
      #constrain speeds
      left = -500 if left<-500 else (500 if left>500 else left)
      right = -500 if right<-500 else (500 if right>500 else right)
      srvc = rospy.ServiceProxy('tank', Tank)
      response = srvc(False,int(left),int(right))
      return response.success
    except rospy.ServiceException, e:
      print "Failed to call tank: %s" %e

  #================================  
  def circle(self,speed,radius):
    """
      make the icreate move in a circle by turning in proportion to the input circle radius by calling circle service on icreate driver using input speed
        radius-radius of circle the icreate will turn around, in mm
        speed-icreate velocity, between -500mm/s(backward) and 500mm/s(forward)
    """
    rospy.wait_for_service('circle')
    try:
      #call service with parameters
      #constrain speed
      speed = -500 if speed<-500 else (500 if speed>500 else speed)
      srvc = rospy.ServiceProxy('circle', Circle)
      response = srvc(False,int(speed),int(radius))
      return response.success
    except rospy.ServiceException, e:
      print "Failed to call circle: %s" %e
  
  #================================
  def turnFor(self,duration,speed): 
    """
      turn the icreate at input speed for input duration then brake
        duration-turning time in seconds
        speed-icreate velocity, between -500mm/s(counter-clockwise) and 500mm/s(clockwise)
    """
    self.turn(speed)
    time.sleep(duration)
    self.brake()
  
  #================================
  def moveFor(self,duration,speed): 
    """
      move the icreate forward or backward at input speed for input duration then brake
        duration-turning time in seconds
        speed-icreate velocity, between -500mm/s(backward) and 500mm/s(forward)
    """
    self.move(speed)
    time.sleep(duration)
    self.brake()
    
  #================================
  def tankFor(self,duration,left,right): 
    """
      move the icreate with input speeds to each wheel for input duration then brake
        duration-turning time in seconds
        left-icreate left wheel velocity, between -500mm/s(backward) and 500mm/s(forward)
        right-icreate right wheel velocity, between -500mm/s(backward) and 500mm/s(forward)
    """
    self.tank(left,right)
    time.sleep(duration)
    self.brake()
  
  #================================
  def circleFor(self,duration,speed,radius): 
    """
      make the icreate move in a circle by turning in proportion to input circle radius for input duration then brake
        duration-turning time in seconds        
        radius-radius of circle the icreate will turn around, in mm
        speed-icreate velocity, between -500mm/s(backward) and 500mm/s(forward)
    """
    self.circle(speed,radius)
    time.sleep(duration)
    self.brake()
  
  #================================
  def turnUntil(self,condition,speed=130): 
    """
      turn the icreate until the condition function returns true, then brake
        condition-input function that is given the icreate as its parameter
        speed-icreate velocity, between -500mm/s(counter-clockwise) and 500mm/s(clockwise)
    """
    self.turn(speed)
    while (not condition(self)):
      pass
    self.brake()

  #================================  
  def moveUntil(self,condition,speed=130): 
    """
      move the icreate forward or backward until the condition function returns true, then brake
        condition-input function that is given the icreate as its parameter
        speed-icreate velocity, between -500mm/s(backward) and 500mm/s(forward)
    """
    self.move(speed)
    while (not condition(self)):
      pass
    self.brake()
  
  #================================
  def tankUntil(self,condition,left,right): 
    """
      move the icreate with input speeds to each wheel until the condition function returns true, then brake
        condition-input function that is given the icreate as its parameter
        left-icreate left wheel velocity, between -500mm/s(backward) and 500mm/s(forward)
        right-icreate right wheel velocity, between -500mm/s(backward) and 500mm/s(forward)
    """
    self.tank(left,right)
    while (not condition(self)):
      pass
    self.brake()
  
  #================================
  def circleUntil(self,condition,speed,radius): 
    """
      make the icreate move in a circle by turning in proportion to input circle radius until the condition function returns true, then brake
        condition-input function that is given the icreate as its parameter
        radius-radius of circle the icreate will turn around, in mm
        speed-icreate velocity, between -500mm/s(backward) and 500mm/s(forward)
    """
    self.circle(speed,radius)
    while (not condition(self)):
      pass
    self.brake()
  
  #================================  
  def turnAngle(self,angle,speed=130): 
    """
      turn the icreate to the given angle with input speed
        angle-relative angle in degrees that the icreate will rotate 
        speed-icreate absolute velocity, between 0mm/s and 500mm/s
    """ 
    #hardcoded formula for turn duration given angle, based off 100mm/s
    #duration = abs(angle*.0240*100.0/abs(speed))
    #self.turnFor(duration,(angle/abs(angle))*speed)
    curr_angle = self._sensors["angle"]
    angTurn = 1 if angle>curr_angle else -1
    self.turnUntil((lambda(c):abs(c.sensor["angle"] - curr_angle) >= abs(angle)),angTurn*speed)
  
  #================================
  def moveDistance(self,distance,speed=130): 
    """
      move the icreate a given distance forward or backward with a input speed
        distance-relative distance in meters
        speed-icreate absolute velocity, between 0mm/s and 500mm/s
    """
    duration = abs(distance*1000/speed)
    self.moveFor(duration,(distance/abs(distance))*speed)


#==============================================================
#Color Blobs
#==============================================================

#================================
def _bloblabel(point,div,pdict,label,blob):
  """
   color blob helper function that labels a point and its neighbors
  """
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

#================================
def _blobrect(blob):
  """
    color blob helper function to find the bounds of a blob
  """
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

#================================
def colorblobs(img,colors,showVideo=False,showDebug=False):
  """
    takes in a cv image and a dictionary of colors and picks out all 
    the color blobs matching the input colors in the image, returns a dictionary of
    {'color':[array of blob rectangle bounds in form ((x1,y1),(x2,y2),(center of blob x, y))}
      img-input cv mat image
      colors-input dictionary of colors in the from {'color':(r,g,b)}
      showVideo-boolean for displaying image on a cv window
      showDebug-boolean for showing color points and blobs on a cv window
  """
  thres = 30 #maximum channel error for color
  (cols,rows) = cv.GetSize(img) #image size
  #colors = {'postit':(151,126,22),'pen':(28,131,116), 'red':(225,15,15)}
  bins = {} #intial color bins
  for k in colors.iterkeys():
    bins[k] = []
  
  #calculate image division by proportion to 160 by 120 image
  div = max(1,int(38400/(cols*rows))) #use 1/div amount of pixels in image
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
        _bloblabel(p,div,v,currlabel,preblobs[currlabel])
        labels.append(currlabel)
        currlabel+=1
        preblobs.append([])
    blobs = [] #final blobs array
    blobthres = 9*3 #minmum number of points in an array
    for b in preblobs:
      if(len(b)>blobthres):
        blobs.append(_blobrect(b))
    colorblobs[k] = blobs    
  
  if(showVideo): #input determines if video is shown
    if(showDebug): #input determins if points and blobs are outlined
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
    #show image in cv window
    cv.NamedWindow("Color",1)
    cv.ShowImage("Color", img)
    cv.WaitKey(3)
  

