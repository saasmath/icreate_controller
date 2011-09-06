#! /usr/bin/env python
import roslib; roslib.load_manifest('icreate_controller')
import rospy
from icreate import *

"""
  icreate_controller color follower demo:
    A program that shows how to use the color blobs part 
    of the module. The robot tries to find the biggest blob 
    of a single color and follows it around. The color itself
    is independent to the lighting and may need to be changed
    in another enviroment.
"""

#helper function to get the biggest blobs of each color
#from a set of blobs given from the color blobs function
def getBiggestBlobs(blobs):
  #go through all color and blob array pairs
  for k,v in blobs.iteritems(): 
    if len(v)>0: #make sure that there are blobs for the color
      biggest=v[0] #set the biggest blob as the first
      #iterate through the rest of the blobs
      for x in range(1,len(v)):
        #check if the any bigger blobs in terms of area
        if(biggest[3]<v[x][3]):
          biggest=v[x]
      blobs[k] = biggest #set the biggest
  return blobs

if __name__== '__main__':
  create = iCreate()
  #the color is the color of a pink post-it
  #it is important to change this value as per to the enviroment of the robot
  #this could be best done using the color_finder demo.
  colors = {"post-it_pink":(168,135,211)}
  col = 160 #width of the input image
  pastDir = 1 #intial turning direction to look for blobs
  while not rospy.is_shutdown(): #while interupt is not hit
    if(create.getCurrentVideoFrame()!=None): #wait till the create gets an image
      #get the color blobs with the pink color, the threshold is set to 30 
      #and the video will be shown. It is important to adjust the threshold
      #if there are too many large blobs or too many small ones.
      blobs = colorblobs(create.getCurrentVideoFrame(),colors,30,True,False)
      blobs = getBiggestBlobs(blobs) #find the biggest blobs
      if(len(blobs)>0): #make sure there are found blobs
        #the offset of the blob from the center
        diff = blobs["post-it_pink"][2][0]-col/2.0 
        #stop the create if the blob is in the middle 3rd of the image
        if(abs(diff) < col/6.0):
          print "Found it!"
          pastDir=diff/abs(diff) #save direction from middle
          create.brake()
        #turn towards the blob if it is on either edge of the image
        else:
          print "Chasing it!"
          pastDir=diff/abs(diff) #save direction from middle
          #turn in proportion to the offset
          create.turn(70*(diff/(5/6.0*col)))
      #if there are no blobs then turn in the direction a blob was last seen
      else:
        print "Looking for it!"
        create.turn(pastDir*70)
      

