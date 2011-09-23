#! /usr/bin/env python
import roslib; roslib.load_manifest('icreate_controller')
import rospy
from icreate import *

"""
  icreate_controller ar follower demo:
    A program that shows how to use the ar tools available
    in the icreate_controller library. The robot will follow
    the closest ar marker it finds and turn to follow it.
"""

#go through marker list and find closest marker
#measured by the marker diameter
def getClosestMarker(markers):
  cmarker = markers[0]
  for x in range(1,len(markers)):
    if(markers[x].diameter < cmarker.diameter):
      cmarker = markers[x]
  return cmarker

if __name__== '__main__':
  create = iCreate()
  col = 160 #width of the input image
  pastDir = 1 #intial turning direction to look for markers
  while not rospy.is_shutdown(): #while interupt is not hit
      #get the ar markers(in a list) currently found by the create
      armarkers = create.getARMarkers()
      if(armarkers!=None and len(armarkers)>0): #make sure there are ar markers found
        marker = getClosestMarker(armarkers) #get the closest marker
        #the offset of the marker from the center
        diff = marker.x-col/2.0 
        #stop the create if the marker is in the middle 3rd of the image
        if(abs(diff) < col/6.0):
          print "Found it!"
          pastDir=diff/(abs(diff)+1) #save direction from middle
          create.brake()
        #turn towards the marker if it is on either edge of the image
        else:
          print "Chasing it!"
          pastDir=diff/(abs(diff)+1) #save direction from middle
          #turn in proportion to the offset
          create.turn(250*(diff/(5/6.0*col)))
      #if there are no markers then turn in the direction a marker was last seen
      else:
        print "Looking for it!"
        create.turn(pastDir*70)
      
