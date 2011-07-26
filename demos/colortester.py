#! /usr/bin/env python
import roslib; roslib.load_manifest('icreate_controller')
import rospy
from image_tester import ImageTester
import cv
import time

def icall(img):
  thres = 20
  (cols,rows) = cv.GetSize(img)
  #colors = {'red':(255,0,0), 'green':(0,255,0), 'blue':(0,0,255)}
  colors = {'postit':(236,217,147),'pen':(28,131,116)}
  bins = {}
  for k in colors.iterkeys():
    bins[k] = []
    
  div = 2
  for x in range(cols/div):
    for y in range(rows/div):
      col = img[y*div,x*div]
      for k,v in colors.iteritems():
        diff = max(abs(v[0]-col[0]),abs(v[1]-col[1]),abs(v[2]-col[2]))
        if(diff < thres):
          bins[k].append((x*div,y*div))

  for k,v in bins.iteritems():
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
    if(not(ax==ay==0 and ix==cols and iy==rows)):
      cv.Rectangle(img, (ix,iy), (ax,ay), colors[k])
  cv.ShowImage("Color", img)
  cv.WaitKey(3)

if __name__== '__main__':
  video = ImageTester(icall)
  cv.NamedWindow("Color",1)
  rospy.spin()
  cv.DestroyAllWindows()
