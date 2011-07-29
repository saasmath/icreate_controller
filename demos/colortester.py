#! /usr/bin/env python
import roslib; roslib.load_manifest('icreate_controller')
import rospy
from image_tester import ImageTester
import cv
import time
        
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

def icall(img):
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

if __name__== '__main__':
  video = ImageTester(icall)
  cv.NamedWindow("Color",1)
  rospy.spin()
  cv.DestroyAllWindows()
