#! /usr/bin/env python
import roslib; roslib.load_manifest('icreate_controller')
import rospy
from icreate import iCreate
import curses

def setMovement(command,create,speed):
  if(command==ord('w')):
    create.move(speed)    
  elif(command==ord('a')):
    create.turn(speed)
  elif(command==ord('s')):
    create.move(-speed)
  elif(command==ord('d')):
    create.turn(-speed)

if __name__ == '__main__':
  screen = curses.initscr()
  speed = 70
  currMove = ''
  create = iCreate()
  while not rospy.is_shutdown():
    inp = screen.getch()
    #quit
    if(inp==ord('q')):
      exit()
    #icreate movement
    elif(inp==ord('w') or inp==ord('a') or inp==ord('s') or inp==ord('d')):
      setMovement(inp,create,speed)
      currMove = inp
    #icreate brake
    elif(inp==ord('f')):
      create.brake()
      currMove = inp
    #speed control
    elif(inp==ord('z')):
      speed = speed-10 if (speed-10)>=50 else speed
      setMovement(currMove,create,speed)
    elif(inp==ord('x')):
      speed = speed+10 if (speed-10)>=50 else speed
      setMovement(currMove,create,speed)
  
  curses.endwin()
