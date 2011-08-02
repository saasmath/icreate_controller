#! /usr/bin/env python
import roslib; roslib.load_manifest('icreate_controller')
import rospy
from icreate import iCreate
import curses

def setMovement(command,create,speed):
  if(command==ord('w')):
    create.move(speed)    
  elif(command==ord('a')):
    create.turn(-speed)
  elif(command==ord('s')):
    create.move(-speed)
  elif(command==ord('d')):
    create.turn(speed)

if __name__ == '__main__':
  screen = curses.initscr()
  speed = 70
  currMove = ''
  screen.border(0)
  screen.addstr(3,3,"Use wasd to move create")
  screen.addstr(4,3,"Use f to brake")
  screen.addstr(5,3,"Use z and x to decrease and increases speeds")
  screen.addstr(6,3,"Use ctrl+c to quit")
  curses.noecho()
  screen.refresh()
  create = iCreate()
  while not rospy.is_shutdown():
    inp = screen.getch()
    #icreate movement
    if(inp==ord('w') or inp==ord('a') or inp==ord('s') or inp==ord('d')):
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
