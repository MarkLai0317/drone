#!/usr/bin/env python
import rospy
import time
import simple_tello
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty




def tello_pass(t1, rec_count=0):
  
  # control loop
  check = False

  

  while not rospy.is_shutdown():
  
    # wait
    while t1.state.target_x[rec_count] == -1 and t1.state.target_y[rec_count] == -1:
      pass
    
    print(t1.state.target_x[rec_count], t1.state.target_y[rec_count])
    
    dx = t1.state.target_x[rec_count] - 480
    dy = t1.state.target_y[rec_count] - 200
    
    if t1.state.canPass[rec_count] == 1:
      msg = Twist()
      msg.linear.y = 1
      #msg.linear.z = 0.1
      t1.controler.move(msg, 1.5)
      
      msg = Twist()
      msg.linear.y = 1
      t1.controler.move(msg, 3)
  
      msg = Twist()
      t1.controler.move(msg, 1)
      #rec_count += 1
      break
    
    elif t1.state.canPass[rec_count] == 0:
      
      msg = Twist()
      msg.linear.y = 0.5
      t1.controler.move(msg, 0.5)
      if check == False:
        if abs(dx) < 24 and abs(dy) < 24:
          check = True
          msg = Twist()
          msg.linear.y = 0.2
          t1.controler.move(msg, 0.5)
        else:
          msg = Twist()
          if dx != 0:
            msg.linear.x = dx / abs(dx) * 0.1
          if dy != 0:
            msg.linear.z = -dy / abs(dy) * 0.2
          
          t1.controler.move(msg, 0.5)
          
      else:
        if abs(dx) >= 60 or abs(dy) >= 30:
          check = False
          msg = Twist()
          if dx != 0:
            msg.linear.x = dx / abs(dx) * 0.1
          if dy != 0:
            msg.linear.z = -dy / abs(dy) * 0.2
          
          t1.controler.move(msg, 0.5) 
        else:
          msg = Twist()
          msg.linear.y = 0.2
          t1.controler.move(msg, 0.5)


def tello_turn(t1):
     while not rospy.is_shutdown():
        while len(t1.state.tag_center) == 0 and len(t1.state.tag_corners) == 0:
            pass
        msg = Twist()
        t1.controler.rotate(msg)

        
          
def main():
  t1 = simple_tello.Tello_drone()
  
  while t1.state.is_flying == False: 
    t1.controler.takeoff()
  
  while t1.state.fly_mode != 6:
    print("wait...")
  
  tello_pass(t1, 0)

  tello_turn(t1)

  tello_pass(t1, 1)
  
  while t1.state.fly_mode != 6:
    print("wait...") 
    
  while t1.state.is_flying == True:
    t1.controler.land() 

  
if __name__ == "__main__":
  rospy.init_node("pass_example", anonymous=True)
  main()
