#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import Vector3
import numpy
from tf.transformations import euler_from_quaternion
from sherpa_msgs.msg import Cmd

### Definig global variables kE and kN
kX = 0
kY = 0
Move = True
exG = 0   #Global Error in x axes
eyG = 0   #Global Error in y axes
error_present = False

def cnst(eX,eY,Vmax,b,R):
    global kX,kY
    if eY >= 0 and eY < 0.0001:
	eY = 0.0001
    elif  eY < 0 and eY > -0.0001:
        eY = -0.0001
    ## Calculating the constraints
    if eX >= 0 and eY >= 0:
       kY = Vmax / (eX/eY+2*b/R)
    if eX >= 0 and eY <= 0:
       kY = Vmax/(eX/eY-2*b/R)
    if eX <= 0 and eY <= 0:
       kY = -Vmax/(eX/eY+2*b/R)
    if eX <= 0 and eY >= 0:
       kY = -Vmax/(eX/eY-2*b/R)
    kX = eX*kY/eY
    ## Absulute values
    kX = math.fabs(kX)
    kY = math.fabs(kY)
    return 0;


def cmd_callback(data):
   global Move
   Move = True
   #if math.fabs(data.param1 - 1) < 0.01:
   #    Move = True
   #else:
   #    Move = False
       

def body_error_callback(data):
   global exG,eyG,error_present
   exG = data.x
   eyG = data.y
   error_present = True

def flp():
   # Variables
   global kX,kY,Move,exG,eyG,error_present
   Vmax = 5
   b = 0.4
   R = 0.8
   eX_Offset = 10
   Tracking_precision = 0.5
   
   # ROS params
   rospy.init_node('flp', anonymous=True)
   rate = rospy.Rate(20)
   
   # Subscribers
   rospy.Subscriber("Cmd", Cmd, cmd_callback)
   rospy.Subscriber("body_error", Vector3, body_error_callback)
   
   #Publishers
   speed_pub = rospy.Publisher('speedfollow', Vector3, queue_size=5)
   
   
   # Pause before executing the command
   #rospy.sleep(10.)
   while not rospy.is_shutdown():
      # Move only if it is requested
      if error_present: 
            eX = exG - eX_Offset
            eY = eyG
      else:
            #print("ciao")
            eX = 0
            eY = 0
      if math.fabs(eX) < Tracking_precision:
          eX=0
      if math.fabs(eY) < Tracking_precision:
          eY=0
      # Calculating the saturation limits
      cnst(eX,eY,Vmax,b,R);
      # Control Action
      Ux = math.atan(10*eX)*kX*2/3.14159265359
      Uy = math.atan(10*eY)*kY*2/3.14159265359
      U = numpy.matrix(( (Ux),(Uy) )).transpose()
      # Sending the Control Action 
      speed = Vector3()
      VRL = numpy.matrix (( (1,-1),(1,1) ))*U
      speed.x = VRL.item(0)  # left hand
      speed.y = VRL.item(1)
      speed.z = 0 
      speed_pub.publish(speed)
      rate.sleep()

if __name__ == '__main__':
    try:
        flp()
    except rospy.ROSInterruptException:
        pass
