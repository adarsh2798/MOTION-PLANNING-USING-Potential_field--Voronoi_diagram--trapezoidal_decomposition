#!/usr/bin/env python3

#  HERE APF IS  IMPLEMEMTED. FOR APFs THE CLOSEST DISTANCE TO THE 4 OBSTACLES IN THE GIVEN ENVIRONMENT IS NEEDED. HERE THAT DISTANCE IS CALCULATED USING PURELY LASER INFO AT EACH INSTANT
#  USING THE APPROACH MENTIONED IN HOWIE CHOSET BOOK WHERE IF WE FIND A LASER READING THAT SMALLELR THAN ITS ADJACENT READINGS IN BOTH DIRECTION IT IS THE CLOEST OBSTACLE DIST.



import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import math
import numpy as np
import message_filters
start=np.array([[0],[-5]])
goal=np.array([[0],[4]])
flag_laser=0
arg=[]
grad_list=[]
def U_rep(d_i,closest_ob):
     Q_star=np.array([1,2,1,2])
     eta=1
     U_rep=0
     for i  in range(d_i.shape[0]):
         if d_i[i]<=Q_star[closest_ob[i]]:
               U_rep_i=0.5*eta*((1/d_i[i])-(1/Q_star[i]))^2
         else:
               U_rep_i=0
          
         U_rep+=U_rep_i
     return U_rep
def U_att(loc):
     epsilon=0.5
     U_att=0.5*epsilon*np.linalg.norm(loc-goal)^2
     return U_att
 
def U_rep_grad(d_i,closest_ob,point_at_end,loc):
          Q_star=np.array([1,2,1,2])
          eta=1
          U_rep_grad=np.zeros((2,1))
          for i  in range(d_i.shape[0]):
               
               if d_i[i]<=Q_star[closest_ob[i]]:
                   poe=point_at_end[i]
                   U_rep_grad_i=eta*((1/d_i[i])-(1/Q_star[i]))*(-1/(d_i[i]**2))*(loc-poe)
               else:
                   U_rep_grad_i=0
          
               U_rep_grad+=U_rep_grad_i
          return U_rep_grad

def U_att_grad(loc):
     epsilon=0.5
     U_att_grad=epsilon*(loc-goal)
     return U_att_grad
def algo(ranges,loc,yaw):
   
   global arg,grad_list
   
   s_m=1 # to dela with gradient with abrupt change in direction, we can smooth graient by taking average of previous "s_m" gradients including current one which ensures bot dosent deviate 
         # too much from current heading
   
   ob=[[(0,2),0.5],[(1.5,0),0.7],[(0,-2),0.5],[(-1.5,0),0.5]]
   ob_center=[np.array(obb[0]) for obb in ob]

   angles=[float(yaw)+i*math.pi/180 for i in range(len(ranges))]
   ranges=[ranges[i] for i in range(0,len(ranges),5)]
   angles=[angles[i] for i in range(0,len(angles),5)]
   ALL_point_at_end=[(loc+np.array([[ranges[i]*math.cos(angles[i])],[ranges[i]*math.sin(angles[i])]])) for i in range(len(ranges))]
   valley_index=[]
   for i in range(1,len(ranges)-1):
          if(ranges[i-1]>ranges[i] and ranges[i]<ranges[i+1] and (np.min(np.linalg.norm(ALL_point_at_end[i].T-ob_center,axis=1))<0.85)):
                      valley_index.append(i)
   if(ranges[0]<ranges[len(ranges)-1] and ranges[0]<ranges[1] and  (np.min(np.linalg.norm(ALL_point_at_end[0].T-ob_center,axis=1))<0.85)):
             valley_index.append(0)
   if(ranges[len(ranges)-1]<ranges[0] and ranges[len(ranges)-1]<ranges[len(ranges)-2] and (np.min(np.linalg.norm(ALL_point_at_end[len(ranges)-1].T-ob_center,axis=1))<0.85)):
              valley_index.append(len(ranges)-1)
   #fig, ax = plt.subplots() # note we must use plt.subplots, not plt.subplot
   #for obb in ob:
   #     c=plt.Circle(obb[0],obb[1],color="black",fill=False)
   #     ax.add_patch(c)
   #r=2
   if(len(valley_index)!=0):
       """
       for i in range(len(ranges)):
             if(ranges[i]>=r):
                 l=r
             else:
                l=ranges[i]
             point=np.array([[loc[0,0]+l*math.cos(angles[i])],[loc[1,0]+l*math.sin(angles[i])]])
             og=loc
             if i in valley_index:
                   color='red'
             else:
                 color='blue'
             x=np.array([og[0,0],point[0,0]])
             y=np.array([og[1,0],point[1,0]])
             plt.plot(x,y,color)
       plt.show()  
       """ 
       point_at_end=[(loc+np.array([[ranges[valley_index]*math.cos(angles[valley_index])],[ranges[valley_index]*math.sin(angles[valley_index])]])) for valley_index in valley_index]
   
       dist_to_ob=np.array([np.min(np.linalg.norm(p.T-ob_center,axis=1)) for p in point_at_end])
       closest_ob=np.array([np.argmin(np.linalg.norm(p.T-ob_center,axis=1)) for p in point_at_end])
       ind=[]
       for obb in range(len(ob)):
           w=np.where(closest_ob==obb)[0]
           if w.size==0:
               continue
           d=np.array(dist_to_ob)[w]
           dd=np.argmin(d)
       
           ind.append(w[dd])
       ind=np.array(ind)
       valley_index_revised=np.array(valley_index)[ind]
       closest_ob=closest_ob[ind]    
       point_at_end=np.array(point_at_end)[ind]
       d_i=np.array(ranges)[valley_index_revised]
       #print(d_i,closest_ob)
       grad_vector=-1*(U_att_grad(loc)+U_rep_grad(d_i,closest_ob,point_at_end,loc))
       grad_list.append(grad_vector)
       
       # SMOOTHENING OF GRADINET
       if(len(grad_list)==s_m):
            
            grad_vector=sum(grad_list)/len(grad_list)
            grad_list=grad_list[0 if len(grad_list)==1 else 1:]
       
   else:
       grad_vector=-1*(U_att_grad(loc))
       grad_list.append(grad_vector)
       if(len(grad_list)==s_m):
            
            grad_vector=sum(grad_list)/len(grad_list)
            grad_list=grad_list[0 if len(grad_list)==1 else 1:]
   ang=math.atan2(grad_vector[1,0],grad_vector[0,0])
   alpha=ang-yaw
   if(alpha<-math.pi):
        alpha=2*math.pi+alpha
   if(alpha>math.pi):
        alpha=2*math.pi-alpha
   v=0.3+0*0.5*np.linalg.norm(grad_vector)/10
   w=v*(2*math.sin(alpha))/(0.35*0.35)
   if(alpha>math.pi/2 or alpha<-math.pi/2):
        w=np.sign(alpha-math.pi/2)*0.7
        v=0
   print("grad----->",grad_vector)
   print("alpha------>",alpha*180/math.pi)
   
   print("_________________________")
   move_the_bot.linear.x=v
   move_the_bot.angular.z=w          
           
    
                                      
  
             
   
 
   publish_to_cmd_vel.publish(move_the_bot) 
   arg=[]
    
    

      

    
def laserdata_callback(msg):
      global flag_laser,arg
      #print((msg.ranges[122]))
      #print(len(msg.ranges))
      if(len(msg.ranges)==360):
         
        arg.append(msg.ranges)
        #print((msg.ranges[0]))
        flag_laser=1
      else:
         arg=[]
       

    
def odom_callback(msg):
     global flag_laser
     global arg
     if(flag_laser==1):
        ori=msg.pose.pose.orientation
        pos=msg.pose.pose.position
        oril=[ori.x,ori.y,ori.z,ori.w]
        yaw=euler_from_quaternion(oril)[2]
        loc=np.array([[pos.x],[pos.y]])
        
        
        arg.append(loc)
        arg.append(yaw)
       
        flag_laser=0
        #print(arg[0])
       
        #print("arg[2]",arg[2])
        if isinstance(arg[2],tuple)==False:
               
                algo(arg[0],arg[1],arg[2])
        #move_the_bot.linear.x=0.0
        #move_the_bot.angular.z=0.3
        #arg=[]
        #publish_to_cmd_vel.publish(move_the_bot) 
        #print("x")
        
       
        



if __name__ == "__main__":
    
    rospy.init_node('turtlebot_controller_node')
    
    r=rospy.Rate(10)
    subscribe_to_laser = rospy.Subscriber('scan', LaserScan, callback = laserdata_callback)
    subscribe_to_odometry=rospy.Subscriber('odom', Odometry, callback = odom_callback)
    
    rospy.loginfo('My node has been started')
    publish_to_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    
  
    #create an object of Twist data

    move_the_bot = Twist()
    r.sleep()
    rospy.spin()


