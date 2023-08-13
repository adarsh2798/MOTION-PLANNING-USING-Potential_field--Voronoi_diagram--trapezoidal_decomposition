#!/usr/bin/env python3


#############################################################################################
# In this code the start and end point is specfied by user.
# then A path is planned bu GVD. For that the 6x12 enviroinment 
# is discretized into grid cells and BRUSHFIRE algorithm is implemented
# to form the GVD where GVD is the collision of 2 wavefronts
# originating from different obstacles. Then we have a set of (x,y) points 
# that lie on GVD. After that relevant points are sampled from that such as
# one where more than 3 wavefronts collide or a 3-way intersection
# also some points on GVD that lie around the boundary of obstacles are sampled
# these form the "vertices" of the GRAPH. TO get edges, we connect those points
# that are closest to atleast 2 same obstacles. ANd some further processing is 
# done to remove some extra edges. After this once we have the graph/ROADMAP 
# formed by GVD, we use DIJKSTRA to find shortest path where each edge weight corresponds
# to it's length. Once we have the path, we have a list of (x,y) coordinates that 
# BOT needs to follow in order to reach goal. 
# FOR THE PATH FOLLOWINF TASK, PURE-PURSUIT algorithm has been implemented                  
##############################################################################################


import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import math
import message_filters


########### SPECIFY START AND END POINT BELOW   #########################

start=[1,-5] ## REMEMBER TO SPECIFY THIS IN LAUNCH FILE BEFORE LAUNCHING GAZEBO!!!
end=[-1,5]

######################################################################

flag_laser=0
arg=[]
path_found=False
waypoints=[]
arrived=False

odom_data=[]
odom_data.append(np.array([[start[0]],[start[1]]]))






def path_finder_GVD(start,end,arrived,odom_data):
    global waypoints
    res=0.1
    x,y = np.meshgrid(np.linspace(-3+res/2,3-res/2,60),np.linspace(-6+res/2,6-res/2,120))

    y=-y
    ob=[[(0,2),0.5],[(1.5,0),0.7],[(0,-2),0.5],[(-1.5,0),0.5]]
    ob_center=[np.array(obb[0]) for obb in ob]
    ob_vertex=[[(ob_center[obc][0]+ob[obc][1]*math.cos(k*math.pi/2),ob_center[obc][1]+ob[obc][1]*math.sin(k*math.pi/2)) for  k in range(4)] for obc in range(len(ob_center))]
    ob_vertex.append([(-2.95,5.95),(2.95,5.95),(2.95,-5.95),(-2.95,-5.95)])


    dist_1=np.sqrt((x-ob_center[0][0])**2 +(y-ob_center[0][1])**2)
    dist_2=np.sqrt((x-ob_center[1][0])**2 +(y-ob_center[1][1])**2)
    dist_3=np.sqrt((x-ob_center[2][0])**2 +(y-ob_center[2][1])**2)
    dist_4=np.sqrt((x-ob_center[3][0])**2 +(y-ob_center[3][1])**2)
    bool_1=np.array(dist_1<=ob[0][1]).astype(int)
    bool_2=np.array(dist_2<=ob[1][1]).astype(int)
    bool_3=np.array(dist_3<=ob[2][1]).astype(int)
    bool_4=np.array(dist_4<=ob[3][1]).astype(int)

    bool_wall1=np.array(x==2.95)
    bool_wall2=np.array( y==-5.95)
    bool_wall3=np.array(x==-2.95)
    bool_wall4=np.array(y==5.95)
    bool_wall=bool_wall1+bool_wall2+bool_wall3+bool_wall4
    bool_mask=np.add(bool_1+bool_2,bool_3+bool_4) + bool_wall

    #  right wall=21
    #  bottom wall=22
    #  left wall=23
    #  up wall=24

    grid=bool_mask

    ind=np.where(grid==1)
    ind=[(ind[0][k],ind[1][k]) for k in range((ind[0].shape[0]))]


    closest_ob=np.ones(grid.shape) *-1
    for i in range(len(ind)):
      dist=np.array([dist_1[ind[i][0],ind[i][1]],dist_2[ind[i][0],ind[i][1]],dist_3[ind[i][0],ind[i][1]],dist_4[ind[i][0],ind[i][1]]])
      closest_ob[ind[i][0],ind[i][1]]=np.argmin(dist)
      if ind[i][0]==0 and 0<=ind[i][1]<grid.shape[1]:
        closest_ob[ind[i][0],ind[i][1]]=24
      if ind[i][0]==grid.shape[0]-1 and 0<=ind[i][1]<grid.shape[1]:
        closest_ob[ind[i][0],ind[i][1]]=22
      if ind[i][1]==0 and 1<=ind[i][0]<grid.shape[0]-1:
        closest_ob[ind[i][0],ind[i][1]]=23
      if ind[i][1]==grid.shape[1]-1 and 1<=ind[i][0]<grid.shape[0]-1:
        closest_ob[ind[i][0],ind[i][1]]=21
  
   



  
  
    

    GVD=[]
    closest_ob_GVD=[]
    def check_neighbor(grid,cell,curr_depth):
      i_seq=[0,1,1,1,0,-1,-1,-1]
      j_seq=[1,1,0,-1,-1,-1,0,1]
      spread=4
  
      for n1 in range(spread):
        n=int(n1*8/spread)
    
        
        neighbor_i=cell[0]+i_seq[n]
        neighbor_j=cell[1]+j_seq[n]
    
        if not (0 <= neighbor_i < grid.shape[0] and 0 <= neighbor_j < grid.shape[1]):
          continue
        elif grid[neighbor_i,neighbor_j]==0:
               grid[neighbor_i,neighbor_j]=curr_depth+1
               closest_ob[neighbor_i,neighbor_j]=closest_ob[cell[0],cell[1]]
        elif grid[neighbor_i,neighbor_j]>1 and closest_ob[neighbor_i,neighbor_j]!=closest_ob[cell[0],cell[1]] :

          if not([x[0,neighbor_j],y[neighbor_i,0]] in GVD):
              GVD.append([x[0,neighbor_j],y[neighbor_i,0]])
              closest_ob_GVD.append([closest_ob[neighbor_i,neighbor_j]])
          closest_ob_GVD[GVD.index([x[0,neighbor_j],y[neighbor_i,0]])].append(closest_ob[cell[0],cell[1]])


    

    def brushfire(grid):
      curr_depth=1
      grid_visited=False

      while not(grid_visited):
       
          ind=np.where(grid==curr_depth)
          ind=[(ind[0][k],ind[1][k]) for k in range((ind[0].shape[0]))]
      
          for c in ind:
            check_neighbor(grid,c,curr_depth)
          curr_depth+=1
          if not(0 in grid):
            grid_visited=True
      
      ind=np.where(grid==curr_depth)
      ind=[(ind[0][k],ind[1][k]) for k in range((ind[0].shape[0]))]
      for c in ind:
            check_neighbor(grid,c,curr_depth)


    brushfire(grid)
   
    def extract_vertex_edges(closest_ob_GVD):
        
        p_x=[GVD[p][0] for p in range (len(GVD))]
        p_y=[GVD[p][1] for p in range (len(GVD))]






        closest_ob_GVD=[[*set(l)] for l in closest_ob_GVD]
        closest_ob_GVD_num=[len(closest_ob_GVD[i]) for i in range(len(closest_ob_GVD))]





        p_x_mod=list(np.array(p_x)[np.where(np.array(closest_ob_GVD_num)>2)])
        p_y_mod=list(np.array(p_y)[np.where(np.array(closest_ob_GVD_num)>2)])

        for obv in ob_vertex:
          for v in range(4):
            dist=np.array([np.sqrt((obv[v][0]-GVD[i][0])**2 +  (obv[v][1]-GVD[i][1])**2 ) for i in range(len(GVD))])
            ind=np.argmin(dist)
            p_x_mod.append(GVD[ind][0])
            p_y_mod.append(GVD[ind][1])


        points=[[p_x_mod[0],p_y_mod[0]]]
        for k in range(len(p_x_mod)):
          bigger=True
          for p in points:
            if ((p_x_mod[k]-p[0])**2 +(p_y_mod[k]-p[1])**2)**0.5<=0.4:
              bigger=False
              break
          if bigger:
            points.append([p_x_mod[k],p_y_mod[k]])

        points_x=[pp[0] for pp in points]
        points_y=[pp[1] for pp in points]
        edges=[]
        vertex_deg=[0 for i in range(len(points))]
        edge_flag=[]



        counter=0
        for p1 in points:
            edges.append([p1])
            edge_flag.append([p1])
    
            points_sort=points[:]
            points_sort.sort(key=lambda x:((x[0]-p1[0])**2 +(x[1]-p1[1])**2)**0.5)
            print(points.index(p1))
            for p2 in points_sort:
              if p2==p1:
                continue
              else:
                ind1=GVD.index(p1)
                ind2=GVD.index(p2)
                l1=closest_ob_GVD[ind1]
                l2=closest_ob_GVD[ind2]
                l_inter=list(set(l1) & set(l2))
                if len(l_inter)>=2:

                  dist_center=np.array([abs(  ( (p2[0]-p1[0])*(p1[1]-obc[1]) - (p2[1]-p1[1])*(p1[0]-obc[0]) )   /    (   (p2[0]-p1[0])**2 +  (p2[1]-p1[1])**2    )**0.5     ) for obc in ob_center])
          
                  flag=False
                  for poi in points:
                    if min(p1[0],p2[0])< poi[0] < max(p1[0],p2[0]) and min(p1[1],p2[1])< poi[1]< max(p1[1],p2[1] and    ((p1[0]-poi[0])**2 +(p1[1]-poi[1]))**.5 <  ((p1[0]-p2[0])**2 +(p1[1]-p2[1])**2)**.5    ):
              
              
                           flag=True
                           break
            
                  if counter==0:
                    print(p1,p2)  
                  if vertex_deg[points.index(p1)] == closest_ob_GVD_num[GVD.index(p1)]:
                     break
                  #if [p1,closest_ob_GVD[GVD.index(p2)]] in edge_flag:
                  if ([p1,l_inter] in edge_flag) and flag==True : 
                    continue
          
                  #np.min(dist_center)<=ob[np.argmin(dist_center)][1]
                  #if flag==True :
            
                  #  continue
                  if counter==0:
                    print(p1,p2)  
                  if(len(edges[-1])==1):
                         edges[-1].append(p2)
                         vertex_deg[points.index(p1)]+=1
                 
                         #edge_flag[-1].append(closest_ob_GVD[GVD.index(p2)])
                         edge_flag[-1].append(l_inter)
                  else:
                    edges.append([p1,p2])
                    vertex_deg[points.index(p1)]+=1
            
                    #edge_flag.append([p1,closest_ob_GVD[GVD.index(p2)]])
                    edge_flag.append([p1,l_inter])
    
            counter+=1      
        return (points,edges)


    points,edges=extract_vertex_edges(closest_ob_GVD)
    """
    print(len(edges))
    fig, ax=plt.subplots(dpi=4*72)
    env=matplotlib.patches.Rectangle((-3,-6),6,12,color='white')
    ax.add_patch(env)
    for obb in ob:
  
      ob_i=matplotlib.patches.Circle(obb[0],radius=obb[1],color='black')
      ax.add_patch(ob_i)
    for e in edges:
      if len(e)==1:
        continue
      plt.plot([e[0][0],e[1][0]],[e[0][1],e[1][1]],color='red',linewidth=0.5)

    plt.scatter([pp[0] for pp in points],[pp[1] for pp in points], s=16.9)
    ax.set_aspect('equal')
    plt.xlim([-3,3])
    plt.ylim([-6,6])
    plt.show()
    """
    
    
    class Graph():
      def __init__(self,vertices):
        self.v=vertices
        self.graph=[[0 for j in range(self.v)] for i in range(self.v)]
  
      def print_sol(self,Set):
        print(Set)
        #for s in Set:
        #  print(points[s])
      def path(self,parent,source,dest):
        path_set=[]
    
        node=dest
        while node!=source:
          path_set.append(node)
          node=parent[node]
        path_set.append(source)
        #self.print_sol(path_set[::-1])
        return path_set

      def mindist_node_not_in_sptSet(self,dist,sptSet):
           thresh=float(math.inf)
           for i  in range(self.v):
             if dist[i]<thresh and i not in sptSet:
               thresh=dist[i]
               node=i
           return node
      def dijkstra(self,source,dest):
        parent=[-1 for i in range(self.v)]
        parent[0]=0
        sptSet=[]
        dist=[math.inf]*self.v
        dist[source]=0
        for  k in range(self.v):
          u=self.mindist_node_not_in_sptSet(dist,sptSet)
          sptSet.append(u)
          for j in range(self.v):
            if self.graph[u][j]>0 and j not in sptSet and dist[j]> dist[u]+self.graph[u][j]:
              dist[j]=dist[u]+self.graph[u][j]
              parent[j]=u
        path_set=self.path(parent,source,dest)
        return path_set


    vertices=len(points)
    G=np.zeros((vertices,vertices))
    v_list=[]
    for e in edges:
      u=points.index(e[0])
      v=points.index(e[1])
      G[u,v]=(((e[0][0]-e[1][0])**2 +(e[0][1]-e[1][1])**2)**0.5)
      v_list.append(points[u])
  
    start=start
    end=end
    dist_start=[  ((start[0]-p[0])**2 +(start[1]-p[1])**2)**0.5 for p in points     ]
    dist_end=[  ((end[0]-p[0])**2 +(end[1]-p[1])**2)**0.5 for p in points     ]
    source=dist_start.index(min(dist_start))
    dest=dist_end.index(min(dist_end))




    g = Graph(vertices)

    g.graph=G

  
    path_set=g.dijkstra(source,dest)
    #print(points[0],points[5])
    if arrived:
       fig, ax=plt.subplots(dpi=4*72)
       env=matplotlib.patches.Rectangle((-3,-6),6,12,color='white')
       ax.add_patch(env)
       for obb in ob:
  
         ob_i=matplotlib.patches.Circle(obb[0],radius=obb[1],color='black')
         ax.add_patch(ob_i)
       p_x_mod=[pp[0] for pp in points]
       p_y_mod=[pp[1] for pp in points]
       if len(points)>100:
         ss=0.2
       else:
         ss=16.9
       plt.scatter(p_x_mod,p_y_mod,s=ss)
       edges_mod=[]
       [edges_mod.append(x) for x in edges if (   ([x[0],x[1]]  not in edges_mod) and ([x[1],x[0]]  not in edges_mod)  )  ]
 
       edges_dijk=[]
       [edges_dijk.append(x) for x in edges if ( x not in edges_dijk  )  ]
 

       for e in edges:
         if len(e)==1:
           continue
         plt.plot([e[0][0],e[1][0]],[e[0][1],e[1][1]],color='red',linewidth=0.5)
       for p in range(len(path_set)-1):
     
         p1=points[path_set[p]]
         p2=points[path_set[p+1]]
         plt.plot([p1[0],p2[0]],[p1[1],p2[1]],color='green',linewidth=2)
         
       odom_data=np.array(odom_data)
       odom_data=np.reshape(odom_data,(odom_data.shape[0],odom_data.shape[1]))
       x_odom_data=odom_data[:,0]
       y_odom_data=odom_data[:,1]
       plt.plot(x_odom_data,y_odom_data,'blue',linestyle='--',linewidth=2)
             
            
       
       plt.plot([start[0],points[source][0]],[start[1],points[source][1]],color='green',linewidth=2)
       
       plt.plot([end[0],points[dest][0]],[end[1],points[dest][1]],color='green',linewidth=2)
       
       plt.title("GVD SHOWN IN 'RED' ; PATH BY DIJKSTRA IN 'GREEN' ; PATH TAKEN BY BOT IN 'DASHED BLUE'",fontsize=5)
       ax.set_aspect('equal')
       plt.xlim([-3,3])
       plt.ylim([-6,6])
       plt.show()
    for p in range(len(path_set)-1):
     
         p1=points[path_set[p]]
         p2=points[path_set[p+1]]
         
         waypoints.append(p1)
    waypoints.append(points[path_set[len(path_set)-1]])
    waypoints=waypoints[::-1]
    waypoints.insert(0,start)
    
    waypoints.append(end)
    return waypoints
    







def PURE_PURSUIT(yaw,pose, waypoints):
   
    print(waypoints)
    flag = 0
    max_wRef = 1000
    vRef = 0.3
    t1 = 0
    t2 = 1
    wpx = waypoints[:, 0]
    wpy = waypoints[:, 1]
    dist_from_wp = np.sqrt((pose[0, 0] - wpx)**2 + (pose[1, 0] - wpy)**2)
    closest_wp_index = np.argmin(dist_from_wp)
    closest_wp = waypoints[closest_wp_index, :]
    s = waypoints.shape
    next_wp = waypoints[closest_wp_index, :]
    if closest_wp_index != s[0]-1:
        next_wp = waypoints[closest_wp_index+1, :]
    if closest_wp_index == s[0]-1:
        closest_wp = pose[0:2, 0].T
        next_wp = waypoints[closest_wp_index, :]
    E = closest_wp.T
    TP = E
    L = next_wp.T
    last_TP_on_path = E
    C = pose[0:2, 0]
    d = L - E
    f = E - C
    l = 0.35  # lookahead distance
    a = np.linalg.norm(d)**2
    b = 2 * np.dot(f, d)
    c = np.linalg.norm(f)**2 - l**2
    D = b**2 - 4*a*c
    #print(E,L,a,b,c)
    if D < 0:
        flag = 1
    if D >= 0:
        D = np.sqrt(D)
        t1 = (-b - D) / (2*a)
        t2 = (-b + D) / (2*a)
        if (t1 < 0 or t1 > 1) and (t2 < 0 or t2 > 1):
            flag = 1
    D = t2
    if flag == 0:
        if D == 0:
            TP = E + t1*d
        if D > 0:
            if (t1 < 0 or t1 > 1) and (t2 >= 0 and t2 <= 1):
                TP = E + t2*d
            if (t1 >= 0 and t1 <= 1) and (t2 < 0 or t2 > 1):
                TP = E + t1*d
            if (t1 >= 0 and t1 <= 1) and (t2 >= 0 and t2 <= 1):
                TP1 = E + t1*d
                TP2 = E + t2*d
                TPs = np.array([TP1.T, TP2.T])
                TPx = TPs[:, 0]
                TPy = TPs[:, 1]
                dist_from_L = np.sqrt((L[0] - TPx)**2 + (L[1] - TPy)**2)
                closest_TP_index = np.argmin(dist_from_L)
                TP = TPs[closest_TP_index, :].T
        last_TP_on_path = TP
    if(flag==1):
        norm1 = np.linalg.norm(pose[0:2,0]-last_TP_on_path)
        norm2 = np.linalg.norm(pose[0:2,0]-E)
        if(norm1<=norm2):
            L=last_TP_on_path
        if(norm1>norm2):
            L=E
        E=pose[0:2,0]
        d=L-E
        TP=E+l*(d)/np.linalg.norm(d)
        #TP=last_TP_on_path
    TP_bot_x=0
    t_dir=0
    distance_from_TP=np.linalg.norm(pose[0:2,0]-TP)
    vv=TP-pose[0:2,0]
    
    
    ang=math.atan2(vv[1],vv[0])
    alpha=ang-yaw
    if(alpha<-math.pi):
         alpha=2*math.pi+alpha
    if(alpha>math.pi):
         alpha=2*math.pi-alpha
    v=0.3
    w=v*(2*math.sin(alpha))/(l*l)
    if(alpha>math.pi/2 or alpha<-math.pi/2):
         w=np.sign(alpha-math.pi/2)*0.7
         v=0
    
    
    """
    phi=np.arctan2(vv[1],vv[0])
    if(phi<0):
        phi=2*np.pi+phi
    ang=phi-(np.sign(pose[1,0]))*pose[1,0]
    if(ang<0):
        ang=2*np.pi+ang  
    if(ang>np.pi):
        TP_bot_x=-l*np.cos((np.pi/2)+(ang-2*np.pi))
    if(ang<=np.pi):
        TP_bot_x=l*np.cos((np.pi/2)-ang)
    
     
    
    #TP_bot_x=sign(phi)*distance_from_TP*cos(abs(phi)+sign(phi)*((pi/2)-pose[2,k-1)));
    vRef=0.75
    R=(l**2)/(2*(TP_bot_x))
    delta=np.arctan(2*TP_bot_x*(R-TP_bot_x)/(l**2))
    wRef =(vRef)/(R)
    if(ang==np.pi):
        wRef=max_wRef
        vRef=0
    #disp(wRef)
    if(abs(wRef)>max_wRef):
        wRef=np.sign(wRef)*max_wRef
    flag=0
    w=wRef/10
    v=vRef
    v=0.3
    """
    
    return v,w














def algo(ranges,loc,yaw):
   
   pose=loc
   global arg,path_found,start,end,arrived,waypoints,odom_data
   if not(path_found):
        waypoints_list=path_finder_GVD(start,end,arrived,odom_data)
        print(len(waypoints_list))
        waypoints=np.array(waypoints_list)
        waypoints=waypoints.reshape((len(waypoints_list),2))
        
        path_found=True
        
   else:
       if ((pose[0,0]-end[0])**2 +(pose[1,0]-end[1])**2)**0.5<=0.5:  ### if too close to goal STOP
           v=0
           w=0
           arrived=True
           waypoints_list=path_finder_GVD(start,end,arrived,odom_data)
           
           
       else:
           
           v,w=PURE_PURSUIT(yaw,pose,waypoints)
           if(np.linalg.norm(pose-odom_data[-1])>=0.1):
             odom_data.append(loc)
           
           print(v,w)
       print("dist to goal ---->",((pose[0,0]-end[0])**2 +(pose[1,0]-end[1])**2)**0.5)
      

      

     
   

  
       
       """
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
       """
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


