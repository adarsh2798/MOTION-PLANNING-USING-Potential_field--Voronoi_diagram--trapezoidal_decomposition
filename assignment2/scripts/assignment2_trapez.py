#!/usr/bin/env python3


#############################################################################################
#   AT TIMES GETS STUCK AT A POSITION WHILE FOLLOWING THE PATH FOR 10-15s(SOME BUG IN PURE PURSUIT IMPLEMENT) 
#   BUT DONT CLOSE THE SIMULATION. IT DOES WORK AFTER THOSE 10-15s              
##############################################################################################


import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl
from collections import defaultdict
import message_filters


########### SPECIFY START AND END POINT BELOW   #########################

start=[-4.3,2] ## REMEMBER TO SPECIFY THIS IN LAUNCH FILE BEFORE LAUNCHING GAZEBO!!!
end=[5,-2]

######################################################################

flag_laser=0
arg=[]
path_found=False
waypoints=[]
arrived=False

odom_data=[]
odom_data.append(np.array([[start[0]],[start[1]]]))






def path_finder_TRAPEZ(start,end,arrived,odom_data):
    global waypoints
    graph = defaultdict(list)
    if arrived:
       fig = plt.figure(dpi=5*72)
       ax = fig.add_subplot(111)
    class vertex:
      def __init__(self,x,y):
        self.x=x
        self.y=y
    class edge:
      def __init__(self,start,end):  #start,end are point class objects
        self.start=start
        self.end=end




    def find_Centroid(v):
	    cent = [0,0]

	    n = len(v)
	    Area = 0

	
	    for i in range(n):

		    x0 = v[i][0]
		    y0 = v[i][1]
		    x1 = v[(i + 1) % n][0]
		    y1 =v[(i + 1) % n][1]

		    A = (x0 * y1) - (x1 * y0)
		    Area += A

		    cent[0] += (x0 + x1) * A
		    cent[1] += (y0 + y1) * A

	    Area *= 0.5
	    cent[0] = (cent[0]) / (6 * Area)
	    cent[1] = (cent[1]) / (6 * Area)

	    return cent





    class trapezoid:
      def __init__(self,left,right,top_a,bottom_a,top_b,bottom_b):
        self.left=left
        self.right=right
        self.top_a=top_a
        self.bottom_a=bottom_a
        self.top_b=top_b
        self.bottom_b=bottom_b
        if self.left!=self.right:
          cent=find_Centroid(  [  [self.right,self.top_b], [self.left,self.top_a] , [self.left,self.bottom_a], [self.right,self.bottom_b]   ]    )
          self.centroid=vertex(cent[0],cent[1])
        else:
          self.centroid=vertex(self.left,(self.top_a+self.bottom_a)/2)
   



    class Node: 
        def __init__(self,key):
          self.val=key
          self.left=None
          self.right=None



    edges=[  [(-5.853, 2.448),(3.873, 2.448)] , [(3.873, 2.448),(6.107, -2.448)],  [(6.107, -2.448),(-3.619, -2.524)],   [(-3.619, -2.524),(-5.853, 2.448)] ,
             [(-1.528,-0.1877),(-1.99,0.06)],   [(-1.99,0.06), (-2.47,-0.81)]  ,  [(-2.47,-0.81), (-2,-1.06)]         ,  [(-2,-1.06),(-1.528,-0.1877) ],
             [(0.5,0.41), (1,0.7)],             [(1,0.7), (0.5,1.58)],             [(0.5,1.58),(-0.007,1.28)],             [(-0.007,1.28),(0.5,0.41)],
             [(3.1,0.3),(2.1,0.6)],             [(2.1,0.6), (1.8,-0.3)],          [(1.8,-0.3),(2.8,-0.6)],               [(2.8,-0.6),(3.1,0.3)]               ]     
    vertices=[]
    vertices=[e[0] for e in edges if e[0] not in vertices]
    vertices.sort(key=lambda x:x[0])
    edges_object=[edge(vertex(e[0][0],e[0][1]), vertex(e[1][0],e[1][1])) for e in edges ]
    vertices_object=[vertex(v[0],v[1]) for v in vertices]
    vertices_object.sort(key=lambda v:v.x)

    def CASE(e1,e2,v_o):
      if e1.end.x>v_o.x and e2.start.x>v_o.x:   #add both
        return 1
      if e1.end.x<v_o.x and e2.start.x<v_o.x:   #del both
        return 2
      if e1.end.x<v_o.x and e2.start.x>v_o.x:   #del e1 add e2
        return 3
      if e1.end.x>v_o.x and e2.start.x<v_o.x:   #del e2 add e1
        return 4
    def inter(v_o,dir,L):

      inters=[]
      for l in L:
        S1=np.array([ [l.start.x],[l.start.y]   ])
        E1=np.array([ [l.end.x],[l.end.y]   ])
        S2=np.array([  [v_o.x], [v_o.y] ])
        d=max(np.linalg.norm(S2-S1), np.linalg.norm(S2-E1))
        E2=S2+d*np.array([  [0],   [dir]])
        A=np.array(  [ [  -(E1[0,0]-S1[0,0])  , (E2[0,0]-S2[0,0])      ] , [-(E1[1,0]-S1[1,0])  , (E2[1,0]-S2[1,0])] ]   )
        B=S1-S2
        t=np.matmul(np.linalg.inv(A),B)
    
    
        if 0<=t[0,0]<=1 and 0<=t[1,0]<=1:
          inter_point=S1+t[0,0]*(E1-S1)
      
      
          if abs(inter_point[0,0]-v_o.x)<1e-3 and abs(inter_point[1,0]-v_o.y)<1e-3:
            continue
          inters.append(vertex(inter_point[0,0],inter_point[1,0]))

      return (min(inters,key=lambda i: abs(i.y-v_o.y) ))

    def inter_obstacle(edges_object,segment):
      flag=False
      for e_o in edges_object:
        S1=np.array([ [e_o.start.x],[e_o.start.y]   ])
        E1=np.array([ [e_o.end.x],[e_o.end.y]   ])
        S2=np.array([  [segment.start.x], [segment.start.y] ])
        E2=np.array([  [segment.end.x], [segment.end.y] ])
    
        A=np.array(  [ [  -(E1[0,0]-S1[0,0])  , (E2[0,0]-S2[0,0])      ] , [-(E1[1,0]-S1[1,0])  , (E2[1,0]-S2[1,0])] ]   )
        B=S1-S2
        t=np.matmul(np.linalg.inv(A),B)
    
    
        if 0<=t[0,0]<=1 and 0<=t[1,0]<=1:
          flag=True
          return edge
      return flag




    ######  building graph #################
    def addEdge(graph,u,v):
      if graph[u]==[None]:
        graph[u].append(v)
        graph[u]=graph[u][1:]
      else: 
        graph[u].append(v)
  
    def get_edges(graph):
        edges = []
  
    
        for node in graph:
          
        
            for neighbour in graph[node]:
              
            
                edges.append((node, neighbour))
        return edges
    ##############################################



    L=[]
    for e_o in edges_object:
        if e_o.start.x==vertices_object[0].x and e_o.start.y==vertices_object[0].y:
          e1=e_o
          L.append(e1)
        if e_o.end.x==vertices_object[0].x and e_o.end.y==vertices_object[0].y:
          e2=e_o
          L.append(e2)

    v_o_c=1
    trapz_node=1
    trapz_list=[]
    for v_o in vertices_object[1:-1]:
      for e_o in edges_object:
        if e_o.start.x==v_o.x and e_o.start.y==v_o.y:
          e1=e_o
        if e_o.end.x==v_o.x and e_o.end.y==v_o.y:
          e2=e_o
 
      c=CASE(e1,e2,v_o)
      if c==3 or c==4:
          inter1=vertex(v_o.x,v_o.y)
          if v_o_c!=1 and v_o_c!=len(vertices_object)-2:
            if e1.end.y<v_o.y or e2.start.y<v_o.y:
              inter2=inter(v_o,1,L)
            else:
              inter2=inter(v_o,-1,L)
          else:
            if e1.end.y>v_o.y or e2.start.y>v_o.y:
              inter2=inter(v_o,1,L)
            else:
              inter2=inter(v_o,-1,L)

          if c==3:
            del L[L.index(e1)]
            L.append(e2)
          if c==4:
            del L[L.index(e2)]
            L.append(e1)
      if c==1 or c==2:
          inter1= inter(v_o,-1,L)
          inter2= inter(v_o,1,L)
          if c==1:
        
            L.append(e1)
            L.append(e2)
          if c==2:
            del L[L.index(e1)]
            del L[L.index(e2)]



      if v_o_c==1:
        if c==3 or c==4:
         trapz_line=trapezoid(v_o.x,v_o.x,max(inter1.y,inter2.y),min(inter1.y,inter2.y),max(inter1.y,inter2.y),min(inter1.y,inter2.y))
         trapz_trapz=trapezoid(vertices_object[0].x,v_o.x,vertices_object[0].y,vertices_object[0].y,max(inter1.y,inter2.y),min(inter1.y,inter2.y))    
         trapz_list.append([trapz_node,trapz_trapz])
         trapz_list.append([trapz_node+1,trapz_line])
         addEdge(graph,trapz_node,trapz_node+1)
         addEdge(graph,trapz_node+1,None)
         trapz_node+=2
      elif(v_o_c>1):
        if c==1:
          trapz_line1=trapezoid(v_o.x,v_o.x,max(inter1.y,inter2.y),v_o.y,max(inter1.y,inter2.y),v_o.y)
          trapz_line2=trapezoid(v_o.x,v_o.x,v_o.y,min(inter1.y,inter2.y),v_o.y,min(inter1.y,inter2.y))
          leaf_key=[k for k,v in graph.items() if v==[None]] #extract leaf nodes
          tt=[]
          kk=[]
          for k in leaf_key:
            t=[t_l[1] for t_l in trapz_list if t_l[0]==k]
            segment1=edge(trapz_line1.centroid,t[0].centroid)
            segment2=edge(trapz_line2.centroid,t[0].centroid)
            flag1=inter_obstacle(edges_object,segment1)
            flag2=inter_obstacle(edges_object,segment2)
            if flag1==False and flag2==False:
              tt.append(t[0])
              kk.append(k)
          trapz_trapz=trapezoid(tt[0].left,v_o.x,max([ttt.top_a for ttt in tt]),min([ttt.bottom_a for ttt in tt]), max(inter1.y,inter2.y),min(inter1.y,inter2.y)   )
          trapz_list.append([trapz_node,trapz_trapz])
          trapz_list.append([trapz_node+1,trapz_line1])
          trapz_list.append([trapz_node+2,trapz_line2])
          for kkk in kk:
            addEdge(graph,kkk,trapz_node)
          addEdge(graph,trapz_node,trapz_node+1)
          addEdge(graph,trapz_node,trapz_node+2)
          addEdge(graph,trapz_node+1,None)
          addEdge(graph,trapz_node+2,None)
          trapz_node+=3
        if c==2:
          trapz_line1=trapezoid(v_o.x,v_o.x,max(inter1.y,inter2.y),v_o.y,max(inter1.y,inter2.y),v_o.y)
          trapz_line2=trapezoid(v_o.x,v_o.x,v_o.y,min(inter1.y,inter2.y),v_o.y,min(inter1.y,inter2.y))
          leaf_key=[k for k,v in graph.items() if v==[None]] #extract leaf nodes
          for k in leaf_key:
            t=[t_l[1] for t_l in trapz_list if t_l[0]==k]
            segment1=edge(trapz_line1.centroid,t[0].centroid)
            segment2=edge(trapz_line2.centroid,t[0].centroid)
            flag1=inter_obstacle(edges_object,segment1)
            flag2=inter_obstacle(edges_object,segment2)
            if flag1==False:
              trapz_trapz=trapezoid(t[0].left,v_o.x,t[0].top_a,t[0].bottom_a,trapz_line1.top_b,trapz_line1.bottom_b)
              trapz_list.append([trapz_node,trapz_trapz])
              trapz_list.append([trapz_node+1,trapz_line1])
              addEdge(graph,k,trapz_node)
              addEdge(graph,trapz_node,trapz_node+1)
              addEdge(graph,trapz_node+1,None)



          
            if flag2==False:
              trapz_trapz=trapezoid(t[0].left,v_o.x,t[0].top_a,t[0].bottom_a,trapz_line2.top_b,trapz_line2.bottom_b)
              trapz_list.append([trapz_node+2,trapz_trapz])
              trapz_list.append([trapz_node+3,trapz_line2])
              addEdge(graph,k,trapz_node+2)
              addEdge(graph,trapz_node+2,trapz_node+3)
              addEdge(graph,trapz_node+3,None)

          trapz_node+=4
        if c==3 or c==4:
          trapz_line=trapezoid(v_o.x,v_o.x,max(inter1.y,inter2.y),min(inter1.y,inter2.y),max(inter1.y,inter2.y),min(inter1.y,inter2.y))
          leaf_key=[k for k,v in graph.items() if v==[None]] #extract leaf nodes
          tt=[]
          kk=[]
          for k in leaf_key:
               t=[t_l[1] for t_l in trapz_list if t_l[0]==k]
               segment=edge(trapz_line.centroid,t[0].centroid)
               flag=inter_obstacle(edges_object,segment)
               if flag==False:
                  tt.append(t[0])
                  kk.append(k)
          trapz_trapz=trapezoid(tt[0].left,v_o.x,max([ttt.top_a for ttt in tt]),min([ttt.bottom_a for ttt in tt]), max(inter1.y,inter2.y),min(inter1.y,inter2.y)   )
          trapz_list.append([trapz_node,trapz_trapz])
          trapz_list.append([trapz_node+1,trapz_line])
          for kkk in kk:
            addEdge(graph,kkk,trapz_node)
          addEdge(graph,trapz_node,trapz_node+1)
          addEdge(graph,trapz_node+1,None)
          trapz_node+=2




           


           








        









      if arrived:
         plt.plot([inter1.x,inter2.x],[inter1.y,inter2.y],'red',linewidth=0.5)    
      v_o_c+=1
      


    trapz_trapz=trapezoid(vertices_object[-2].x,vertices_object[-1].x,max(inter1.y,inter2.y),min(inter1.y,inter2.y),vertices_object[-1].y,vertices_object[-1].y)    
    trapz_list.append([trapz_node,trapz_trapz])

    leaf_key=[k for k,v in graph.items() if v==[None]] #extract leaf nodes
    for k in leaf_key:
       t=[t_l[1] for t_l in trapz_list if t_l[0]==k]
       segment=edge(trapz_trapz.centroid,t[0].centroid)
       flag=inter_obstacle(edges_object,segment)
       if flag==False:
         addEdge(graph,k,trapz_node)
    addEdge(graph,trapz_node,None)


    
    graph_vertices=[]
    graph_edges=[]
    for k,v in graph.items():
      tk=[t_l[1] for t_l in trapz_list if t_l[0]==k]
      graph_vertices.append((tk[0].centroid.x,tk[0].centroid.y))
      if k==trapz_node:
        break
      for vv in v:
        tvv=[t_l[1] for t_l in trapz_list if t_l[0]==vv]
        graph_edges.append([(tk[0].centroid.x,tk[0].centroid.y), (tvv[0].centroid.x,tvv[0].centroid.y)    ])
        graph_edges.append([(tvv[0].centroid.x,tvv[0].centroid.y),(tk[0].centroid.x,tk[0].centroid.y)     ])
        if arrived:
           plt.plot([tk[0].centroid.x,tvv[0].centroid.x    ],[tk[0].centroid.y,tvv[0].centroid.y], "blue",linewidth=0.5)
           plt.scatter([tk[0].centroid.x,tvv[0].centroid.x    ],[tk[0].centroid.y,tvv[0].centroid.y],s=5,c="black")

       
  


    ##################################################################################################################################
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



    G=np.zeros((len(graph_vertices),len(graph_vertices)))
    v_list=[]
    for e in graph_edges:
      u=graph_vertices.index(e[0])
      v=graph_vertices.index(e[1])
      G[u,v]=(((e[0][0]-e[1][0])**2 +(e[0][1]-e[1][1])**2)**0.5)
      v_list.append(graph_vertices[u])
  
    start=start
    end=end
    dist_start=[  ((start[0]-p[0])**2 +(start[1]-p[1])**2)**0.5 for p in graph_vertices    ]
    dist_end=[  ((end[0]-p[0])**2 +(end[1]-p[1])**2)**0.5 for p in graph_vertices   ]
    dist_start_sorted=sorted(dist_start)
    dist_end_sorted=sorted(dist_end)
    vertices_start=sorted(graph_vertices,key=lambda d: ((start[0]-d[0])**2 +(start[1]-d[1])**2)**0.5)
    vertices_end=sorted(graph_vertices,key=lambda d: ((end[0]-d[0])**2 +(end[1]-d[1])**2)**0.5)
    for d in range(len(dist_start_sorted)):
      flag=inter_obstacle(edges_object,   edge( vertex(vertices_start[d][0],vertices_start[d][1]), vertex(start[0],start[1]) ) )
      if flag==False:
        source=graph_vertices.index(vertices_start[d])
        break
    for d in range(len(dist_end_sorted)):
      flag=inter_obstacle(edges_object,   edge( vertex(vertices_end[d][0],vertices_end[d][1]), vertex(end[0],end[1]) ) )
      if flag==False:
        dest=graph_vertices.index(vertices_end[d])
        break




    g = Graph(len(graph_vertices))

    g.graph=G

    """
    g.graph = [[0, 4, 0, 0, 0, 0, 0, 8, 0],
		[4, 0, 8, 0, 0, 0, 0, 11, 0],
		[0, 8, 0, 7, 0, 4, 0, 0, 2],
		[0, 0, 7, 0, 9, 14, 0, 0, 0],
		[0, 0, 0, 9, 0, 10, 0, 0, 0],
		[0, 0, 4, 14, 10, 0, 2, 0, 0],
		[0, 0, 0, 0, 0, 2, 0, 1, 6],
		[8, 11, 0, 0, 0, 0, 1, 0, 7],
		[0, 0, 2, 0, 0, 0, 6, 7, 0]
		]

    """
    path_set=g.dijkstra(source,dest)



 
    waypoints=[]
    if arrived:
       plt.title("DASHED: ODOM DATA OF BOT; GREEN: DIJKSTRA PATH; RED: TRAPEZ_DECOMP",fontsize=5)
    for p in range(len(path_set)-1):
      p1=graph_vertices[path_set[p]]
  
      p2=graph_vertices[path_set[p+1]]
      if arrived:
         plt.plot([p1[0],p2[0]],[p1[1],p2[1]],color='green',linewidth=1.3)
      waypoints.append(p1) 
    if arrived:
       plt.plot([start[0],graph_vertices[source][0]],[start[1],graph_vertices[source][1]],color='green',linewidth=1.3)
       plt.plot([end[0],graph_vertices[dest][0]],[end[1],graph_vertices[dest][1]],color='green',linewidth=1.3)
    waypoints.append(graph_vertices[path_set[len(path_set)-1]])
    waypoints=waypoints[::-1]
    waypoints.insert(0,start)
    waypoints.append(end)
    
    waypoints_mod=[[waypoints[0][0],waypoints[0][1]]]
    for w in waypoints:
      bigger=True
      for ww in waypoints_mod:
        
         if ((ww[0]-w[0])**2 +(ww[1]-w[1])**2)**0.5<0.4:
             bigger=False
             break
      if bigger:
             waypoints_mod.append([w[0],w[1]])
      for wp_m in range(len(waypoints_mod)-1):
            if ((waypoints_mod[wp_m][0]-waypoints_mod[wp_m+1][0])**2 +(waypoints_mod[wp_m][1]-waypoints_mod[wp_m+1][1])**2)**0.5>1:
                   point=[(waypoints_mod[wp_m][0]+waypoints_mod[wp_m+1][0])/2,(waypoints_mod[wp_m][1]+waypoints_mod[wp_m+1][1])/2]
                   waypoints_mod.insert(wp_m+1,point)
            
    #####################################################################################################################################






    odom_data=np.array(odom_data)
    odom_data=np.reshape(odom_data,(odom_data.shape[0],odom_data.shape[1]))
    x_odom_data=odom_data[:,0]
    y_odom_data=odom_data[:,1]
    if arrived:
       plt.plot(x_odom_data,y_odom_data,'blue',linestyle='--',linewidth=1.3)






    #fig, ax=plt.subplots(dpi=5*72)


    if arrived:
       for e in edges:
        
           plt.plot([e[0][0],e[1][0]],[e[0][1],e[1][1]],color='black',linewidth=2)


       ax.set_aspect('equal')
       plt.xlim([-7,7])
       plt.ylim([-3,3])
       plt.show()
    return waypoints_mod
    







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
         w=np.sign(alpha-math.pi/2)*0.3
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
        waypoints_list=path_finder_TRAPEZ(start,end,arrived,odom_data)
        print(len(waypoints_list))
        waypoints=np.array(waypoints_list)
        waypoints=waypoints.reshape((len(waypoints_list),2))
        
        path_found=True
        
   else:
       if ((pose[0,0]-end[0])**2 +(pose[1,0]-end[1])**2)**0.5<=0.5:  ### if too close to goal STOP
           v=0
           w=0
           arrived=True
           waypoints_list=path_finder_TRAPEZ(start,end,arrived,odom_data)
           
           
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


