from heapq import heappush, heappop # for priority queue
import math
import random
import numpy as np
import cv2

#Teams can add other helper functions
#which can be added here

def play(img):
    # MAIN
    dirs = 4 # number of possible directions to move on the map

    dx = [1, 0, -1, 0]
    dy = [0, 1, 0, -1]



    n = 54 # horizontal size of the map
    m = 48 # vertical size of the map
    the_map = []
    row = [0] * n
    for i in range(m): # create empty map
       the_map.append(list(row))

    for i in range(m):
        for j in range(n):
            the_map[i][j]=1
    
    bx1=0
    bx2=0
    ############################################################
    #to detect red color
            
    param1 = [20,75,250]
    param2 = [70,120,255]
    lower = np.array(param1)    
    upper = np.array(param2)


    mask  = cv2.inRange(img, lower, upper)
    res   = cv2.bitwise_and(img, img, mask= mask)

    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #cv2.drawContours(img,contours,-1,(0,255,0),5)
    #print len(contours)

    for i in range(0,len(contours)):
        a=cv2.contourArea(contours[i])
        if (a>900):
            #print i,"Area = ", cv2.contourArea(contours[i])
            M = cv2.moments(contours[i])
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            print "red = ", cx, ", ", cy
            if (cx < 270):
                storage_red_x = cx/10
                storage_red_y = cy/10
            else:
                service_red_x = cx/10
                service_red_y = cy/10




    ###############################################################
    #to detect blue color

    param1 = [150,70,0]
    param2 = [210,100,40]
    lower = np.array(param1)    
    upper = np.array(param2)


    mask  = cv2.inRange(img, lower, upper)
    res   = cv2.bitwise_and(img, img, mask= mask)

    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #cv2.drawContours(img,contours,-1,(0,255,0),5)
    #print len(contours)

    for i in range(0,len(contours)):
        a=cv2.contourArea(contours[i])
        if (a>900):
            #print i,"Area = ", cv2.contourArea(contours[i])
            M = cv2.moments(contours[i])
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            print "blue = ", cx, ", ", cy
            if (cx < 270):
                storage_blue_x = cx/10
                storage_blue_y = cy/10
            else:
                service_blue_x = cx/10
                service_blue_y = cy/10
                
    ###############################################################
    #to detect yellow color
                
    param1 = [75,200,180]
    param2 = [120,237,230]
    lower = np.array(param1)    
    upper = np.array(param2)


    mask  = cv2.inRange(img, lower, upper)
    res   = cv2.bitwise_and(img, img, mask= mask)

    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #cv2.drawContours(img,contours,-1,(0,255,0),5)
    #print len(contours)

    for i in range(0,len(contours)):
        a=cv2.contourArea(contours[i])
        if (a>900):
            #print i,"Area = ", cv2.contourArea(contours[i])
            M = cv2.moments(contours[i])
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            print "yellow = ", cx, ", ", cy
            if (cx < 270):
                storage_yellow_x = cx/10
                storage_yellow_y = cy/10
            else:
                service_yellow_x = cx/10
                service_yellow_y = cy/10


    ###############################################################
    #to detect green obstacles color

    param1 = [70,70,0]
    param2 = [130,130,8]
    lower = np.array(param1)    
    upper = np.array(param2)

    mask  = cv2.inRange(img, lower, upper)
    res   = cv2.bitwise_and(img, img, mask= mask)

    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #print len(contours)
    b1=0
    b2=0
    for i in range(0,len(contours)):
        a=cv2.contourArea(contours[i])
        if (a>700):
            #print i,"Area = ", cv2.contourArea(contours[i])
            M = cv2.moments(contours[i])
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            print "green = ", cx, ", ", cy
            if (cx>135 and cx<270):
                bx1=bx1+cx
                b1=b1+1
                if (b1 ==1):
                    obstacle_2_x = cx
                    obstacle_2_y = cy
                else:
                    obstacle_1_x = cx
                    obstacle_1_y = cy
            if (cx>270 and cx<405):
                bx2=bx2+cx
                b2=b2+1
                if (b2 ==1):
                    obstacle_4_x = cx
                    obstacle_4_y = cy
                else:
                    obstacle_3_x = cx
                    obstacle_3_y = cy
            cv2.drawContours(img,contours,i,(0,255,0),5)

    bx1=bx1/2-50
    bx1=bx1/10
    bx2=bx2/2+64
    bx2=bx2/10
    bx3=270/10
    #print bx1,bx2,bx3


    m=48
    n=54

    the_map = []
    row = [0] * n
    for i in range(m): # create empty map
        the_map.append(list(row))

    for i in range(m):
        for j in range(n):
            the_map[i][j]=1
        
    for i in range(m):
        the_map[i][bx1]=0

    for i in range(m):
        the_map[i][bx2]=0

    for i in range(m):
        the_map[i][bx3]=0

    for i in range(storage_red_x,bx1):
        the_map[storage_red_y][i]=0

    for i in range(storage_blue_x,bx1):
        the_map[storage_blue_y][i]=0

    for i in range(storage_yellow_x,bx1):
        the_map[storage_yellow_y][i]=0

    for i in range(bx2,service_red_x):
        the_map[service_red_y][i]=0

    for i in range(bx2,service_blue_x):
        the_map[service_blue_y][i]=0

    for i in range(bx2,service_yellow_x):
        the_map[service_yellow_y][i]=0

    upper_edge_1 = obstacle_1_y-60
    lower_edge_1 = obstacle_1_y+60

    upper_edge_2 = obstacle_2_y-60
    lower_edge_2 = obstacle_2_y+60

    if (upper_edge_1 >= 100):
        for i in range(5,(upper_edge_1-50)/10):
            for j in range(bx1,bx3):
                the_map[i][j]=0

    if (upper_edge_2-lower_edge_1 >= 100):
        for i in range((lower_edge_1+50)/10,(upper_edge_2-50)/10):
            for j in range(bx1,bx3):
                the_map[i][j]=0

    if (480-lower_edge_2 >= 100):
        for i in range((lower_edge_2+50)/10,43):
            for j in range(bx1,bx3):
                the_map[i][j]=0


    upper_edge_1 = obstacle_3_y-60
    lower_edge_1 = obstacle_3_y+60

    upper_edge_2 = obstacle_4_y-60
    lower_edge_2 = obstacle_4_y+60

    if (upper_edge_1 >= 100):
        for i in range(5,(upper_edge_1-50)/10):
            for j in range(bx3,bx2):
                the_map[i][j]=0

    if (upper_edge_2-lower_edge_1 >= 100):
        for i in range((lower_edge_1+50)/10,(upper_edge_2-50)/10):
            for j in range(bx3,bx2):
                the_map[i][j]=0

    if (480-lower_edge_2 >= 100):
        for i in range((lower_edge_2+50)/10,43):
            for j in range(bx3,bx2):
                the_map[i][j]=0
    print the_map
    xA = storage_red_x-1
    yA = storage_red_y
    print xA,yA
    xB = service_blue_x-1
    yB = service_blue_y
    print xB,yB
    
    route = pathFind(the_map, n, m, dirs, dx, dy, xA, yA, xB, yB)

    route_path = []
    # mark the route on the map
    if len(route) > 0:
        x = xA
        y = yA
        
        for i in range(len(route)):
            j = int(route[i])
            x += dx[j]
            y += dy[j]
            route_path.append([y+1,x+1])
            


    # display the map with the route added
    route_length = len(route)
        
    return route_length, route_path

class node:
    xPos = 0 # x position
    yPos = 0 # y position
    distance = 0 # total distance already travelled to reach the node
    priority = 0 # priority = distance + remaining distance estimate
    def __init__(self, xPos, yPos, distance, priority):
        self.xPos = xPos
        self.yPos = yPos
        self.distance = distance
        self.priority = priority
    def __lt__(self, other): # comparison method for priority queue
        return self.priority < other.priority
    def updatePriority(self, xDest, yDest):
        self.priority = self.distance + self.estimate(xDest, yDest) * 10 # A*
    # give higher priority to going straight instead of diagonally
    def nextMove(self, dirs, d): # d: direction to move
        self.distance += 10
    # Estimation function for the remaining distance to the goal.
    def estimate(self, xDest, yDest):
        xd = xDest - self.xPos
        yd = yDest - self.yPos
        # Euclidian Distance
        d = math.sqrt(xd * xd + yd * yd)
        return(d)

# A-star algorithm.
def pathFind(the_map, n, m, dirs, dx, dy, xA, yA, xB, yB):
    closed_nodes_map = [] # map of closed (tried-out) nodes
    open_nodes_map = [] # map of open (not-yet-tried) nodes
    dir_map = [] # map of dirs
    row = [0] * n
    for i in range(m): # create 2d arrays
        closed_nodes_map.append(list(row))
        open_nodes_map.append(list(row))
        dir_map.append(list(row))

    pq = [[], []] # priority queues of open (not-yet-tried) nodes
    pqi = 0 # priority queue index
    # create the start node and push into list of open nodes
    n0 = node(xA, yA, 0, 0)
    n0.updatePriority(xB, yB)
    heappush(pq[pqi], n0)
    open_nodes_map[yA][xA] = n0.priority # mark it on the open nodes map

    # A* search
    while len(pq[pqi]) > 0:
        # get the current node w/ the highest priority
        # from the list of open nodes
        n1 = pq[pqi][0] # top node
        n0 = node(n1.xPos, n1.yPos, n1.distance, n1.priority)
        x = n0.xPos
        y = n0.yPos
        heappop(pq[pqi]) # remove the node from the open list
        open_nodes_map[y][x] = 0
        closed_nodes_map[y][x] = 1 # mark it on the closed nodes map

        # quit searching when the goal is reached
        # if n0.estimate(xB, yB) == 0:
        if x == xB and y == yB:
            # generate the path from finish to start
            # by following the dirs
            path = ''
            while not (x == xA and y == yA):
                j = dir_map[y][x]
                c = str((j + dirs / 2) % dirs)
                path = c + path
                x += dx[j]
                y += dy[j]
            return path

        # generate moves (child nodes) in all possible dirs
        for i in range(dirs):
            xdx = x + dx[i]
            ydy = y + dy[i]
            if not (xdx < 0 or xdx > n-1 or ydy < 0 or ydy > m - 1
                    or the_map[ydy][xdx] == 1 or closed_nodes_map[ydy][xdx] == 1):
                # generate a child node
                m0 = node(xdx, ydy, n0.distance, n0.priority)
                m0.nextMove(dirs, i)
                m0.updatePriority(xB, yB)
                # if it is not in the open list then add into that
                if open_nodes_map[ydy][xdx] == 0:
                    open_nodes_map[ydy][xdx] = m0.priority
                    heappush(pq[pqi], m0)
                    # mark its parent node direction
                    dir_map[ydy][xdx] = (i + dirs / 2) % dirs
                elif open_nodes_map[ydy][xdx] > m0.priority:
                    # update the priority
                    open_nodes_map[ydy][xdx] = m0.priority
                    # update the parent direction
                    dir_map[ydy][xdx] = (i + dirs / 2) % dirs
                    # replace the node
                    # by emptying one pq to the other one
                    # except the node to be replaced will be ignored
                    # and the new node will be pushed in instead
                    while not (pq[pqi][0].xPos == xdx and pq[pqi][0].yPos == ydy):
                        heappush(pq[1 - pqi], pq[pqi][0])
                        heappop(pq[pqi])
                    heappop(pq[pqi]) # remove the target node
                    # empty the larger size priority queue to the smaller one
                    if len(pq[pqi]) > len(pq[1 - pqi]):
                        pqi = 1 - pqi
                    while len(pq[pqi]) > 0:
                        heappush(pq[1-pqi], pq[pqi][0])
                        heappop(pq[pqi])       
                    pqi = 1 - pqi
                    heappush(pq[pqi], m0) # add the better node instead
    return '' # if no route found


if __name__ == "__main__":
    #code for checking output for single image
    img = cv2.imread('1.jpg')
    route_length, route_path = play(img)
    print "route length = ", route_length
    print "route path   = ", route_path
    cv2.imshow('image',img)
    #code for checking output for all images
'''   route_length_list = []
   route_path_list   = []
   for file_number in range(1,6):
       file_name = "test_images/test_image"+str(file_number)+".png"
       pic = cv2.imread(file_name)
       route_length, route_path = play(pic)
       route_length_list.append(route_length)
       route_path_list.append(route_path)

   print route_length_list
   print route_path_list'''
cv2.waitKey(0)     
cv2.destroyAllWindows()
