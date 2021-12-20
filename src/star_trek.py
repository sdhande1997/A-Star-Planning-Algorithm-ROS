#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import numpy
from tf.transformations import euler_from_quaternion

check_fuel = False
car_pose = None
star_map = None
goal_found = True
all_Ok = True
pose_x = None
pose_y = None
yaw_thresh = math.pi / 30
norm_cost = 1

goal_x = rospy.get_param('goal_x')
goal_y = rospy.get_param('goal_y')
initial_x = -8.0
initial_y = -2.0
speed = Twist()
speed_forward = speed.linear
list_num = 0
eins = math.pi
eins_2 = eins * 2
ein_2 = 0

class Node:
    def __init__(self, parent = None, spot = None):
        self.parent = parent
        self.spot = spot

    def __eq__(self,other):
        return self.spot == other.spot
    
class cost(Node):
    def __init__(self, parent, spot):
        Node.__init__(self, parent, spot)
        self.f = 0
        self.g = 0
class heu(Node):
    def heuristic(self, start, goal):
        return (pow(start[0] - goal[0], 2) + (pow(start[1] - goal[1], 2)))
    def euclidean(self, x2, x1, y2, y1):
        return math.sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2))
    
def init_main():
    global car_velocity, goal_found, check_fuel, initial_x, initial_y, list_num, on_route
    # if not check_fuel:
    #     fuel_check()
    # else:
    get_map()
    on_route = False
    rospy.init_node("star_trek", anonymous = True)
    rospy.Subscriber("base_pose_ground_truth", Odometry, fuel_check)
    car_velocity = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
    rospy.spin()
    
def neighbours(c_node):
    global star_map
    child_list = []
    child_nodes = [(0,-1),    #left
                   (0,1),     #right
                   (-1,0),    #up
                   (1,0),     #down
                   (-1,-1),   #upper left
                   (-1,1),    #upper right
                   (1,-1),    #down left
                   (1,1)]     #down right
    for child in (child_nodes):
        #Get the new node position
        child_pose = (c_node.spot[0] + child[0], c_node.spot[1] + child[1])
        
        #See if its in the valid range
        if child_pose[0] > (len(star_map) - 1) or child_pose[0] < 0 or child_pose[1] > (len(star_map) - 1) or child_pose[1] < 0:
            continue
        
        #See if the current path is exploreable
        if star_map[child_pose[0]][child_pose[1]] != 0:
            continue
        child_node = cost(None, child_pose)
        normal_cost = c_node.g + norm_cost
        child_node.g = normal_cost     
        child_node.parent = c_node
        child_list.append(child_node)
    return child_list

class AStarPlanning():

    def get_path(self, pune_pose, mumbai_pose):
        open_list = []
        close_list = []
        child_lists = []
        
        start_node = cost(None, pune_pose)
        goal_node = cost(None, mumbai_pose)
        
        start_node.g = 0
        goal_node.g = 0
        initial_cost = start_node.g + heu().heuristic(pune_pose, mumbai_pose)
        initial_goal_cost =  goal_node.g + heu().heuristic(pune_pose, mumbai_pose)
        start_node.f = initial_cost
        goal_node.f = initial_goal_cost
        open_list.append(start_node)
        
        path = []
        while len(open_list) > 0:
            initial_node = open_list[0]
            initial_list_idx = 0
            
            for idx, i in enumerate(open_list):    #get the index of element from open_list with lowest f_score
                if i.f < initial_node.f:
                    initial_node = i
                    initial_list_idx = idx
                    # print("initial_list_idx", initial_list_idx)
            open_list.pop(initial_list_idx)      #.pop removes and returns the first value from the list
                    
            if initial_node == goal_node:
                path_node = initial_node
                while path_node.parent is not None:
                    path.append(path_node.spot)
                    path_node = path_node.parent
                # print("Open_list length", len(open_list))
                return path[::-1]
            
            close_list.append(initial_node)
            
            #Generate the neighbours/Childrens
            
            children_nodes = neighbours(initial_node)
            # print("children_nodes", len(children_nodes))
            
            for children in children_nodes:
                if children not in close_list:
                    low_cost = children.g + heu().heuristic(children.spot, mumbai_pose)
                    children.f = low_cost
                    
                    if children not in open_list:
                        open_list = child_lists
                        child_lists.append(children)
                    else:
                        open_node = child_lists[child_lists.index(children)]
                        if children.g < open_node.g:
                            open_node.g = children.g
                            open_node.parent = children.parent
        return False

def get_coordinates(column, row):
    global list_num, find_path
    if len(find_path) <= 12:
        if list_num < 7:
            x = column - 9 + 0.5
            y = 10 - row - 0.5
            return x, y
        if 16 > list_num > 7 :  
            x = column - 9 + 0.35
            y = 10 - row - 0.35
            return x, y
        else:
            x = column - 9 + 0.50
            y = 10 - row + 0.25
            return x, y
    elif len(find_path) > 12:
        if list_num < 7:
            x = column - 9 + 1.0
            y = 10 - row - 1.0
            return x, y
        if 16 > list_num > 7 :  
            x = column - 9 + 0.35
            y = 10 - row - 0.35
            return x, y
        else:
            x = column - 9 + 0.50
            y = 10 - row + 0.25
            return x, y
        
def car():
    global find_path, pose_x, pose_y, new_node, list_num, car_velocity, check_fuel, goal_found, on_route, yaw
    
    if not on_route:
        # print("START List_Num", list_num)
        # print("find path ======",len(find_path))
        find_path_x = find_path[list_num][1]
        find_path_y = find_path[list_num][0]
        new_node = get_coordinates(find_path_x, find_path_y)

        new_node_yaw = math.atan2(new_node[1] - pose_y, new_node[0] - pose_x)
        difference = new_node_yaw - yaw

        if goal_found:
            if math.fabs(difference) > 3.14159:
                difference = difference - (2 * math.pi * difference)/(math.fabs(difference))
            yaw_difference = difference
            if math.fabs(yaw_difference) > yaw_thresh:
                if yaw_difference < 0:
                    yaw_difference += eins_2
                elif yaw_difference < ein_2:
                    ein_2 == yaw_difference
                if yaw_difference > eins:
                    speed_forward.x = 0.3
                    speed.angular.z = -1.0
                    car_velocity.publish(speed) 
                else:
                    speed_forward.x = 0.3
                    speed.angular.z = 1.0
                    car_velocity.publish(speed)
            else:
                goal_found = False  
        else:
            difference = heu().euclidean(new_node[0], pose_x, new_node[1], pose_y)
            if difference > 0.5:
                speed.angular.z = 0
                speed_forward.x = 4
                car_velocity.publish(speed)
            elif difference < ein_2:
                ein_2 == difference
            else:
                goal_found = True
                if list_num +1 < len(find_path):
                        list_num = list_num + 1
                        if ein_2 > 0:
                            ein_2 == list_num
                        # print("LIST NUM ADD== ", list_num)
    if len(find_path) -1 == list_num:
        goal_found == True
        # print("***GOAL FOUND***")

map = [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,1,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,1,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
       0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
       0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]

def get_map():
    global star_map, pune_pose, mumbai_pose, find_path, initial_x, initial_y, map
    star_map = numpy.reshape(map, (20, 18))
    pune_pose = (10 - int(math.floor(initial_y)), 9 + int(math.floor(initial_x)))
    mumbai_pose = (10 - int(math.floor(goal_y)),  9 + int(math.floor(goal_x)))
    
    find_path = AStarPlanning().get_path(pune_pose, mumbai_pose)
    # print("Find Path",find_path)

def fuel_check(car_data):
    global car_pose, pose_x, pose_y, check_fuel, yaw
    car_pose = car_data.pose.pose
    pose_x = car_pose.position.x
    pose_y = car_pose.position.y
    orient_car = (car_pose.orientation.x, car_pose.orientation.y, car_pose.orientation.z, car_pose.orientation.w)
    math_euler = euler_from_quaternion(orient_car)
    yaw = math_euler[2]
    car()
    
if __name__ == "__main__":
    try:
        init_main()
    except rospy.ROSInterruptException:
        pass