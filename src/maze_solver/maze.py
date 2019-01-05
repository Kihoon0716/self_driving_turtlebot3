#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import *
from compressed_image_transport import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from  mazeSolver import Solver
import tf
import math
import cv2
import numpy as np
import threading
from std_msgs.msg import String
import heapq
import timeit

def existance(arr, num):
    for i in range(0, len(arr)):
        if arr[i] == num:
            return True
    return False

def configure(arr):
    arr_ = []
    for i in range(0, len(arr)):
        if existance(arr_, arr[i]) == False:
            arr_.append(arr[i])
    return arr_

def distance_dot2line(a, b, c, x0, y0):
    distance = abs(x0*a + y0*b + c)/math.sqrt(a*a + b*b)
    return distance

def distance_dot2dot(x1, y1, x2, y2):
    distance = math.sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2-y1))
    return distance


def theta_dot2dot(start, end):
    theta = math.atan2(end[1]-start[1], end[0]-start[0])
    return theta

def euler_from_quaternion(rot):
    quaternion = (rot)
    theta = tf.transformations.euler_from_quaternion(quaternion)[2] - np.pi / 2
    return theta

def sign(num):
    if num < 0:
        return -1
    else:
        return 1



class Orientation(object):
    def __init__(self, trans, rot):
        self.x = trans[0]
        self.y = trans[1]
        self.theta = euler_from_quaternion(rot)


        


class Maze_pathfinder():
    def __init__(self):

        self._sub = rospy.Subscriber('/map', OccupancyGrid, self.callback, queue_size=1)
        self._sub = rospy.Subscriber('/odom', Odometry, self.callback2, queue_size=1)

        self._sub = rospy.Subscriber('/clicked_point', PointStamped, self.callback4, queue_size=1)

        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._pub2 = rospy.Publisher('/maze', String, queue_size=1)
        
        self._pub_marker_path = rospy.Publisher('/marker_path', Marker, queue_size=1)
        self._pub_marker_start = rospy.Publisher('/marker_start', Marker, queue_size=1)
        self._pub_marker_moving = rospy.Publisher('/marker_moving', Marker, queue_size=1)
        self.state = 'stop' # path_finding, stop, going, direction_setting

        # variables used in maze solve


        self.theta = 0


        self.goal = None
        self.path = None

    def callback(self, map_data):
        time0 = timeit.default_timer()
        if self.goal == None:
            return
        map = np.zeros((384, 384)) #empty space
        
        # create map
        for i in range(0, 384):
            for j in range(0, 384):
                # if map_data.data[384*j + i] != 100 and map_data.data[384*j + i] != -1 and map_data.data[384*j + i] != 0:
                #     print "find other value! : ", map_data.data[384*j + i]
                if map_data.data[384*j + i] == 100: # there is a obstacle
                    #map[i][j] = 1
                    
                    for k in range(-6, 7):
                        for h in range(-6, 7):
                            if i-k >= 384 or i-k < 0 or j-h >=384 or j-h <0:
                                continue
                            map[i-k][j-h] = 1       
                else:
                    map[i][j] = 0
                    
        
 
        col_start = int(200 + self.position_now[1] * 20)
        row_start = int(196 + self.position_now[0] * 20)
        col_goal = int(200 + self.goal.y * 20) 
        row_goal = int(196 + self.goal.x *20)

        # print "start : ", map[row_start][col_start]

        for i in range (5):
            if map[row_start][col_start] == 0:
                    break
            for j in range(-i, i+1):
                if map[row_start][col_start] == 0:
                    break
                elif map[row_start + i][col_start + j] == 0:
                    row_start += i
                    col_start += j
            for j in range(-i, i+1):
                if map[row_start][col_start] == 0:
                    break
                elif map[row_start - i][col_start + j] == 0:
                    row_start -= i
                    col_start += j
            for j in range(-i, i+1):
                if map[row_start][col_start] == 0:
                    break
                elif map[row_start + j][col_start + i] == 0:
                    row_start += j
                    col_start += i
            for j in range(-i, i+1):
                if map[row_start][col_start] == 0:
                    break
                elif map[row_start + j][col_start - i] == 0:
                    row_start += j
                    col_start -= i    
            

        
        solver = Solver(map, 384, 384, int(row_start), int(col_start), int(row_goal), int(col_goal))
        solver.solve()
        self.path = solver.getPath()

        #publish marker
        marker_path = Marker()
        marker_path.header.frame_id = "map"
        marker_path.header.stamp = rospy.Time()
        marker_path.ns = "path"
        marker_path.id = 0
        marker_path.type = 4
        marker_path.action = Marker.ADD

        marker_path.pose.orientation.x = 0.0
        marker_path.pose.orientation.y = 0.0
        marker_path.pose.orientation.z = 0.0
        marker_path.pose.orientation.w = 0.0
        marker_path.scale.x = 0.05
        marker_path.scale.y = 0.05
        marker_path.scale.z = 0.05
        marker_path.color.a = 1.0
        marker_path.color.r = 0.0
        marker_path.color.g = 1.0
        marker_path.color.b = 0.0

        # appending points

        # print "len : ", len(self.path)
        for i in range(1, len(self.path)):
            row = (self.path[i][0] - 196) / 20 
            col = (self.path[i][1] - 200) / 20 
            self.addMarkerLine(marker_path, row, col)

        self._pub_marker_path.publish(marker_path)
        
        time1 = timeit.default_timer()
        # print "time 1 : ", time1 - time0


    def callback2(self, odometry):
        ## converting odometry to x position, y position and theta

        self.theta = euler_from_quaternion([odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w])
        self.position_now = [odometry.pose.pose.position.x, odometry.pose.pose.position.y]
        if self.theta < 0:
            self.theta = self.theta + np.pi*2
        
        # draw marker of start point
        marker_start_point = Marker()
        marker_start_point.header.frame_id = "map"
        marker_start_point.header.stamp = rospy.Time()
        marker_start_point.ns = "start point"
        marker_start_point.id = 0
        marker_start_point.type = 2
        marker_start_point.action = Marker.ADD
        marker_start_point.pose.position.x = self.position_now[0]
        marker_start_point.pose.position.y = self.position_now[1]
        marker_start_point.pose.orientation.x = 0
        marker_start_point.pose.orientation.y = 0
        marker_start_point.pose.orientation.z = 0
        marker_start_point.pose.orientation.w = 0
        marker_start_point.scale.x = 0.1
        marker_start_point.scale.y = 0.1
        marker_start_point.scale.z = 0.1
        marker_start_point.color.a = 1.0
        marker_start_point.color.r = 1.0
        marker_start_point.color.g = 0.0
        marker_start_point.color.b = 0.0
        self._pub_marker_start.publish(marker_start_point)

        #print self.position_now
        #print self.theta
        
        # stop when goal is None
        if self.goal == None:
            self.move(0, 0)
            return
        else:    
            move_pose_x = 0
            move_pose_y = 0
            try:
                if len(self.path) > 10:
                    move_pose_x = (self.path[len(self.path) - 10][0] - 196) / 20
                    move_pose_y = (self.path[len(self.path) - 10][1] - 200) / 20
                else:
                    move_pose_x = (self.path[1][0] - 196) / 20
                    move_pose_y = (self.path[1][1] - 200) / 20
            except:
                return

            # draw marker of moving point
            marker_moving_point = Marker()
            marker_moving_point.header.frame_id = "map"
            marker_moving_point.header.stamp = rospy.Time()
            marker_moving_point.ns = "start point"
            marker_moving_point.id = 0
            marker_moving_point.type = 2
            marker_moving_point.action = Marker.ADD
            marker_moving_point.pose.position.x = move_pose_x
            marker_moving_point.pose.position.y = move_pose_y
            marker_moving_point.pose.orientation.x = 0
            marker_moving_point.pose.orientation.y = 0
            marker_moving_point.pose.orientation.z = 0
            marker_moving_point.pose.orientation.w = 0
            marker_moving_point.scale.x = 0.05
            marker_moving_point.scale.y = 0.05
            marker_moving_point.scale.z = 0.05
            marker_moving_point.color.a = 1.0
            marker_moving_point.color.r = 0.0
            marker_moving_point.color.g = 0.0
            marker_moving_point.color.b = 1.0
            self._pub_marker_moving.publish(marker_moving_point)


            # print "move pose x : ", move_pose_x, " y : ", move_pose_y
            desired_theta = math.atan2(move_pose_y - self.position_now[1], move_pose_x - self.position_now[0])
            desired_theta -= np.pi/2
            if desired_theta < 0:
                desired_theta += np.pi * 2
            #print desired_theta
            diff = desired_theta - self.theta
            if diff < np.pi:
                diff += np.pi*2
            if diff > np.pi:
                diff -= np.pi*2
            #print "diff : ", diff
            if diff > np.pi * 1/3:
                speed = 0
            else:
                speed = 0.3 * abs(np.pi - diff) /np.pi/3
            
            self.move(diff, speed)
            
        
        
        if distance_dot2dot(self.goal.x, self.goal.y, self.position_now[0], self.position_now[1]) < 0.1:
            self.goal = None


    def callback4(self, clicked_point):
        # print "self x", self.position_now[0]
        # print "self y", self.position_now[1]
        # print "self theta", self.theta
        # print "dist x", clicked_point.point.x
        # print "dist y", clicked_point.point.y

        self.goal = clicked_point.point

        desired_theta = math.atan2(clicked_point.point.y - self.position_now[1], clicked_point.point.x - self.position_now[0])
        desired_theta -= np.pi/2
        if desired_theta < 0:
            desired_theta += np.pi * 2
        #print desired_theta
        diff = desired_theta - self.theta
        if diff < np.pi:
            diff += np.pi*2
        if diff > np.pi:
            diff -= np.pi*2
        #print "diff : ", diff




        #publish marker

        # marker = Marker()
        # marker.header.frame_id = "base_link"
        # marker.header.stamp = rospy.Time()
        # marker.ns = "my_namespace"
        # marker.id = 0
        # marker.type = 4
        # marker.action = Marker.ADD

        # marker.pose.orientation.x = 0.0
        # marker.pose.orientation.y = 0.0
        # marker.pose.orientation.z = 0.0
        # marker.pose.orientation.w = 0.0
        # marker.scale.x = 0.05
        # marker.scale.y = 0.05
        # marker.scale.z = 0.05
        # marker.color.a = 1.0
        # marker.color.r = 0.0
        # marker.color.g = 1.0
        # marker.color.b = 0.0

        # # appending points
        # self.addMarkerLine(marker, 0,0)
        # self.addMarkerLine(marker, 1,1)
        # self.addMarkerLine(marker, 2,1)
        # self.addMarkerLine(marker, 0,7)

        # self._pub_marker.publish(marker)



    def publishing_vel(self, angular_x, angular_y, angular_z, linear_x, linear_y, linear_z):
        vel = Twist()
        vel.angular.x = angular_x
        vel.angular.y = angular_y
        vel.angular.z = angular_z
        vel.linear.x = linear_x
        vel.linear.y = linear_y
        vel.linear.z = linear_z
        self._pub.publish(vel)

    def move(self, speed, direction):
        self.publishing_vel(0, 0, speed, direction, 0, 0)


    def addMarkerLine(self, marker, p_x, p_y):
        p = Point()
        p.x = p_x
        p.y = p_y
        marker.points.append(p)
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('maze_pathfinder')
    mazesolver = Maze_pathfinder()
    mazesolver.main()
