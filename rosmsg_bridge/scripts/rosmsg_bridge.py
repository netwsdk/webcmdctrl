#!/usr/bin/env python
#! -*- coding: utf-8 -*-

import rospy
import threading
import json
import math
import random
import time
import base64
import socket
import os
import signal
import sys
import tf
import geometry_msgs.msg

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionResult
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CompressedImage
from visualization_msgs.msg import MarkerArray

# socket message provider ID string, unique
WEBCMD_CLIENT_MSG_CLASS = "webcmd_client"

# socket message provider ID string, unique
ROS_MSG_BRIDGE_CLASS = "ros_msg_bridge"

# this is webcmd server msg class string
WEBCMD_SERVER_MSG_CLASS = "ros_webcmd_server"

BINARY_DATA_VERSION = 0
BINARY_DATA_TYPE_IMAGE = 1

# head has 8 bytes, 4 magic number and 4 bytes length
FRAME_HEADER_LEN = 8

MAX_LEN_PER_FRAME = (8*1024*1024 - 1000)

# some data of ros is not continuous stream. so we save the data, and send them to clients in a loop.
ROS_DATA_FRAME_RATE = 10
# So we need define the cycle(unit: second) for each kinds of data sent to clients
TF_SENT_CYCLE_TIME = 0.1
LASER_SENT_CYCLE_TIME = 0.5

# TF listener timeout
TF_LISTEN_TIMEOUT = 30

# max steering angle: 30 degree
max_steering_angle = 0.5

# 由于人手抖动，控制精度不可能保证直行时角度正好是0，需要设定一个抖动的角度范围，这个抖动角度范围内都是认为角度为0
# 超过这个抖动角度后，才开始计算转向角度，而且还要减去这个抖动的角度数值
straight_delta_angle = 0.18

# lidar has a max distance limited, so ranges data will have value named "Infinity", we need replace this into the max distance value
LASER_Infinity_value = "0.0"  # 10m

isExitSignal = False
isConnected = False

def goal_result_callback(result):
    global webcmd
    webcmd.goal_result_inform(result)

def laser_scan_callback(laser):
    global webcmd
    webcmd.laser_scan_inform(laser)

def global_navpath_callback(path):
    global webcmd
    webcmd.global_navpath_inform(path)

def local_navpath_callback(path):
    global webcmd
    webcmd.local_navpath_inform(path)

def map_callback(map):
    global webcmd
    webcmd.map_inform(map)

def camera_callback(image):
    global webcmd
    webcmd.camera_inform(image)

def explorer_frontiers_callback(frontiers):
    global webcmd
    # if we get this message, that means explorer is working, not completed. currently the data of frontiers is not used!
    # we just send an informing message to webcmd client
    webcmd.explorer_inform(frontiers)

class webcmd_forward:
    def __init__(self):
        self.has_cmd_sent = False
        self.last_cmd_time = 0.0

        self.last_tf_sent_time = rospy.get_time()
        self.last_laser_sent_time = rospy.get_time()

        self.goal_status = None
        self.goal_update = False # True: has goal result msg, need send to client

        self.laser_scan_data = None
        self.laser_scan_update = False

        self.global_navpath_data = None
        self.global_navpath_update = False

        self.local_navpath_data = None
        self.local_navpath_update = False

        self.map_data = None
        self.map_update = False
        self.map_cache_available = False
        self.map_cache = ""

        self.camera_data = None
        self.camera_update = False

        self.explorer_frontier_data = None
        self.explorer_frontier_update = False

        # if True, will use topic /ackermann_cmd to publish the control command of car
        self.ackermann_mode = bool(rospy.get_param('~ackermann_mode',False))

        # setup robot car joystick control cmd topic publisher or subscriber
        if self.ackermann_mode :
            self.pub_cmd = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=5)
        else:
            self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # setup topic of robot car init pose setting
        self.pub_setinitpose = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
        # setup topic of robot car goal setting
        self.pub_setgoalpose = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.cur_goal_id = ''
        # cancel goal, if goal id is null, cancel all goal task! used by stopping explorer map service to stop robot
        self.pub_cancelgoal = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)
        
        self.tfListener = tf.TransformListener()
        self.tfTransDict = {} # dict tfTransDict[base_frame_id][frame_id] is the result of tf
        self.tfTransDict_lock = threading.Lock()
        
        # 20Hz because we want to support camera image frame rate up to 20 frames / seconds (if camera framerate is > 20, then will drop some of data)
        self.rate = rospy.Rate(ROS_DATA_FRAME_RATE)

    def startSubscribers(self):
        # navigation result
        self.goal_result_sub = rospy.Subscriber("move_base/result", MoveBaseActionResult, goal_result_callback)
        # laser scan points
        self.laser_scan_sub = rospy.Subscriber("scan", LaserScan, laser_scan_callback)
        # global path of navigation (AMCL)
        self.global_path_sub = rospy.Subscriber("move_base/GlobalPlanner/plan", Path, global_navpath_callback)
        # local path of navigation (TEB)
        self.local_path_sub = rospy.Subscriber("move_base/TebLocalPlannerROS/local_plan", Path, local_navpath_callback)
        # world map
        self.map_sub = rospy.Subscriber("map", OccupancyGrid, map_callback)
        # camera: /image/compressed
        self.camera_sub = rospy.Subscriber("image_raw/compressed", CompressedImage, camera_callback)
        # explorer map
        self.explorer_sub = rospy.Subscriber("/explore/frontiers", MarkerArray, explorer_frontiers_callback)

    def restartAllTopics(self):
        # setup robot car joystick control cmd topic publisher or subscriber
        if self.ackermann_mode :
            self.pub_cmd.unregister()
        else:
            self.pub_cmd.unregister()

        self.pub_setinitpose.unregister()
        # setup topic of robot car goal setting
        self.pub_setgoalpose.unregister()
        self.pub_cancelgoal.unregister()
        # navigation result
        self.goal_result_sub.unregister()
        # laser scan points
        self.laser_scan_sub.unregister()
        # global path of navigation (AMCL)
        self.global_path_sub.unregister()
        # local path of navigation (TEB)
        self.local_path_sub.unregister()
        # world map
        self.map_sub.unregister()
        # camera: /image/compressed
        self.camera_sub.unregister()
        self.explorer_sub.unregister()

        # setup robot car joystick control cmd topic publisher or subscriber
        if self.ackermann_mode :
            self.pub_cmd = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=5)
        else:
            self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        # setup initialpose setting topic publisher
        self.pub_setinitpose = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
        # setup topic of robot car goal setting
        self.pub_setgoalpose = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.pub_cancelgoal = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)

        # navigation result
        self.goal_result_sub = rospy.Subscriber("move_base/result", MoveBaseActionResult, goal_result_callback)
        # laser scan points
        self.laser_scan_sub = rospy.Subscriber("scan", LaserScan, laser_scan_callback)
        # global path of navigation (AMCL)
        self.global_path_sub = rospy.Subscriber("move_base/GlobalPlanner/plan", Path, global_navpath_callback)
        # local path of navigation (TEB)
        self.local_path_sub = rospy.Subscriber("move_base/TebLocalPlannerROS/local_plan", Path, local_navpath_callback)
        # world map
        self.map_sub = rospy.Subscriber("map", OccupancyGrid, map_callback)
        # camera: /image/compressed
        self.camera_sub = rospy.Subscriber("image_raw/compressed", CompressedImage, camera_callback)
        self.explorer_sub = rospy.Subscriber("/explore/frontiers", MarkerArray, explorer_frontiers_callback)

    def send_car_cmd(self, speed, angle):
        if self.ackermann_mode :
            self.ackermannCmd = AckermannDriveStamped()
            self.ackermannCmd.header.stamp = rospy.Time.now()
            self.ackermannCmd.header.frame_id = 'webcmd_ackermann_frame'

            self.ackermannCmd.drive.speed = speed
            self.ackermannCmd.drive.steering_angle = angle

            self.pub_cmd.publish(self.ackermannCmd)
        else:
            # 这个车的处于普通控制模式（全向轮）时，注意：当倒车时，转向角的正负号 和 前进时相反了
            if speed < 0 :
                angle = -1 * angle

            self.twist = Twist()
            self.twist.linear.x = 0
            self.twist.linear.y = 0
            self.twist.linear.z = 0
            self.twist.angular.x = 0
            self.twist.angular.y = 0
            self.twist.angular.z = 0

            self.twist.linear.x = speed
            self.twist.angular.z = angle

            self.pub_cmd.publish(self.twist)
        
        self.last_cmd_time = rospy.get_time()
        self.has_cmd_sent = False
    
    def set_init_pose(self, pose):
        self.initPoseCovar = PoseWithCovarianceStamped()
        self.initPoseCovar.header.stamp = rospy.Time.now()
        self.initPoseCovar.header.frame_id = "map"

        self.initPoseCovar.pose.pose.position.x=pose['pos']['x']
        self.initPoseCovar.pose.pose.position.y=pose['pos']['y']
        self.initPoseCovar.pose.pose.position.z=pose['pos']['z']

        self.initPoseCovar.pose.pose.orientation.x=pose['orientation']['x']
        self.initPoseCovar.pose.pose.orientation.y=pose['orientation']['y']
        self.initPoseCovar.pose.pose.orientation.z=pose['orientation']['z']
        self.initPoseCovar.pose.pose.orientation.w=pose['orientation']['w']

        # self.initPoseCovar.pose.covariance=[0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        self.initPoseCovar.pose.covariance[6*0+0] = 0.5 * 0.5
        self.initPoseCovar.pose.covariance[6*1+1] = 0.5 * 0.5
        self.initPoseCovar.pose.covariance[6*3+3] = math.pi/12.0 * math.pi/12.0

        self.pub_setinitpose.publish(self.initPoseCovar)
    
    def set_goal_pose(self, pose):
        self.goalPose = MoveBaseActionGoal()
        self.goalPose.goal.target_pose.header.stamp = self.goalPose.header.stamp = rospy.Time.now()
        self.goalPose.goal.target_pose.header.frame_id = self.goalPose.header.frame_id = "map"

        self.cur_goal_id = "goal_" + str(random.randint(10000000, 99999999))
        self.goalPose.goal_id.id = self.cur_goal_id

        self.goalPose.goal.target_pose.pose.position.x=pose['pos']['x']
        self.goalPose.goal.target_pose.pose.position.y=pose['pos']['y']
        self.goalPose.goal.target_pose.pose.position.z=pose['pos']['z']

        self.goalPose.goal.target_pose.pose.orientation.x=pose['orientation']['x']
        self.goalPose.goal.target_pose.pose.orientation.y=pose['orientation']['y']
        self.goalPose.goal.target_pose.pose.orientation.z=pose['orientation']['z']
        self.goalPose.goal.target_pose.pose.orientation.w=pose['orientation']['w']

        self.pub_setgoalpose.publish(self.goalPose)

    # currently, we don't set goal id, just cancel all goal tasks!
    def cancel_goal(self, goal_id="") :
        self.cancel_goal_id = GoalID()
        self.cancel_goal_id.stamp = rospy.Time.now()
        self.cancel_goal_id.id = goal_id
        self.pub_cancelgoal.publish(self.cancel_goal_id)

    def goal_result_inform(self, result):
        self.goal_status = result
        self.goal_update = True
    
    def send_goal_result_toClients(self, cur_time, client_socket):
        if self.goal_update :
            resData = {}
            resData['ros_web_cmd'] = "goal_result"
            resData['goal_id'] = self.goal_status.status.goal_id.id
            resData['status'] = self.goal_status.status.status
            resData['desc'] = self.goal_status.status.text

            send_webcmd_msg(client_socket, resData)
            
            self.goal_update = False
    
    def laser_scan_inform(self, laser):
        self.laser_scan_data = laser
        self.laser_scan_update = True

    def send_laser_scan_toClients(self, cur_time, client_socket):
        if cur_time - self.last_laser_sent_time < LASER_SENT_CYCLE_TIME:
            return
        
        if self.laser_scan_update :
            resData = {}
            resData['ros_web_cmd'] = "laser_scan"
            resData['frame_id'] = self.laser_scan_data.header.frame_id
            resData['angle_min'] = self.laser_scan_data.angle_min
            resData['angle_max'] = self.laser_scan_data.angle_max
            resData['angle_increment'] = self.laser_scan_data.angle_increment
            resData['time_increment'] = self.laser_scan_data.time_increment
            resData['scan_time'] = self.laser_scan_data.scan_time
            resData['range_min'] = self.laser_scan_data.range_min
            resData['range_max'] = self.laser_scan_data.range_max
            resData['ranges'] = self.laser_scan_data.ranges
            resData['intensities'] = self.laser_scan_data.intensities

            send_webcmd_msg(client_socket, resData, laser_replace_Infinity)
            
            self.laser_scan_update = False
            self.last_laser_sent_time = cur_time
    
    def global_navpath_inform(self, path):
        self.global_navpath_data = path
        self.global_navpath_update = True

    def send_global_navpath_toClients(self, cur_time, client_socket):
        if self.global_navpath_update :
            resData = {}
            resData['ros_web_cmd'] = "global_navpath"
            resData['frame_id'] = self.global_navpath_data.header.frame_id
            resData['poses'] = []
            for pose in self.global_navpath_data.poses:
                poseData = {}
                poseData['position'] = {}
                poseData['position']['x'] = pose.pose.position.x
                poseData['position']['y'] = pose.pose.position.y
                poseData['position']['z'] = pose.pose.position.z

                poseData['orientation'] = {}
                poseData['orientation']['x'] = pose.pose.orientation.x
                poseData['orientation']['y'] = pose.pose.orientation.y
                poseData['orientation']['z'] = pose.pose.orientation.z
                poseData['orientation']['w'] = pose.pose.orientation.w

                resData['poses'].append(poseData)

            send_webcmd_msg(client_socket, resData)
            
            self.global_navpath_update = False
    
    def local_navpath_inform(self, path):
        self.local_navpath_data = path
        self.local_navpath_update = True

    def send_local_navpath_toClients(self, cur_time, client_socket):
        if self.local_navpath_update :
            resData = {}
            resData['ros_web_cmd'] = "local_navpath"
            resData['frame_id'] = self.local_navpath_data.header.frame_id
            resData['poses'] = []
            for pose in self.local_navpath_data.poses:
                poseData = {}
                poseData['position'] = {}
                poseData['position']['x'] = pose.pose.position.x
                poseData['position']['y'] = pose.pose.position.y
                poseData['position']['z'] = pose.pose.position.z

                poseData['orientation'] = {}
                poseData['orientation']['x'] = pose.pose.orientation.x
                poseData['orientation']['y'] = pose.pose.orientation.y
                poseData['orientation']['z'] = pose.pose.orientation.z
                poseData['orientation']['w'] = pose.pose.orientation.w

                resData['poses'].append(poseData)

            send_webcmd_msg(client_socket, resData)
            self.local_navpath_update = False

    def force_send_map_cache(self):
        # force to set map_update to true if map cache is avalible, or just wait for map to be updated
        if self.map_cache_available:
            self.map_update = True

    def map_inform(self, map):
        self.map_data = map
        self.map_update = True

    def send_map_toClients(self, cur_time, client_socket):
        if self.map_update:
            resData = {}
            resData['ros_web_cmd'] = "world_map"
            
            if self.map_update:
                resData['update'] = 1
            else:
                resData['update'] = 0

            resData['frame_id'] = self.map_data.header.frame_id
            resData['resolution'] = self.map_data.info.resolution
            resData['width'] = self.map_data.info.width
            resData['height'] = self.map_data.info.height
            resData['origin_pose'] = {}
            resData['origin_pose']['position'] = {}
            resData['origin_pose']['position']['x'] = self.map_data.info.origin.position.x
            resData['origin_pose']['position']['y'] = self.map_data.info.origin.position.y
            resData['origin_pose']['position']['z'] = self.map_data.info.origin.position.z

            resData['origin_pose']['orientation'] = {}
            resData['origin_pose']['orientation']['x'] = self.map_data.info.origin.orientation.x
            resData['origin_pose']['orientation']['y'] = self.map_data.info.origin.orientation.y
            resData['origin_pose']['orientation']['z'] = self.map_data.info.origin.orientation.z
            resData['origin_pose']['orientation']['w'] = self.map_data.info.origin.orientation.w

            resData['data'] = self.map_data.data

            self.map_cache = send_webcmd_msg(client_socket, resData)
            self.map_update = False
            self.map_cache_available = True

    def camera_inform(self, image):
        self.camera_data = image
        self.camera_update = True

    def send_camera_toClients(self, cur_time, client_socket):
        if self.camera_update:
            # resData = {}
            #resData['ros_web_cmd'] = "camera_image"
            # resData['frame_id'] = self.camera_data.header.frame_id
            # resData['format'] = self.camera_data.format
            # resData['data'] = base64.b64encode(self.camera_data.data).decode('ascii')
            # response_text = json.dumps(resData)
            # client_socket.send(response_text)

            image_header = bytearray(2)
            image_header[0] = BINARY_DATA_VERSION
            image_header[1] = BINARY_DATA_TYPE_IMAGE

            image_bytes_data = bytearray(self.camera_data.data)
            image_bin = bytes(image_header + image_bytes_data)

            # not string msg, bytes data use tcp_frame_send_bytes to send directly!!!
            tcp_frame_send_bytes(client_socket, image_bin)
            
            self.camera_update = False

    def explorer_inform(self, frontiers):
        self.explorer_frontier_data = frontiers
        self.explorer_frontier_update = True
    
    def send_explorer_frontiers_toClients(self, cur_time, client_socket):
        if self.explorer_frontier_update:
            resData = {}
            resData['ros_web_cmd'] = "explorer_frontiers"
            # resData['frontiers'] = self.explorer_frontier_data

            msg_text = json.dumps(resData)
            self.map_cache = tcp_frame_send_internal_msg(client_socket, msg_text)
            self.explorer_frontier_update = False

    def listen_tf_pose(self, base_frame_id, frame_id):
        isBaseFrameExist = False
        isSubFrameExist = False

        self.tfTransDict_lock.acquire()

        for key_i in self.tfTransDict:
            if key_i == base_frame_id:
                isBaseFrameExist = True
                for key_j in self.tfTransDict[key_i]:
                    if key_j == frame_id:
                        self.tfTransDict[key_i][key_j]["time"] = rospy.get_time()
                        isSubFrameExist = True
        
        if not isBaseFrameExist:
            self.tfTransDict[base_frame_id] = {}
            self.tfTransDict[base_frame_id][frame_id] = {}
            self.tfTransDict[base_frame_id][frame_id]["time"] = rospy.get_time()
            self.tfTransDict[base_frame_id][frame_id]["trans"] = None
            self.tfTransDict[base_frame_id][frame_id]["rot"] = None
        elif not isSubFrameExist:
            self.tfTransDict[base_frame_id][frame_id] = {}
            self.tfTransDict[base_frame_id][frame_id]["time"] = rospy.get_time()
            self.tfTransDict[base_frame_id][frame_id]["trans"] = None
            self.tfTransDict[base_frame_id][frame_id]["rot"] = None

        self.tfTransDict_lock.release()

    def check_timeout_tf(self, cur_time):
        timeoutDict = []

        self.tfTransDict_lock.acquire()

        # search timeout frame_id
        for key_i in self.tfTransDict:
            for key_j in self.tfTransDict[key_i]:
                if cur_time - self.tfTransDict[key_i][key_j]["time"] > TF_LISTEN_TIMEOUT:
                    print("tf[%s][%s] timeout" %(key_i, key_j))
                    timeoutDict.append(key_j)
        
            # remove frame_id keys
            for i in range(0, len(timeoutDict)):
                print("tf[%s][%s] removed" %(key_i, timeoutDict[i]))
                del self.tfTransDict[key_i][timeoutDict[i]]
        
        timeoutDict = []
        # check whether base_frame_id has no sub frame_id, they need be removed too.
        for key_i in self.tfTransDict:
            if len(self.tfTransDict[key_i]) <= 0:
                print("tf[%s] is empty" %(key_i))
                timeoutDict.append(key_i)
        
        # remove base_frame_id keys
        for i in range(0, len(timeoutDict)):
            print("tf[%s] is removed" %(timeoutDict[i]))
            del self.tfTransDict[timeoutDict[i]]

        self.tfTransDict_lock.release()

    def get_tf_pose(self, base_frame_id, frame_id):
        for key_i in self.tfTransDict:
            if key_i == base_frame_id:
                for key_j in self.tfTransDict[key_i]:
                    if key_j == frame_id:
                        return self.tfTransDict[key_i][key_j]["trans"], self.tfTransDict[key_i][key_j]["rot"]
        return None, None

    def update_tf_pose(self, base_frame_id, frame_id):
        try:
            # print("try get tf fixed frame:%s frame:%s" %(base_frame_id,frame_id))
            (self.tfTransDict[base_frame_id][frame_id]["trans"], self.tfTransDict[base_frame_id][frame_id]["rot"]) = self.tfListener.lookupTransform(base_frame_id, frame_id, rospy.Time(0))
            # print(self.tfTransDict[base_frame_id][frame_id]["trans"])
   
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Error:get tf fixed frame:%s frame:%s" %(base_frame_id,frame_id))
            return None
    
    def update_tf(self):
        # search timeout frame_id
        for key_i in self.tfTransDict:
            for key_j in self.tfTransDict[key_i]:
                self.tfTransDict_lock.acquire()
                self.update_tf_pose(key_i, key_j)
                self.tfTransDict_lock.release()
    
    def check_car_cmd_timeout(self, cur_time):
        # do not send cmd here, it will conflict with navigation's cmd
        # self.send_car_cmd(self.motor_speed, self.steering_angle)
        # we check the time to the last cmd from websocket client, if time > 500ms, then send cmd to stop car
        if self.has_cmd_sent :
            if cur_time - self.last_cmd_time > 1:
                self.has_cmd_sent = False
                self.send_car_cmd(0, 0)
                print("WebCtrl cmd Timeout to stop car!")

    def send_tf_toClients(self, cur_time, client_socket):
        if cur_time - self.last_tf_sent_time < TF_SENT_CYCLE_TIME:
            return

        for key_i in self.tfTransDict:
            for key_j in self.tfTransDict[key_i]:
                trans = self.tfTransDict[key_i][key_j]["trans"]
                rot = self.tfTransDict[key_i][key_j]["rot"]

                if trans :
                    resData = {}
                    resData['ros_web_cmd'] = "restf"
                    resData['base_frame_id'] = key_i
                    resData['frame_id'] = key_j

                    resData['pos'] = {}
                    resData['pos']['x'] = trans[0]
                    resData['pos']['y'] = trans[1]
                    resData['pos']['z'] = trans[2]

                    resData['rotation'] = {}
                    resData['rotation']['x'] = rot[0]
                    resData['rotation']['y'] = rot[1]
                    resData['rotation']['z'] = rot[2]
                    resData['rotation']['w'] = rot[3]

                    send_webcmd_msg(client_socket, resData)
        
        self.last_tf_sent_time = cur_time

def edit_msg_rawString_Nothing(msg_raw_string):
    return msg_raw_string

def laser_replace_Infinity(msg_raw_string):
    msg_raw_string = msg_raw_string.replace("Infinity", LASER_Infinity_value)
    return msg_raw_string

def send_webcmd_msg(client_socket, msgData, edit_msg_rawString_func = edit_msg_rawString_Nothing):
    msgData["msg_class"] = ROS_MSG_BRIDGE_CLASS
    msg_text = json.dumps(msgData)

    # use edit_msg_rawString_func to provide a chance to edit raw string, such as: laser data need use it to replace Infinity to value 0.
    msg_text = edit_msg_rawString_func(msg_text)

    if len(msg_text) <= 0 or len(msg_text) > MAX_LEN_PER_FRAME:
        print("Fatal Error: websocket send %d length string 1!!!" %(len(msg_text)))
    else :
        tcp_frame_send_msg(client_socket, msg_text)
    return msg_text

def submitThread():
    global webcmd
    global isConnected
    global isExitSignal
    global recv_rosservice_msg
    global client_socket

    while not rospy.is_shutdown() and not isExitSignal:
        cur_time = rospy.get_time()

        webcmd.check_car_cmd_timeout(cur_time)
        webcmd.check_timeout_tf(cur_time)
        webcmd.update_tf()

        if isConnected:
            try:
                webcmd.send_tf_toClients(cur_time, client_socket)
                webcmd.send_goal_result_toClients(cur_time, client_socket)
                webcmd.send_laser_scan_toClients(cur_time, client_socket)
                webcmd.send_global_navpath_toClients(cur_time, client_socket)
                webcmd.send_local_navpath_toClients(cur_time, client_socket)
                webcmd.send_map_toClients(cur_time, client_socket)
                webcmd.send_camera_toClients(cur_time, client_socket)
                webcmd.send_explorer_frontiers_toClients(cur_time, client_socket)
            except Exception as e:
                print("submitThread Exception:", e)
                time.sleep(1)
                continue

        # about 10 times per second
        time.sleep(0.1)

def cmd_process(client_socket, bytes_data) :
    global webcmd
    global isExitSignal
    global isConnected
    
    recv_text = bytes_data.decode()
    # print(recv_text)

    jsonMsg = json.loads(recv_text)
    resData = {}

    if jsonMsg['ros_web_cmd'] == "joystick" :
        speed = 0.0
        steer_angle = 0.0
        degree = 0.0

        # json string : '{"ros_web_cmd":"joystick", "id":"0", "password":"123456", "sticks":{"s":"UP","x":1,"y":2}, "btnL":"UP", "btnR":"UP","rotate":{"y":0,"x":1}}';
        # jsonMsg['id'] is the index of robots
        # jsonMsg['password'] is the password of robots

        if jsonMsg['sticks']['s'] == "DOWN" or jsonMsg['sticks']['s'] == "MOVE" :
            length = math.sqrt(math.pow(jsonMsg['sticks']['x'], 2) + math.pow(jsonMsg['sticks']['y'], 2))
            if length > 0:
                degree = math.degrees(math.acos(abs(jsonMsg['sticks']['y'])/length))

                if jsonMsg['sticks']['y'] < 0 :
                    speed = (length*(-1.0))/100
                else :
                    speed = length/100

                steer_angle = math.acos(abs(jsonMsg['sticks']['y'])/length)

                # 消除人手抖动，更容易走直线
                if steer_angle < straight_delta_angle :
                    steer_angle = 0
                else :
                    steer_angle -= straight_delta_angle

                if jsonMsg['sticks']['x'] < 0 :
                    # turn left : steer_angle > 0                    
                    if steer_angle > max_steering_angle :
                        steer_angle = max_steering_angle
                elif jsonMsg['sticks']['x'] > 0 :
                    # turn right : steer_angle < 0
                    steer_angle = -1 * steer_angle
                    if steer_angle < (-1*max_steering_angle) :
                        steer_angle = (-1*max_steering_angle)
                else :
                    # go straight
                    steer_angle = 0
        else :
            degree = 0
            speed = 0
            steer_angle = 0
        
        if speed > 1 :
            speed = 1

        if speed < -1 :
            speed = -1

        if speed < 0:
            steer_angle = -steer_angle
        
        # we just send car cmd here, because this will not conflict with other's cmd senders
        webcmd.send_car_cmd(speed, steer_angle)

        print("car cmd: speed=%f steer_angle=%f (degree:%f)" %(speed, steer_angle, degree))
    
    elif jsonMsg['ros_web_cmd'] == "setinitpose" :
        # json string : '{"ros_web_cmd":"setinitpose", "id":"0", "password":"123456", "pose":{"pos": {"x": 0, "y":1, "z":0}, "orientation":{"x": 0, "y":0, "z":1, "w":0}}"
        webcmd.set_init_pose(jsonMsg['pose'])
    elif jsonMsg['ros_web_cmd'] == "setgoalpose" :
        # json string : '{"ros_web_cmd":"setgoalpose", "id":"0", "password":"123456", "pose":{"pos": {"x": 0, "y":1, "z":0}, "orientation":{"x": 0, "y":0, "z":1, "w":0}}"
        webcmd.set_goal_pose(jsonMsg['pose'])
    elif jsonMsg['ros_web_cmd'] == "cancelgoal" :
        webcmd.cancel_goal(jsonMsg['goal_id'])
    elif jsonMsg['ros_web_cmd'] == "gettf" :
        # json string : '{"ros_web_cmd":"gettf", "id":"0", "password":"123456", "base_frame_id":"map", "frame_id":"base_foot_print"}'
        webcmd.listen_tf_pose(jsonMsg['base_frame_id'], jsonMsg['frame_id'])
    elif jsonMsg['ros_web_cmd'] == "get_map" :
        # force to set map_update to true if map cache is avalible, or just wait for map to be updated
        webcmd.force_send_map_cache()
    else :
        print("unkown msg: %s" %(recv_text))
        resData['ros_web_cmd'] = "error"
        resData['content'] = "unkown msg:" + recv_text

        send_webcmd_msg(client_socket, resData)

# this function is used to send internal msg to webcmd client, don't need be forwarded to webcmd server
def tcp_frame_send_internal_msg(client_socket, msg):
    if not isConnected:
        return None

    data = msg.encode()

    # print("data length = %d" %(len(data)))

    # magic number : 0523 as header flag means msg string data
    head_data = bytearray(4)
    head_data[0] = 0
    head_data[1] = 5
    head_data[2] = 2
    head_data[3] = 1

    # print("head_data %d bytes" %(len(head_data)))

    # real data length
    length_data = bytearray(4)
    length_data[0] = ((len(data) & int('0xff000000', 16)) >> 24)
    length_data[1] = ((len(data) & int('0x00ff0000', 16)) >> 16)
    length_data[2] = ((len(data) & int('0x0000ff00', 16)) >> 8)
    length_data[3] = ((len(data) & int('0x000000ff', 16)))

    # print("length_data %d bytes" %(len(length_data)))

    bytes_header_data = bytes(head_data + length_data)

    sent_len = 0
    while sent_len < len(bytes_header_data) :
        ret = client_socket.send(bytes_header_data[sent_len:])
        # print("send %d bytes" %(ret))
        sent_len += ret

    # print("start to send %d bytes" %(len(bytes_send_data)))

    sent_len = 0
    while sent_len < len(data) :
        ret = client_socket.send(data[sent_len:])
        # print("send %d bytes" %(ret))
        sent_len += ret

    return msg

def tcp_frame_send_msg(client_socket, msg):
    if not isConnected:
        return None

    data = msg.encode()

    # print("data length = %d" %(len(data)))

    # magic number : 0523 as header flag means msg string data
    head_data = bytearray(4)
    head_data[0] = 0
    head_data[1] = 5
    head_data[2] = 2
    head_data[3] = 3

    # print("head_data %d bytes" %(len(head_data)))

    # real data length
    length_data = bytearray(4)
    length_data[0] = ((len(data) & int('0xff000000', 16)) >> 24)
    length_data[1] = ((len(data) & int('0x00ff0000', 16)) >> 16)
    length_data[2] = ((len(data) & int('0x0000ff00', 16)) >> 8)
    length_data[3] = ((len(data) & int('0x000000ff', 16)))

    # print("length_data %d bytes" %(len(length_data)))

    bytes_header_data = bytes(head_data + length_data)

    sent_len = 0
    while sent_len < len(bytes_header_data) :
        ret = client_socket.send(bytes_header_data[sent_len:])
        # print("send %d bytes" %(ret))
        sent_len += ret

    # print("start to send %d bytes" %(len(bytes_send_data)))

    sent_len = 0
    while sent_len < len(data) :
        ret = client_socket.send(data[sent_len:])
        # print("send %d bytes" %(ret))
        sent_len += ret

    return msg

def tcp_frame_send_bytes(client_socket, bytes_data):
    if not isConnected:
        return None

    # print("data length = %d" %(len(data)))

    # magic number : 0520 as header flag means bytes data frame
    head_data = bytearray(4)
    head_data[0] = 0
    head_data[1] = 5
    head_data[2] = 2
    head_data[3] = 0

    # print("head_data %d bytes" %(len(head_data)))

    # real data length
    length_data = bytearray(4)
    length_data[0] = ((len(bytes_data) & int('0xff000000', 16)) >> 24)
    length_data[1] = ((len(bytes_data) & int('0x00ff0000', 16)) >> 16)
    length_data[2] = ((len(bytes_data) & int('0x0000ff00', 16)) >> 8)
    length_data[3] = ((len(bytes_data) & int('0x000000ff', 16)))

    # print("length_data %d bytes" %(len(length_data)))

    bytes_header_data = bytes(head_data + length_data)

    sent_len = 0
    while sent_len < len(bytes_header_data) :
        ret = client_socket.send(bytes_header_data[sent_len:])
        # print("send %d bytes" %(ret))
        sent_len += ret

    # print("start to send %d bytes" %(len(bytes_send_data)))

    sent_len = 0
    while sent_len < len(bytes_data) :
        ret = client_socket.send(bytes_data[sent_len:])
        # print("send %d bytes" %(ret))
        sent_len += ret

    return bytes_data

def exit_app(signum, frame):
    global isExitSignal
    global isConnected
    global server_socket
    global client_socket

    print("get exit_app signal on rosmsg_bridge ...")
    isExitSignal = True

    if isConnected:
        client_socket.close()
    server_socket.close()

def receive_exactly_len(local_socket, read_len):
    bytes_data = bytearray()
    total_len = 0
                
    while total_len < read_len:
        try:
            data = local_socket.recv(read_len-total_len)
            if data == '' or not data:
                return None
            
            total_len += len(data)
            bytes_data += bytearray(data)

        except socket.timeout:
            continue
            
        except Exception as e:
            print("receive_exactly_len Exception:", e)
            break

    return bytes_data
    
def main(argv):
    global webcmd
    global isExitSignal
    global isConnected
    global server_socket
    global client_socket

    # init ROS node 
    rospy.init_node(ROS_MSG_BRIDGE_CLASS)
    rospy.loginfo("Starting ROS message bridge node")

    print("wait for connection to ros time service...")
    while True:
        if rospy.get_time() > 0 :
            break
        time.sleep(1)
    print("ros time service connected!")

    webcmd = webcmd_forward()

    # start submit message thread
    server = threading.Thread(target=submitThread)
    server.start()

    webcmd.startSubscribers()

    host_address = rospy.get_param('~server_address',"127.0.0.1")
    port = rospy.get_param('~server_port','8001')
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((host_address, port))
    server_socket.listen(1)

    while not isExitSignal:
        try:
            client_socket, addr = server_socket.accept()
            print('Connected by ', addr)
            
            isConnected = True
        
        except socket.timeout:
            continue

        except Exception as e:
            print("server_socket.accept Exception:", e)
            break

        check_len = 0
        total_len = 0
        read_request_len = FRAME_HEADER_LEN # always read 8 bytes header at first!

        while not isExitSignal:
            data = receive_exactly_len(client_socket, read_request_len)
            if not data:
                break

            total_len += len(data)

            if total_len == FRAME_HEADER_LEN and check_len == 0:
                #print("header=%d:%d:%d:%d" %(bytes_data[0], bytes_data[1], bytes_data[2], bytes_data[3]))
                if data[0] == 0 and data[1] == 5 and data[2] == 2 and data[3] == 3 :
                    #print("Got a right head flag 0523 !")
                    check_len = (int(data[4]) << 24) + (int(data[5]) << 16) + (int(data[6]) << 8) + int(data[7])
                    #print("check len=%d" %(check_len))

                    # then read all datas at next time!
                    read_request_len = check_len
                    continue
                else:
                    print("Fatal Error: wrong data frame head magic number!")
                    break
            
            if total_len == check_len+FRAME_HEADER_LEN :
                # print("get a completed data frame=%d" %(total_len-8))
                cmd_process(client_socket, data)

                check_len = 0
                total_len = 0
                read_request_len = FRAME_HEADER_LEN
            else :
                print("Fatal Error(ROSMSG): total_len=%d not equal to (check_len+8)=%d" %(total_len, check_len+FRAME_HEADER_LEN))
                break
        
        isConnected = False
        client_socket.close()
        print("ros msg bridge client disconnected!")


if __name__ == "__main__":
    signal.signal(signal.SIGINT, exit_app)
    signal.signal(signal.SIGTERM, exit_app)
    signal.signal(signal.SIGHUP, exit_app)

    main(sys.argv[1:])
