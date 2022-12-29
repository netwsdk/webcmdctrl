#!/usr/bin/env python3
#! -*- coding: utf-8 -*-

import rospy
import asyncio
import websockets
import threading
import json
import math
import random
import tf2_ros
import time
import base64

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseActionResult
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CompressedImage

# some data of ros is not continuous stream. so we save the data, and send them to clients in a loop.
# So we need define the cycle(unit: second) for each kinds of data sent to clients
MAP_SENT_CYCLE_TIME = 5
TF_SENT_CYCLE_TIME = 0.1

# TF listener timeout
TF_LISTEN_TIMEOUT = 30

# max steering angle: 30 degree
max_steering_angle = 0.5

# 由于人手抖动，控制精度不可能保证直行时角度正好是0，需要设定一个抖动的角度范围，这个抖动角度范围内都是认为角度为0
# 超过这个抖动角度后，才开始计算转向角度，而且还要减去这个抖动的角度数值
straight_delta_angle = 0.18

websocket_users = set()

# 接收客户端消息并处理，这里只是简单把客户端发来的返回回去
async def recv_user_msg(websocket):
    global webcmd
    global websocket_users

    websocket_users.add(websocket)
    
    while True:
        recv_text = await websocket.recv()
        # print("recv_text:%s" %(recv_text))

        jsonMsg = json.loads(recv_text)
        resData = {}

        # print("cmd:%s" %(jsonMsg['cmd']))

        if jsonMsg['cmd'] == "acquire" :
            resData['cmd'] = "accepted"
            resData['name'] = "wsdk car"

            response_text = json.dumps(resData)
            print("send response: %s" %(response_text))
            await websocket.send(response_text)
        elif jsonMsg['cmd'] == "joystick" :
            # json string : '{"cmd":"joystick", "id":"0", "password":"123456", "sticks":{"s":"UP","x":1,"y":2}, "btnL":"UP", "btnR":"UP","rotate":{"y":0,"x":1}}';
            # jsonMsg['id'] is the index of robots
            # jsonMsg['password'] is the password of robots

            if jsonMsg['sticks']['s'] == "DOWN" or jsonMsg['sticks']['s'] == "MOVE" :
                length = math.sqrt(math.pow(jsonMsg['sticks']['x'], 2) + math.pow(jsonMsg['sticks']['y'], 2))
                if length <= 0:
                    return

                degree = math.degrees(math.acos(abs(jsonMsg['sticks']['y'])/length))

                if jsonMsg['sticks']['y'] < 0 :
                    speed = (length*(-1.0))/100
                else :
                    speed = length/100

                    straight_delta_angle

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
            
            # we just send car cmd here, because this will not conflict with other's cmd senders
            webcmd.send_car_cmd(speed, steer_angle)

            print("car cmd: speed=%f steer_angle=%f (degree:%f)" %(speed, steer_angle, degree))
        
        elif jsonMsg['cmd'] == "setinitpose" :
            # json string : '{"cmd":"setinitpose", "id":"0", "password":"123456", "pose":{"pos": {"x": 0, "y":1, "z":0}, "orientation":{"x": 0, "y":0, "z":1, "w":0}}"
            webcmd.set_init_pose(jsonMsg['pose'])
        elif jsonMsg['cmd'] == "setgoalpose" :
            # json string : '{"cmd":"setgoalpose", "id":"0", "password":"123456", "pose":{"pos": {"x": 0, "y":1, "z":0}, "orientation":{"x": 0, "y":0, "z":1, "w":0}}"
            webcmd.set_goal_pose(jsonMsg['pose'])
        elif jsonMsg['cmd'] == "gettf" :
            # json string : '{"cmd":"gettf", "id":"0", "password":"123456", "base_frame_id":"map", "frame_id":"base_foot_print"}'
            webcmd.listen_tf_pose(jsonMsg['base_frame_id'], jsonMsg['frame_id'])
        else :
            print("unkown msg: %s" %(recv_text))
            resData['cmd'] = "error"
            resData['content'] = f'unkown msg:{recv_text}'

            response_text = json.dumps(resData)
            print("send response: %s" %(response_text))
            await websocket.send(response_text)

async def sendToAllClients(jsonmsg):
    global websocket_users

    client_closed = []
    for client in websocket_users:
        try:
            await client.send(jsonmsg)
        except websockets.ConnectionClosed:
            print("ConnectionClosed...")    # 链接断开
            client_closed.append(client)
        except websockets.InvalidState:
            print("InvalidState...")    # 无效状态
        except Exception as e:
            print("Exception:", e)
    
    # remove client disconnected
    for client in client_closed:
        websocket_users.remove(client)

# 服务器端主逻辑
async def run(websocket, path):
    global webcmd
    while True:
        try:
            await recv_user_msg(websocket)
        except websockets.ConnectionClosed:
            print("ConnectionClosed...", path)    # 链接断开
            print("websocket_users old:", websocket_users)
            websocket_users.remove(websocket)
            print("websocket_users new:", websocket_users)

            # when connection close, stop the car!
            webcmd.send_car_cmd(0, 0)
            break
        except websockets.InvalidState:
            print("InvalidState...")    # 无效状态
            break
        except Exception as e:
            print("Exception:", e)

def threadcreate_callback():
    host_address = rospy.get_param('~address',"127.0.0.1")
    port = int(rospy.get_param('~port','8181'))

    # this is a thread, so we use new_event_loop to create a new loop for this thread
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    ws_server = websockets.serve(run, host_address, 8181)

    print("%s:%d websocket start..." %(host_address, port))

    loop.run_until_complete(ws_server)
    loop.run_forever() # this is missing
    loop.close()

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

class webcmd_forward:
    def __init__(self):
        self.has_cmd_sent = False
        self.last_cmd_time = 0.0

        self.last_tf_sent_time = rospy.get_time()

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
        self.last_map_sent_time = rospy.get_time()

        self.camera_data = None
        self.camera_update = False

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

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.tfTransDict = {} # dict tfTransDict[base_frame_id][frame_id] is the result of tf
        self.tfTransDict_lock = threading.Lock()
        
        # 20Hz because we want to support camera image frame rate up to 20 frames / seconds (if camera framerate is > 20, then will drop some of data)
        self.rate = rospy.Rate(20)

    def startSubscribers(self):
        # navigation result
        rospy.Subscriber("move_base/result", MoveBaseActionResult, goal_result_callback)
        # laser scan points
        rospy.Subscriber("scan", LaserScan, laser_scan_callback)
        # global path of navigation (AMCL)
        rospy.Subscriber("move_base/GlobalPlanner/plan", Path, global_navpath_callback)
        # local path of navigation (TEB)
        rospy.Subscriber("move_base/TebLocalPlannerROS/local_plan", Path, local_navpath_callback)
        # world map
        rospy.Subscriber("map", OccupancyGrid, map_callback)
        # camera: /image/compressed
        rospy.Subscriber("image_raw/compressed", CompressedImage, camera_callback)

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

    def goal_result_inform(self, result):
        self.goal_status = result
        self.goal_update = True
    
    async def send_goal_result_toClients(self, cur_time):
        global websocket_users
        if self.goal_update :
            resData = {}
            resData['cmd'] = "goal_result"
            resData['goal_id'] = self.goal_status.status.goal_id.id
            resData['status'] = self.goal_status.status.status
            resData['desc'] = self.goal_status.status.text

            response_text = json.dumps(resData)

            await sendToAllClients(response_text)
            
            self.goal_update = False
    
    def laser_scan_inform(self, laser):
        self.laser_scan_data = laser
        self.laser_scan_update = True

    async def send_laser_scan_toClients(self, cur_time):
        global websocket_users
        if self.laser_scan_update :
            resData = {}
            resData['cmd'] = "laser_scan"
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

            response_text = json.dumps(resData)

            await sendToAllClients(response_text)
            
            self.laser_scan_update = False
    
    def global_navpath_inform(self, path):
        self.global_navpath_data = path
        self.global_navpath_update = True

    async def send_global_navpath_toClients(self, cur_time):
        global websocket_users
        if self.global_navpath_update :
            resData = {}
            resData['cmd'] = "global_navpath"
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

            response_text = json.dumps(resData)

            await sendToAllClients(response_text)
            
            self.global_navpath_update = False
    
    def local_navpath_inform(self, path):
        self.local_navpath_data = path
        self.local_navpath_update = True

    async def send_local_navpath_toClients(self, cur_time):
        global websocket_users
        if self.local_navpath_update :
            resData = {}
            resData['cmd'] = "local_navpath"
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

            response_text = json.dumps(resData)

            await sendToAllClients(response_text)
            
            self.local_navpath_update = False

    def map_inform(self, map):
        self.map_data = map
        self.map_update = True

    async def send_map_toClients(self, cur_time):
        global websocket_users

        if cur_time - self.last_map_sent_time > MAP_SENT_CYCLE_TIME or self.map_update:
            resData = {}
            resData['cmd'] = "world_map"
            
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

            response_text = json.dumps(resData)

            await sendToAllClients(response_text)
            
            self.map_update = False
            self.last_map_sent_time = cur_time

    def camera_inform(self, image):
        self.camera_data = image
        self.camera_update = True

    async def send_camera_toClients(self, cur_time):
        global websocket_users

        if self.camera_update:
            resData = {}
            resData['cmd'] = "camera_image"

            resData['frame_id'] = self.camera_data.header.frame_id
            resData['format'] = self.camera_data.format
            resData['data'] = base64.b64encode(self.camera_data.data).decode('ascii')

            response_text = json.dumps(resData)

            await sendToAllClients(response_text)
            
            self.camera_update = False

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
        elif not isSubFrameExist:
            self.tfTransDict[base_frame_id][frame_id] = {}
            self.tfTransDict[base_frame_id][frame_id]["time"] = rospy.get_time()
            self.tfTransDict[base_frame_id][frame_id]["trans"] = None

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
        
        timeoutDict.clear()
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

    def get_tf2_pose(self, base_frame_id, frame_id):
        for key_i in self.tfTransDict:
            if key_i == base_frame_id:
                for key_j in self.tfTransDict[key_i]:
                    if key_j == frame_id:
                        return self.tfTransDict[key_i][key_j]["trans"]
        return None

    def update_tf2_pose(self, base_frame_id, frame_id):
        try:
            # print("try get tf fixed frame:%s frame:%s" %(base_frame_id,frame_id))
            self.tfTransDict[base_frame_id][frame_id]["trans"] = self.tfBuffer.lookup_transform(base_frame_id, frame_id, rospy.Time(0))
            # print(self.tfTransDict[base_frame_id][frame_id]["trans"])
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None
    
    def update_tf(self):
        # search timeout frame_id
        for key_i in self.tfTransDict:
            for key_j in self.tfTransDict[key_i]:
                self.tfTransDict_lock.acquire()
                self.update_tf2_pose(key_i, key_j)
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

    async def send_tf_toClients(self, cur_time):
        global websocket_users

        if cur_time - self.last_tf_sent_time < TF_SENT_CYCLE_TIME:
            return

        for key_i in self.tfTransDict:
            for key_j in self.tfTransDict[key_i]:
                trans = self.tfTransDict[key_i][key_j]["trans"]
                if trans :
                    resData = {}
                    resData['cmd'] = "restf"
                    resData['base_frame_id'] = key_i
                    resData['frame_id'] = key_j

                    resData['pos'] = {}
                    resData['pos']['x'] = trans.transform.translation.x
                    resData['pos']['y'] = trans.transform.translation.y
                    resData['pos']['z'] = trans.transform.translation.z

                    resData['rotation'] = {}
                    resData['rotation']['x'] = trans.transform.rotation.x
                    resData['rotation']['y'] = trans.transform.rotation.y
                    resData['rotation']['z'] = trans.transform.rotation.z
                    resData['rotation']['w'] = trans.transform.rotation.w

                    response_text = json.dumps(resData)

                    await sendToAllClients(response_text)
        
        self.last_tf_sent_time = cur_time

    def timeoutCheckThread(self):
        while not rospy.is_shutdown():
            cur_time = rospy.get_time()

            self.check_car_cmd_timeout(cur_time)
            self.check_timeout_tf(cur_time)
            self.update_tf()

            # this is not a thread, it is main process. so we use get_event_loop to get the loop
            loop = asyncio.get_event_loop()
            future = asyncio.gather(self.send_tf_toClients(cur_time), 
                            self.send_goal_result_toClients(cur_time), 
                            self.send_laser_scan_toClients(cur_time), 
                            self.send_global_navpath_toClients(cur_time),
                            self.send_local_navpath_toClients(cur_time),
                            self.send_map_toClients(cur_time),
                            self.send_camera_toClients(cur_time))
            result = loop.run_until_complete(future)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # init ROS node 
        rospy.init_node("webcmd_forward")
        rospy.loginfo("Starting Web Cmd Follow node")

        print("wait for connection to ros time service...")
        while True:
            if rospy.get_time() > 0 :
                break
            time.sleep(1)
        print("ros time service connected!")

        webcmd = webcmd_forward()
        server = threading.Thread(target=threadcreate_callback, daemon=True)
        server.start()

        webcmd.startSubscribers()
        #webcmd.listen_tf_pose("map", "base_footprint")
        webcmd.timeoutCheckThread()

    except KeyboardInterrupt:
        print("Shutting down webcmd_forward node.")
