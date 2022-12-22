#!/usr/bin/env python3
#! -*- coding: utf-8 -*-

import rospy
import asyncio
import websockets
import threading
import json
import math

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

# max steering angle: 30 degree
max_steering_angle = 0.5

# 由于人手抖动，控制精度不可能保证直行时角度正好是0，需要设定一个抖动的角度范围，这个抖动角度范围内都是认为角度为0
# 超过这个抖动角度后，才开始计算转向角度，而且还要减去这个抖动的角度数值
straight_delta_angle = 0.18

websocket_users = set()

# 接收客户端消息并处理，这里只是简单把客户端发来的返回回去
async def recv_user_msg(websocket):
    global webcmd
    
    while True:
        recv_text = await websocket.recv()
        print("recv_text:%s" %(recv_text))

        jsonMsg = json.loads(recv_text)
        resData = {}
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
        else :
            print("unkown msg: %s" %(recv_text))
            resData['cmd'] = "error"
            resData['content'] = f'unkown msg:{recv_text}'

            response_text = json.dumps(resData)
            print("send response: %s" %(response_text))
            await websocket.send(response_text)

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

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    ws_server = websockets.serve(run, host_address, 8181)

    print("%s:%d websocket start..." %(host_address, port))

    loop.run_until_complete(ws_server)
    loop.run_forever() # this is missing
    loop.close()

class webcmd_forward:
    def __init__(self):
        self.has_cmd_sent = False
        self.last_cmd_time = 0.0

        # if True, will use topic /ackermann_cmd to publish the control command of car
        self.ackermann_mode = bool(rospy.get_param('~ackermann_mode',False))

        #define topic publisher or subscriber
        if self.ackermann_mode :
            self.pub_cmd = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=5)
        else:
            self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        self.rate = rospy.Rate(10)

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

    def timeoutCheckThread(self):
        while not rospy.is_shutdown():
            # do not send cmd here, it will conflict with navigation's cmd
            # self.send_car_cmd(self.motor_speed, self.steering_angle)
            # we check the time to the last cmd from websocket client, if time > 500ms, then send cmd to stop car
            cur_time = rospy.get_time()
            if self.has_cmd_sent :
                if cur_time - self.last_cmd_time > 1:
                    self.has_cmd_sent = False
                    self.send_car_cmd(0, 0)
                    print("WebCtrl cmd Timeout to stop car!")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # init ROS node 
        rospy.init_node("webcmd_forward")
        rospy.loginfo("Starting Web Cmd Follow node")
        
        server = threading.Thread(target=threadcreate_callback, daemon=True)
        server.start()

        webcmd = webcmd_forward()
        webcmd.timeoutCheckThread()

    except KeyboardInterrupt:
        print("Shutting down webcmd_forward node.")
