#!/usr/bin/env python3
#! -*- coding: utf-8 -*-

import asyncio
import websockets
import threading
import json
import math
import random
import time
import base64
import uuid

# socket message provider ID string, unique
WEBCMD_CLIENT_MSG_CLASS = "webcmd_client"
# socket message provider ID string, unique
ROS_MSG_BRIDGE_CLASS = "ros_msg_bridge"
# this is webcmd server itself class string
WEBCMD_SERVER_MSG_CLASS = "ros_webcmd_server"

robots_set = set()
users_set = set()

class client_unit:
    def __init__(self, websocket):
        self.role_type = ""
        self.subtype = ""
        self.name = ""
        self.uid = ""
        self.service = ""
        self.explorer_map_status = ""
        self.is_map_ready = False
        self.websocket = websocket
        self.isBusy = False # if bind user with a robot, then user and robot 's client_unit both need set this to True
        self.to_unit = self # default to_unit is self, for client, can use bindWith to set it to a robot

    def update(self, role_type, subtype, name, uid):
        self.role_type = role_type
        self.subtype = subtype
        self.name = name
        self.uid = "{0}".format(uid)  # enforce uid to be a string

    # to_unit is class client_unit
    def bindWith(self, to_unit):
        self.to_unit = to_unit
        self.isBusy = True

    def rleaseToUnit(self):
        self.to_unit = self # just point to self
        self.isBusy = False


async def send_webcmd_msg(websocket, msgData):
    msgData["msg_class"] = WEBCMD_SERVER_MSG_CLASS
    msg_text = json.dumps(msgData)
    await websocket.send(msg_text)

# 服务器端主逻辑
async def run(websocket, path):
    global robots_set
    global users_set

    role_type = "none"
    print("one client connected!")

    client_handler = client_unit(websocket)

    try:
        while True:
            recv_msg = await websocket.recv()
            # print("recv_msg:%s" %(recv_msg))

            if isinstance(recv_msg, str) :
                jsonMsg = json.loads(recv_msg)

                if jsonMsg['ros_web_cmd'] == "role" :
                    if jsonMsg['type'] == "robot":
                        role_type = "robot"
                        client_handler.update(jsonMsg['type'], jsonMsg['subtype'], jsonMsg['name'], jsonMsg['uid'])
                        # ensure set isBusy to false !!! when indicate robot role.
                        client_handler.rleaseToUnit()
                        client_handler.to_unit.rleaseToUnit()

                        robots_set.add(client_handler)

                        print("one robot online...")

                        infoData = {}
                        infoData['ros_web_cmd'] = "role_accepted"
                        infoData['uid'] = client_handler.uid

                        await send_webcmd_msg(websocket, infoData)

                        robotInfo = {}
                        robotInfo['ros_web_cmd'] = "robot_online"
                        robotInfo['name'] = client_handler.name
                        robotInfo['uid'] = client_handler.uid
                        robotInfo['subtype'] = client_handler.subtype
                        for one_user in users_set:
                            await send_webcmd_msg(one_user.websocket, robotInfo)

                    elif jsonMsg['type'] == "user":
                        role_type = "user"
                        client_handler.update(jsonMsg['type'], jsonMsg['subtype'], jsonMsg['name'], uuid.uuid1())
                        client_handler.rleaseToUnit()
                        client_handler.to_unit.rleaseToUnit()

                        users_set.add(client_handler)

                        print("one user(%s) online..." %(client_handler.uid))

                        infoData = {}
                        infoData['ros_web_cmd'] = "role_accepted"
                        infoData['uid'] = client_handler.uid

                        await send_webcmd_msg(websocket, infoData)
                    else:
                        print("Error role type:", jsonMsg['type'])

                        infoData = {}
                        infoData['ros_web_cmd'] = "role_error"
                        infoData['errmsg'] = "Error role type: {0}".format(jsonMsg['type'])

                        await send_webcmd_msg(websocket, infoData)

                elif jsonMsg['ros_web_cmd'] == "get_robots" :
                    print("user request to get list of robots")
                    infoData = {}
                    infoData['ros_web_cmd'] = "robot_list"
                    infoData['robots'] = []
                    for robot in robots_set:
                        robotInfo = {}
                        robotInfo['name'] = robot.name
                        robotInfo['uid'] = robot.uid
                        robotInfo['subtype'] = robot.subtype
                        robotInfo['service'] = robot.service
                        robotInfo['explorer_map_status'] = robot.explorer_map_status
                        robotInfo['is_map_ready'] = robot.is_map_ready
                        robotInfo['busy'] = robot.isBusy

                        infoData['robots'].append(robotInfo)
                    
                    await send_webcmd_msg(websocket, infoData)

                elif jsonMsg['ros_web_cmd'] == "bind_robot" :
                    infoData = {}
                    if role_type == "user":
                        print("user request to bind robot:%s" %(jsonMsg['uid']))

                        uid_found = False
                        for robot in robots_set:
                            if robot.uid == jsonMsg['uid'] :
                                uid_found = True

                                if robot.isBusy:
                                    infoData['ros_web_cmd'] = "bind_error"
                                    infoData['errmsg'] = "robot is busy"

                                    await send_webcmd_msg(websocket, infoData)
                                else :
                                    infoData['ros_web_cmd'] = "bind_ok"
                                    infoData['uid'] = robot.uid

                                    await send_webcmd_msg(websocket, infoData)

                                    robot.bindWith(client_handler)
                                    client_handler.bindWith(robot)
                        
                        if not uid_found :
                            infoData['ros_web_cmd'] = "bind_error"
                            infoData['errmsg'] = "robot uid is unknown"

                            await send_webcmd_msg(websocket, infoData)
                    else :
                        print("Error: only user can bind robot")

                        infoData['ros_web_cmd'] = "bind_error"
                        infoData['errmsg'] = "only user can bind robot"

                        await send_webcmd_msg(websocket, infoData)

                elif jsonMsg['ros_web_cmd'] == "release_robot" :
                    if role_type == "user":
                        print("user request to release robot")
                        # just call release, even this client has no to_unit
                        client_handler.to_unit.rleaseToUnit()
                        client_handler.rleaseToUnit()
                    else :
                        print("Error: only user can release robot")

                else : 
                    # forward all msg to the client_handler.to_unit
                    #if role_type == "user" :
                    #    print("foward user msg: %s" %(recv_msg))
                    
                    #if role_type == "robot" :
                    #    if jsonMsg['ros_web_cmd'] == "switch_ros_service_response" :
                    #        print("foward robot msg: %s" %(recv_msg))

                    # need update the status into robot list
                    if jsonMsg['ros_web_cmd'] == "ros_service_status" :
                        client_handler.service = jsonMsg['service']
                        client_handler.explorer_map_status = jsonMsg['explorer_map_status']
                        client_handler.is_map_ready = jsonMsg['is_map_ready']

                    if client_handler.isBusy :
                        try:
                            await client_handler.to_unit.websocket.send(recv_msg)
                        except websockets.ConnectionClosed:
                            # below 4 lines codes can't be called, because will cause error "remove unit cause size changed in sendToAllClients for loop"
                            print("Connection Closed to_unit...")    # 链接断开
                            client_handler.to_unit.rleaseToUnit()
                            client_handler.rleaseToUnit()
                        except websockets.InvalidState:
                            print("InvalidState to_unit...")    # 无效状态
                            client_handler.to_unit.rleaseToUnit()
                            client_handler.rleaseToUnit()
                        except Exception as e:
                            print("Exception to_unit:", e)
                            client_handler.to_unit.rleaseToUnit()
                            client_handler.rleaseToUnit()
            
            elif isinstance(recv_msg, bytes) :
                #if role_type == "user" :
                #    print("foward user bytes data: %d" %(len(recv_msg)))
                    
                #if role_type == "robot" :
                #    print("foward robot bytes data: %d" %(len(recv_msg)))

                # bytes data (like: image of camera) forward to binded user
                if client_handler.isBusy :
                    try:
                        await client_handler.to_unit.websocket.send(recv_msg)
                    except websockets.ConnectionClosed:
                        # below 4 lines codes can't be called, because will cause error "remove unit cause size changed in sendToAllClients for loop"
                        print("Connection Closed to_unit...")    # 链接断开
                        client_handler.to_unit.rleaseToUnit()
                        client_handler.rleaseToUnit()
                    except websockets.InvalidState:
                        print("InvalidState to_unit...")    # 无效状态
                        client_handler.to_unit.rleaseToUnit()
                        client_handler.rleaseToUnit()
                    except Exception as e:
                        print("Exception to_unit:", e)
                        client_handler.to_unit.rleaseToUnit()
                        client_handler.rleaseToUnit()
    
    except websockets.ConnectionClosed:
        print("Connection Closed...")    # 链接断开
        if role_type == "robot":
            robots_set.remove(client_handler)
            client_handler.rleaseToUnit()
            client_handler.to_unit.rleaseToUnit()

            robotInfo = {}
            robotInfo['ros_web_cmd'] = "robot_offline"
            robotInfo['name'] = client_handler.name
            robotInfo['uid'] = client_handler.uid
            robotInfo['subtype'] = client_handler.subtype
            for one_user in users_set:
                await send_webcmd_msg(one_user.websocket, robotInfo)

        elif role_type == "user":
            users_set.remove(client_handler)
            client_handler.to_unit.rleaseToUnit()
            client_handler.rleaseToUnit()
    except websockets.InvalidState:
        print("InvalidState...")    # 无效状态
        if role_type == "robot":
            robots_set.remove(client_handler)
            client_handler.rleaseToUnit()
            client_handler.to_unit.rleaseToUnit()

            robotInfo = {}
            robotInfo['ros_web_cmd'] = "robot_offline"
            robotInfo['name'] = client_handler.name
            robotInfo['uid'] = client_handler.uid
            robotInfo['subtype'] = client_handler.subtype
            for one_user in users_set:
                await send_webcmd_msg(one_user.websocket, robotInfo)

        elif role_type == "user":
            users_set.remove(client_handler)
            client_handler.to_unit.rleaseToUnit()
            client_handler.rleaseToUnit()

    except Exception as e:
        print("Exception:", e)
        if role_type == "robot":
            robots_set.remove(client_handler)
            client_handler.rleaseToUnit()
            client_handler.to_unit.rleaseToUnit()

            robotInfo = {}
            robotInfo['ros_web_cmd'] = "robot_offline"
            robotInfo['name'] = client_handler.name
            robotInfo['uid'] = client_handler.uid
            robotInfo['subtype'] = client_handler.subtype
            for one_user in users_set:
                await send_webcmd_msg(one_user.websocket, robotInfo)
                
        elif role_type == "user":
            users_set.remove(client_handler)
            client_handler.to_unit.rleaseToUnit()
            client_handler.rleaseToUnit()

if __name__ == '__main__':
    try:
        host_address = ""
        # host_address = "172.31.163.120"
        # host_address = "192.168.137.204"
        port = 9523

        # this is a thread, so we use new_event_loop to create a new loop for this thread
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        ws_server = websockets.serve(run, host_address, port, max_size=8*1024*1024, max_queue=128)

        print("%s:%d websocket start..." %(host_address, port))

        loop.run_until_complete(ws_server)
        loop.run_forever() # this is missing
        loop.close()

    except KeyboardInterrupt:
        print("Shutting down webcmd_forward node.")
