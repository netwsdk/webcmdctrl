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
import socket
import queue
import signal
import sys, os, getopt
import shlex
import subprocess

isExitSignal = False
simulation_mode = False
open_rviz = False
isConnected = False
isRosMsgBridgeConnected = False

# socket message provider ID string, unique
WEBCMD_CLIENT_MSG_CLASS = "webcmd_client"

# socket message provider ID string, unique
ROS_MSG_BRIDGE_CLASS = "ros_msg_bridge"

# this is webcmd server msg class string
WEBCMD_SERVER_MSG_CLASS = "ros_webcmd_server"

# cmd definition
CMD_SWITCH_ROSSERVICE = "switch_ros_service"
RES_SWITCH_ROSSERVICE = "switch_ros_service_response"
CMD_RESTART_TO_ROSSERVICE = "restart_to_ros_service"
RES_RESTART_TO_ROSSERVICE = "restart_to_ros_service_response"
CMD_START_EXPLORERMAP = "start_explorer_map"
RES_START_EXPLORERMAP = "start_explorer_map_response"
CMD_STOP_EXPLORERMAP = "stop_explorer_map"
RES_STOP_EXPLORERMAP = "stop_explorer_map_response"
CMD_RESTART_EXPLORERMAP = "restart_explorer_map"
RES_RESTART_EXPLORERMAP = "restart_explorer_map_response"
CMD_SAVE_MAP = "save_map"
RES_SAVE_MAP = "save_map_response"

# service name string
ROS_SERVICE_UNKOWN = "unkown"
ROS_SERVICE_NAVIGATION = "navigation"
ROS_SERVICE_EXPLORER = "explorer"

# status string
STATUS_EXPLORER_MAP_UNINIT = "explorer_map_uninit"
STATUS_EXPLORER_MAP_COMPLETED = "explorer_map_completed"
STATUS_EXPLORER_MAP_STARTUP = "explorer_map_startup"
STATUS_EXPLORER_MAP_DOING = "explorer_map_doing"

# operation return code of response message
CMD_PROCESS_SUCCESS = 0
ERROR_CODE_NO_MAP = 1
ERROR_CODE_NO_MAP_INFO = "Error: there is no map! can't start navigation."
ERROR_CODE_UNKOWN_SERVICE = 2
ERROR_CODE_UNKOWN_SERVICE_INFO = "Error: switch to unkown ros service!"
ERROR_CODE_EXPLORER_SERVICE_NOTRUNNING = 3
ERROR_CODE_EXPLORER_SERVICE_NOTRUNNING_INFO = "ros explorer service is not running! explorering map needs it!"
ERROR_CODE_SAVE_MAP_EXPLORER_INCOMPLETED = 4
ERROR_CODE_SAVE_MAP_EXPLORER_INCOMPLETED_INFO = "ros explorer map process is not completed! need wait..."

# current ros service type flag
ROS_NOSERVICE_FLAG = 0
ROS_NAVIGATION_FLAG = 1
ROS_EXPLORERMAP_FLAG = 2

# when do explorering map, the status of explorer
STATUS_EXPLORER_UNINIT = 0
STATUS_WAIT_EXPLORER_STARTUP = 1
STATUS_EXPLORERING_MAP = 2
STATUS_EXPLORER_COMPLETED = 3

EXPLORER_STARTUP_TIMEOUT = 20 # need wait 20 seconds before tell the server explorer is started
EXPLORER_COMPLETED_TIMEOUT = 20 # if there is no msg coming from explorer node in EXPLORER_COMPLETED_TIMEOUT seconds, means explorer is completed.

# current ros service subprocess working mode
WORKMODE_NO_ROSSERVICE = 0 # IDEL no ros service running
WORKMODE_CHECK_ROSSERVICE = 1  # check current ros service is stopped or not

CMDLINE_ROS_MASTER = "roscore" # run once and keep running forever
CMDLINE_CHECK_ROS_MASTER_RUNNING = "rostopic list" # use this cmd line output string, search "rosout_agg" to ensure ros master running
CHECK_ROS_MASTER_RUNNING_KEYSTR = "rosout_agg"

CMDLINE_ROS_SAVEMAP = "rosrun map_server map_saver -f " # file will be saved in current directory: new_map.pgm new_map.yaml
CMDLINE_ROS_EXPLORER_LITE = "roslaunch robot_navigation explore.launch"

CMDLINE_ROS_NAVIGATION_BASE = "roslaunch robot_navigation robot_nav_web.launch"
CMDLINE_ROS_EXPLORERMAP_BASE = "roslaunch robot_navigation robot_slam_laser_web.launch planner:=teb"

CMDLINE_ROS_NAVIGATION_SIMULATION = " simulation:=true host_type:=pc"
CMDLINE_ROS_EXPLORERMAP_SIMULATION = " simulation:=true"
CMDLINE_OPEN_RVIZ = " open_rviz:=true"

WORKSPACE_DIR = "/home/bingda/workspace/"
NAV_MAP_DIR = "/home/bingda/catkin_ws/src/robot_navigation/maps/"
NAV_MAP_NAME = "elon-home" # elon-home.pgm elon-home.yaml

# head has 8 bytes, 4 magic number and 4 bytes length
FRAME_HEADER_LEN = 8


def is_map_ready():
    map_file = NAV_MAP_DIR + NAV_MAP_NAME + ".yaml"
    return os.path.exists(map_file)

def run_ros_service(command):
    process = subprocess.Popen(shlex.split(command)) #, stdout=subprocess.PIPE)
    return process

def isExisted_ros_master():
    #parameters: bufsize = 1   can be used when command line dosn't flush the stdout channel
    process = subprocess.Popen(CMDLINE_CHECK_ROS_MASTER_RUNNING, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    while True:
        line = process.stdout.readline()
        #line = process.stdout.read()
        if line:
            # print(line)
            if line.decode().find(CHECK_ROS_MASTER_RUNNING_KEYSTR) >= 0:
                print("find keystr, ros master running!")
                return True
        else:
            print("ros master not found!")
            return False

def wait_ros_master():
    #parameters: bufsize = 1   can be used when command line dosn't flush the stdout channel
    while True:
        process = subprocess.Popen(CMDLINE_CHECK_ROS_MASTER_RUNNING, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

        while True:
            line = process.stdout.readline()
            #line = process.stdout.read()
            if line:
                # print(line)
                if line.decode().find(CHECK_ROS_MASTER_RUNNING_KEYSTR) >= 0:
                    print("find keystr, ros master running!")
                    return process
            else:
                time.sleep(3)
                break

def stop_ros_service(process):
    process.terminate()

def kill_ros_service(process):
    process.kill()

def isExit_ros_service(process):
    return process.poll() is not None

def exit_ros_service(process):
    stop_ros_service(process)

    wait_exit_time = 0
    while not isExit_ros_service(process):
        time.sleep(1)
        wait_exit_time += 1
        if wait_exit_time > 10:
            kill_ros_service(process)
            break

    while not isExit_ros_service(process):
        time.sleep(1)

    print("ros services exited!")

# call a ros command and wait it to exit
def call_ros_service(command):
    process = run_ros_service(command)
    while not isExit_ros_service(process):
        time.sleep(1)

def explorer_frontiers_callback(frontiers):
    global curExplorerStatus
    global waitExplorerTime

    # there is no useful value in frontiers, we just know if there is msg coming, that means explorer is running
    # if there is no message coming in 
    if curExplorerStatus == STATUS_WAIT_EXPLORER_STARTUP:
        curExplorerStatus = STATUS_EXPLORERING_MAP
        waitExplorerTime = 0
    elif curExplorerStatus == STATUS_EXPLORERING_MAP:
        # clear timeout counter
        waitExplorerTime = 0

async def send_webcmd_msg(websocket, msgData):
    msgData["msg_class"] = WEBCMD_CLIENT_MSG_CLASS
    msg_text = json.dumps(msgData)
    await websocket.send(msg_text)

async def client_msg_process(client_socket):
    global webcmd
    global isConnected
    global isExitSignal
    global curRosService
    global curWorkMode
    global curExplorerStatus
    global ros_process
    global explorer_process

    async for recv_text in client_socket:
        if isExitSignal:
            return

        # print("recv_text:%s" %(recv_text))

        jsonMsg = json.loads(recv_text)
        resData = {}

        # print("cmd:%s" %(jsonMsg['ros_web_cmd']))

        if jsonMsg['msg_class'] == WEBCMD_SERVER_MSG_CLASS :
            if jsonMsg['ros_web_cmd'] == "role_accepted" :
                print("indicate robot role accepted!")

        elif jsonMsg['msg_class'] == WEBCMD_CLIENT_MSG_CLASS :
            if jsonMsg['ros_web_cmd'] == CMD_SWITCH_ROSSERVICE:
                resData['ros_web_cmd'] = RES_SWITCH_ROSSERVICE
                if jsonMsg['service'] == ROS_SERVICE_NAVIGATION:
                    if is_map_ready():
                        if curRosService != ROS_NAVIGATION_FLAG or curWorkMode == WORKMODE_NO_ROSSERVICE:
                            print("Switch to navigation ros service...")
                            exit_ros_service(ros_process)

                            if curExplorerStatus != STATUS_EXPLORER_UNINIT:
                                print("stop explorer map ros service...")
                                exit_ros_service(explorer_process)
                                curExplorerStatus = STATUS_EXPLORER_UNINIT

                            ros_process = run_ros_service(CMDLINE_ROS_NAVIGATION)
                            curRosService = ROS_NAVIGATION_FLAG
                            curWorkMode = WORKMODE_CHECK_ROSSERVICE

                        resData['service'] = ROS_SERVICE_NAVIGATION
                        resData['ret_code'] = CMD_PROCESS_SUCCESS
                    else:
                        resData['service'] = ROS_SERVICE_NAVIGATION
                        resData['ret_code'] = ERROR_CODE_NO_MAP
                        resData['error'] = ERROR_CODE_NO_MAP_INFO

                elif jsonMsg['service'] == ROS_SERVICE_EXPLORER:
                    if curRosService != ROS_EXPLORERMAP_FLAG or curWorkMode == WORKMODE_NO_ROSSERVICE:
                        print("Switch to explorer map ros service...")
                        exit_ros_service(ros_process)

                        ros_process = run_ros_service(CMDLINE_ROS_EXPLORERMAP)
                        curRosService = ROS_EXPLORERMAP_FLAG
                        curWorkMode = WORKMODE_CHECK_ROSSERVICE

                    resData['service'] = ROS_SERVICE_EXPLORER
                    resData['ret_code'] = CMD_PROCESS_SUCCESS
                else:
                    print("Error: switch to unkown ros service!")
                    resData['service'] = ROS_SERVICE_UNKOWN
                    resData['ret_code'] = ERROR_CODE_UNKOWN_SERVICE
                    resData['error'] = ERROR_CODE_UNKOWN_SERVICE_INFO
                
                try:
                    await send_webcmd_msg(client_socket, resData)
                except Exception as e:
                    print("Exception client_msg_process:", e)
                    return
            
            elif jsonMsg['ros_web_cmd'] == CMD_RESTART_TO_ROSSERVICE:
                resData['ros_web_cmd'] = RES_RESTART_TO_ROSSERVICE

                # force to exit all service
                exit_ros_service(ros_process)
                if curExplorerStatus != STATUS_EXPLORER_UNINIT:
                    exit_ros_service(explorer_process)
                    curExplorerStatus = STATUS_EXPLORER_UNINIT
                curWorkMode = WORKMODE_NO_ROSSERVICE
                curRosService = ROS_NOSERVICE_FLAG

                if jsonMsg['service'] == ROS_SERVICE_NAVIGATION:
                    if is_map_ready():
                        ros_process = run_ros_service(CMDLINE_ROS_NAVIGATION)
                        curRosService = ROS_NAVIGATION_FLAG
                        curWorkMode = WORKMODE_CHECK_ROSSERVICE

                        resData['service'] = ROS_SERVICE_NAVIGATION
                        resData['ret_code'] = CMD_PROCESS_SUCCESS
                    else:
                        resData['service'] = ROS_SERVICE_NAVIGATION
                        resData['ret_code'] = ERROR_CODE_NO_MAP
                        resData['error'] = ERROR_CODE_NO_MAP_INFO

                        # start explorer service as default
                        ros_process = run_ros_service(CMDLINE_ROS_EXPLORERMAP)
                        curRosService = ROS_EXPLORERMAP_FLAG
                        curWorkMode = WORKMODE_CHECK_ROSSERVICE

                elif jsonMsg['service'] == ROS_SERVICE_EXPLORER:
                    ros_process = run_ros_service(CMDLINE_ROS_EXPLORERMAP)
                    curRosService = ROS_EXPLORERMAP_FLAG
                    curWorkMode = WORKMODE_CHECK_ROSSERVICE

                    resData['service'] = ROS_SERVICE_EXPLORER
                    resData['ret_code'] = CMD_PROCESS_SUCCESS
                else:
                    print("Error: switch to unkown ros service!")
                    resData['service'] = ROS_SERVICE_UNKOWN
                    resData['ret_code'] = ERROR_CODE_UNKOWN_SERVICE
                    resData['error'] = ERROR_CODE_UNKOWN_SERVICE_INFO

                    # start explorer service as default
                    ros_process = run_ros_service(CMDLINE_ROS_EXPLORERMAP)
                    curRosService = ROS_EXPLORERMAP_FLAG
                    curWorkMode = WORKMODE_CHECK_ROSSERVICE
                
                try:
                    await send_webcmd_msg(client_socket, resData)
                except Exception as e:
                    print("Exception client_msg_process:", e)
                    return

            elif jsonMsg['ros_web_cmd'] == CMD_START_EXPLORERMAP:
                resData['ros_web_cmd'] = RES_START_EXPLORERMAP
                if curRosService == ROS_EXPLORERMAP_FLAG and curWorkMode == WORKMODE_CHECK_ROSSERVICE:
                    resData['ret_code'] = CMD_PROCESS_SUCCESS

                    if curExplorerStatus == STATUS_EXPLORER_UNINIT:
                        explorer_process = run_ros_service(CMDLINE_ROS_EXPLORER_LITE)
                        curExplorerStatus = STATUS_WAIT_EXPLORER_STARTUP
                        waitExplorerTime = 0
                else:
                    resData['ret_code'] = ERROR_CODE_EXPLORER_SERVICE_NOTRUNNING
                    resData['error'] = ERROR_CODE_EXPLORER_SERVICE_NOTRUNNING_INFO
                
                try:
                    await send_webcmd_msg(client_socket, resData)
                except Exception as e:
                    print("Exception client_msg_process:", e)
                    return
            
            elif jsonMsg['ros_web_cmd'] == CMD_STOP_EXPLORERMAP:
                resData['ros_web_cmd'] = RES_STOP_EXPLORERMAP
                resData['ret_code'] = CMD_PROCESS_SUCCESS # always success for stop explorer map
                if curExplorerStatus == STATUS_WAIT_EXPLORER_STARTUP or curExplorerStatus == STATUS_EXPLORERING_MAP:
                    exit_ros_service(explorer_process)
                    curExplorerStatus = STATUS_EXPLORER_UNINIT
                
                try:
                    await send_webcmd_msg(client_socket, resData)
                except Exception as e:
                    print("Exception client_msg_process:", e)
                    return

            elif jsonMsg['ros_web_cmd'] == CMD_RESTART_EXPLORERMAP:
                resData['ros_web_cmd'] = RES_RESTART_EXPLORERMAP
                if curRosService == ROS_EXPLORERMAP_FLAG and curWorkMode == WORKMODE_CHECK_ROSSERVICE:
                    resData['ret_code'] = CMD_PROCESS_SUCCESS

                    if curExplorerStatus != STATUS_EXPLORER_UNINIT:
                        exit_ros_service(explorer_process)
                        curExplorerStatus = STATUS_EXPLORER_UNINIT
                    
                    explorer_process = run_ros_service(CMDLINE_ROS_EXPLORER_LITE)
                    curExplorerStatus = STATUS_WAIT_EXPLORER_STARTUP
                    waitExplorerTime = 0
                else:
                    resData['ret_code'] = ERROR_CODE_EXPLORER_SERVICE_NOTRUNNING
                    resData['error'] = ERROR_CODE_EXPLORER_SERVICE_NOTRUNNING_INFO
                
                try:
                    await send_webcmd_msg(client_socket, resData)
                except Exception as e:
                    print("Exception client_msg_process:", e)
                    return

            elif jsonMsg['ros_web_cmd'] == CMD_SAVE_MAP:
                resData['ros_web_cmd'] = RES_SAVE_MAP
                if curRosService == ROS_EXPLORERMAP_FLAG and curWorkMode == WORKMODE_CHECK_ROSSERVICE:
                    # we don't check the explorer map is completed or not, just save the map.
                    # then this can support user use joystick to explorer map and even using explorer-lite and not completed, we still can save current map!
                    resData['ret_code'] = CMD_PROCESS_SUCCESS

                    cmdline = CMDLINE_ROS_SAVEMAP + WORKSPACE_DIR + NAV_MAP_NAME
                    call_ros_service(cmdline)
                    cmdline = "cp {0}{1}.pgm {2}{3}.pgm -f".format(WORKSPACE_DIR, NAV_MAP_NAME, NAV_MAP_DIR, NAV_MAP_NAME)
                    call_ros_service(cmdline)
                    cmdline = "cp {0}{1}.yaml {2}{3}.yaml -f".format(WORKSPACE_DIR, NAV_MAP_NAME, NAV_MAP_DIR, NAV_MAP_NAME)
                    call_ros_service(cmdline)
                else:
                    resData['ret_code'] = ERROR_CODE_EXPLORER_SERVICE_NOTRUNNING
                    resData['error'] = ERROR_CODE_EXPLORER_SERVICE_NOTRUNNING_INFO
                
                try:
                    await send_webcmd_msg(client_socket, resData)
                except Exception as e:
                    print("Exception client_msg_process:", e)
                    return

        elif jsonMsg['msg_class'] == ROS_MSG_BRIDGE_CLASS :
            # just foward all message with type ROS_MSG_BRIDGE_CLASS to tcp socket
            await forwardToRosMsgBridge(recv_text)
        else :
            print("unkown msg: %s" %(recv_text))
            resData['ros_web_cmd'] = "error"
            resData['content'] = f'unkown msg:{recv_text}'

            try:
                await send_webcmd_msg(client_socket, resData)
            except Exception as e:
                print("Exception client_msg_process:", e)
                return

async def setup_websocket_conn(wsurl):
    global isConnected
    global isExitSignal
    global client_websocket
    global websocket_recv_task

    loop = asyncio.get_event_loop()

    while not isExitSignal:
        try:
            async with websockets.connect(wsurl) as websocket:
                print("server connected!")

                # send a cmd to indicate I'm ROBOT
                cmdmsg = '{"ros_web_cmd":"role", "msg_class":"'+ WEBCMD_CLIENT_MSG_CLASS +'", "type":"robot", "subtype":"ackermann_car", "name":"' + ROBOT_NAME + '","uid":"'+ ROBOT_ID +'"}'
                await websocket.send(cmdmsg)

                isConnected = True
                client_websocket = websocket

                # client websocket receive msg process task
                websocket_recv_task = loop.create_task(client_msg_process(websocket))
                await websocket_recv_task
        except websockets.ConnectionClosed:
            print("ConnectionClosed...")    # 链接断开
        except websockets.InvalidState:
            print("InvalidState...")    # 无效状态
        except Exception as e:
            print("Exception:", e)
        except KeyboardInterrupt:
            isExitSignal = True
            print("Shutting down webcmd_forward node.")

        isConnected = False
        print("server disconnected!")
        await asyncio.sleep(5)
    
    print("setup_websocket_conn exited!")

def explorer_frontiers_inform():
    global curExplorerStatus
    global waitExplorerTime

    # there is no useful value in frontiers, we just know if there is msg coming, that means explorer is running
    # if there is no message coming in 
    if curExplorerStatus == STATUS_WAIT_EXPLORER_STARTUP:
        curExplorerStatus = STATUS_EXPLORERING_MAP
        waitExplorerTime = 0
    elif curExplorerStatus == STATUS_EXPLORERING_MAP:
        # clear timeout counter
        waitExplorerTime = 0

async def submit_ros_status():
    global isConnected
    global isExitSignal
    global client_websocket
    global waitExplorerTime
    global curWorkMode
    global curRosService
    global curExplorerStatus

    print("submit_ros_status task start run...")

    while not isExitSignal:
        resData = {}
        resData['ros_web_cmd'] = "ros_service_status"

        if curWorkMode == WORKMODE_CHECK_ROSSERVICE:
            if curRosService == ROS_NAVIGATION_FLAG:
                resData['service'] = ROS_SERVICE_NAVIGATION
                resData['explorer_map_status'] = STATUS_EXPLORER_MAP_UNINIT
            elif curRosService == ROS_EXPLORERMAP_FLAG:
                resData['service'] = ROS_SERVICE_EXPLORER

                if curExplorerStatus == STATUS_WAIT_EXPLORER_STARTUP:
                    # wait some time ... then switch to STATUS_EXPLORERING_MAP
                    waitExplorerTime += 1

                    if waitExplorerTime > EXPLORER_STARTUP_TIMEOUT:
                        curExplorerStatus = STATUS_EXPLORERING_MAP
                        waitExplorerTime = 0
                    
                    resData['explorer_map_status'] = STATUS_EXPLORER_MAP_STARTUP
                        
                elif curExplorerStatus == STATUS_EXPLORERING_MAP:
                    # check the explorer status according to the msg from ros topic 
                    waitExplorerTime += 1

                    if waitExplorerTime > EXPLORER_COMPLETED_TIMEOUT:
                        curExplorerStatus = STATUS_EXPLORER_COMPLETED
                        waitExplorerTime = 0
                    
                    resData['explorer_map_status'] = STATUS_EXPLORER_MAP_DOING

                elif curExplorerStatus == STATUS_EXPLORER_COMPLETED:
                    # send msg to server to inform exploer map done!
                    resData['explorer_map_status'] = STATUS_EXPLORER_MAP_COMPLETED
                else :
                    resData['explorer_map_status'] = STATUS_EXPLORER_MAP_UNINIT
            else:
                print("Error: Unkown ROS service Flag!")
                resData['service'] = ROS_SERVICE_UNKOWN
                resData['explorer_map_status'] = STATUS_EXPLORER_MAP_UNINIT
        else:
            resData['service'] = ROS_SERVICE_UNKOWN
            resData['explorer_map_status'] = STATUS_EXPLORER_MAP_UNINIT
        
        if is_map_ready():
            resData['is_map_ready'] = True
        else:
            resData['is_map_ready'] = False

        print(resData)

        try:
            if isConnected:
                await send_webcmd_msg(client_websocket, resData)
            else:
                print("not connected! send ros status failed")
        except Exception as e:
            print("Exception submit_ros_status:", e)

        await asyncio.sleep(2)
    
    print("submit_ros_status task Exited!")

async def forwardToRosMsgBridge(msg):
    global isRosMsgBridgeConnected
    global rosmsg_bridge_sender

    if not isRosMsgBridgeConnected:
        return None
    
    data = msg.encode()

    # print("data length = %d" %(len(data)))

    # magic number : 0523 as header flag
    head_data = bytearray(4)
    head_data[0] = 0
    head_data[1] = 5
    head_data[2] = 2
    head_data[3] = 3

    # real data length
    length_data = bytearray(4)
    length_data[0] = ((len(data) & int('0xff000000', 16)) >> 24)
    length_data[1] = ((len(data) & int('0x00ff0000', 16)) >> 16)
    length_data[2] = ((len(data) & int('0x0000ff00', 16)) >> 8)
    length_data[3] = ((len(data) & int('0x000000ff', 16)))

    bytes_header_data = bytes(head_data + length_data)

    try:
        rosmsg_bridge_sender.write(bytes_header_data)
        await rosmsg_bridge_sender.drain()
    except Exception as e:
        print("Exception forwardToRosMsgBridge:", e)

    try:
        rosmsg_bridge_sender.write(data)
        await rosmsg_bridge_sender.drain()
    except Exception as e:
        print("Exception forwardToRosMsgBridge:", e)
    
    return msg

async def cmd_process(bytes_data, isBinaryData, isInternalMsg) :
    global isConnected
    global client_websocket

    try: 
        if isBinaryData:
            if isConnected:
                await client_websocket.send(bytes_data)
        else :
            recv_text = bytes_data.decode()

            if isInternalMsg:
                # print(recv_text)
                jsonMsg = json.loads(recv_text)
                if jsonMsg['ros_web_cmd'] == "explorer_frontiers":
                    explorer_frontiers_inform()
            else:
                if isConnected:
                    # check connection of websocket then send msg by websocket
                    await client_websocket.send(recv_text)
    except Exception as e:
        print("Exception forward msg to websockets:", e)

async def thread_tcp_rosmsg_bridge(ip, port):
    global isExitSignal
    global rosmsg_bridge_sender
    global isConnected
    global isRosMsgBridgeConnected

    rosmsg_bridge_addr = (ip, port)

    local_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # local_socket.bind(local_addr)
    local_socket.settimeout(1)

    while not isExitSignal:
        try:
            reader, writer = await asyncio.open_connection(ip, port)
            rosmsg_bridge_sender = writer
            isRosMsgBridgeConnected = True
        except Exception as e:
            print("Exception open_connection:", e)
            await asyncio.sleep(5)
            print("try connect to ros msg bridge after 5 seconds...")
            continue

        print("ros message bridge connected!")

        check_len = 0
        total_len = 0
        isBinaryData = False # there tow kinds of data from ros msg bridge, string and bytes
        isInternalMsg = False # some message is used internally, don't need forward them to webcmd server
        read_request_len = FRAME_HEADER_LEN # always read 8 bytes header at first!

        while not isExitSignal:
            try:
                data = await reader.readexactly(read_request_len)
                # print("%d bytes received" %(len(data)))
            except Exception as e:
                print("Exception rosmsg_bridge message process:", e)
                break

            total_len += len(data)

            if total_len == FRAME_HEADER_LEN and check_len == 0:
                bytes_data = bytearray(data)
                # print("header=%d:%d:%d:%d" %(bytes_data[0], bytes_data[1], bytes_data[2], bytes_data[3]))
                if bytes_data[0] == 0 and bytes_data[1] == 5 and bytes_data[2] == 2 :
                    if bytes_data[3] == 0:
                        isBinaryData = True
                    else:
                        isBinaryData = False
                        if bytes_data[3] == 1:
                            isInternalMsg = True
                        elif bytes_data[3] == 3:
                            isInternalMsg = False
                        else:
                            print("Fatal Error: wrong data frame head magic number!")
                            break
                    
                    # print("Got a right head flag 0523 !")
                    check_len = (int(bytes_data[4]) << 24) + (int(bytes_data[5]) << 16) + (int(bytes_data[6]) << 8) + int(bytes_data[7])
                    # print("check len=%d" %(check_len))

                    # then read all datas at next time!
                    read_request_len = check_len
                    continue
                else:
                    print("Fatal Error: wrong data frame head magic number!")
                    break
            
            elif total_len == check_len+FRAME_HEADER_LEN :
                # print("get a completed data frame=%d" %(total_len-8))
                await cmd_process(data, isBinaryData, isInternalMsg)

                check_len = 0
                total_len = 0
                isBinaryData = False
                read_request_len = FRAME_HEADER_LEN
            else :
                print("Fatal Error: total_len=%d not equal to (check_len+8)=%d" %(total_len, check_len+FRAME_HEADER_LEN))
                break
        
        isRosMsgBridgeConnected = False
        writer.close()
        # await writer.wait_closed()
        print("ros message bridge disconnected!")

    print("thread_tcp_rosmsg_bridge exited!")

def exit_app(signum, frame):
    global isExitSignal
    global loop
    global isConnected
    global websocket_recv_task
    global websocket_conn_task
    global rosmsg_bridge_task
    global submit_ros_status_task

    print("get exit_app signal on webcmd_forward...")
    isExitSignal = True

    if isConnected:
        websocket_recv_task.cancel()

    websocket_conn_task.cancel()
    rosmsg_bridge_task.cancel()
    submit_ros_status_task.cancel()

    # loop.stop()
    loop.call_soon_threadsafe(loop.stop)

def main(argv):
    global simulation_mode
    global open_rviz
    global ROBOT_NAME
    global ROBOT_ID
    global loop
    global websocket_recv_task
    global websocket_conn_task
    global rosmsg_bridge_task
    global submit_ros_status_task
    global curRosService
    global curWorkMode
    global curExplorerStatus
    global explorer_process
    global ros_process
    global ros_master_process
    global CMDLINE_ROS_NAVIGATION
    global CMDLINE_ROS_EXPLORERMAP
    global CMDLINE_ROS_EXPLORER_LITE
    global waitExplorerTime

    # robot name and description info
    ROBOT_NAME = "Clement Ackermann Car"
    ROBOT_ID = "8ea6b398-0732-42cb-9fa5-c30bab75cc68"
    # simulation or open rviz flag
    simulation = 'false'
    rviz = 'false'
    # webcmd server address
    ws_url = "ws://clement.server.ros:9523"
    # internal ros msg bridge server
    rosmsg_bridge_server_address = "127.0.0.1"
    rosmsg_bridge_server_port = 8001

    try:
        opts, args = getopt.getopt(argv,"h:s:r:n:i:w:p:",["simulation=","rviz=","name=","id=","ws=","port="])
    except getopt.GetoptError:
        print ('ros_service_mgr.py -s true -r false -n Ackermann -i 8ea6b398-0732-42cb-9fa5-c30bab75cc68')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print ('ros_service_mgr.py -s <simulation> -r <rviz> -n <robotname> -i <robotid>')
            sys.exit()
        elif opt in ("-s", "--simulation"):
            simulation = arg
        elif opt in ("-r", "--rviz"):
            rviz = arg
        elif opt in ("-n", "--name"):
            ROBOT_NAME = arg
        elif opt in ("-i", "--id"):
            ROBOT_ID = arg
        elif opt in ("-w", "--ws"):
            ws_url = arg
        elif opt in ("-p", "--port"):
            rosmsg_bridge_server_port = int(arg)
    
    print("simulation=%s rviz=%s name=%s id=%s wsurl=%s msgport=%d" %(simulation, rviz, ROBOT_NAME, ROBOT_ID, ws_url, rosmsg_bridge_server_port))

    if simulation.find("true") >= 0:
        simulation_mode = True
    
    if rviz.find("true") >= 0:
        open_rviz = True

    # check ros master is existed or not and run it
    isNeedRunRosMaster = not isExisted_ros_master()
    if isNeedRunRosMaster:
        ros_master_process = run_ros_service(CMDLINE_ROS_MASTER)

    print("wait for ROS MASTER to start completedly... ")
    wait_ros_master()

    CMDLINE_ROS_NAVIGATION = CMDLINE_ROS_NAVIGATION_BASE
    CMDLINE_ROS_EXPLORERMAP = CMDLINE_ROS_EXPLORERMAP_BASE
    if simulation_mode:
        CMDLINE_ROS_NAVIGATION += CMDLINE_ROS_NAVIGATION_SIMULATION
        CMDLINE_ROS_EXPLORERMAP += CMDLINE_ROS_EXPLORERMAP_SIMULATION
    else:
        # when using real robot car, there is no base_link, need use base_footprint
        CMDLINE_ROS_EXPLORER_LITE += " robot_base_frame:=base_footprint"

    if open_rviz:
        CMDLINE_ROS_NAVIGATION += CMDLINE_OPEN_RVIZ
        CMDLINE_ROS_EXPLORERMAP += CMDLINE_OPEN_RVIZ

    if is_map_ready():
        print("map is ready, startup to navigation...")

        curRosService = ROS_NAVIGATION_FLAG
        ros_process = run_ros_service(CMDLINE_ROS_NAVIGATION)
        curWorkMode = WORKMODE_CHECK_ROSSERVICE

        curExplorerStatus = STATUS_EXPLORER_UNINIT
        # when curExplorerStatus is STATUS_EXPLORER_UNINIT, explorer_subprocess can not be used!!!
    else:
        print("map is not ready! startup to explorer...")

        curRosService = ROS_EXPLORERMAP_FLAG
        ros_process = run_ros_service(CMDLINE_ROS_EXPLORERMAP)
        curWorkMode = WORKMODE_CHECK_ROSSERVICE

        curExplorerStatus = STATUS_EXPLORER_UNINIT
    
    waitExplorerTime = 0
    
    loop = asyncio.get_event_loop()
    #loop.run_until_complete(setup_websocket_conn(ws_url))  # can not use loop.stop !!!
    websocket_conn_task = loop.create_task(setup_websocket_conn(ws_url)) # can use loop.stop
    rosmsg_bridge_task = loop.create_task(thread_tcp_rosmsg_bridge(rosmsg_bridge_server_address, rosmsg_bridge_server_port))
    submit_ros_status_task = loop.create_task(submit_ros_status())

    try:
        loop.run_forever()
    except Exception as e:
        print("Exception loop:", e)
    # loop.close()

    if curExplorerStatus != STATUS_EXPLORER_UNINIT:
        print("wait for explorer map service to exit...")
        exit_ros_service(explorer_process)
    
    print("wait for ros service to exit...")
    exit_ros_service(ros_process)
    
    if isNeedRunRosMaster :
        print("wait for ros master to exit...")
        exit_ros_service(ros_master_process)
    
    print("Webcmd Client Exited!")

if __name__ == '__main__':
    signal.signal(signal.SIGINT, exit_app)
    signal.signal(signal.SIGTERM, exit_app)
    signal.signal(signal.SIGHUP, exit_app)

    main(sys.argv[1:])
