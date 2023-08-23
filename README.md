# webcmdctrl
This package has three parts:
1. /client

This client runs on the robot car.

It can receive the ROS1 service message(NAV, car cmd, status) from rosmsg_bridge service, and forward these message to the server.

The application is only one file: webcmd_client.py

Normally we suggest to setup a system service for this application, to support autorun when robot is startup. 

How to setup system service:

Create service configuration files: rc-robotnav.service and rc-robotnav.local (you can change file name as you wish, but need ensure using same file name)

    Create these two files at right place as below:

        sudo vi /etc/systemd/system/rc-robotnav.service

        sudo vi /etc/rc-robotnav.local

    Using systemctl cmd to enable this service:

        sudo systemctl enable rc-robotnav

    Manually start/stop this service:

        sudo systemctl start rc-robotnav.service

        sudo systemctl stop rc-robotnav.service

    Check the status of this service:

        sudo systemctl status rc-robotnav.service

Before start this client, please check the server's ip address and port inside the code. Change them to your server's parameters.


2. /rosmsg_bridge

This is a ROS1 service, which is used to get the ROS msg(NAV msg, car status, ctrl) and forward the msg over socket(TCP).

Those ROS msg are transferred to client(webcmd_client.py) through the local internal network of robot car.

Attention:

For ROS1 only can use python2, so this service only can use socket without websocket support.

That is why, we need use webcmd_client.py as above to forward ROS msg to server by using websocket.


3. /server

This server is used as a bridge between user and robot on internet.

All the cmd and msg are transferred through the server.

The logic of binding user with robot device is simple, currently there is no auth check or database.

The user who submit the controlling request to the robot at first, will get the access to control the robot.

When user's connection is closed, the robot's status will be free.


This project need to work with a web page , User can control the robot on that web page in browser.
Please take a look at the web page project: 
**[robot-nav-web](https://github.com/netwsdk/robot-nav-web)**

