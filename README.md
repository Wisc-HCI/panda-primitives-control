# PandaFCI
Welcome to PandaFCI! This is a set of interfaces for controlling the Panda robot in real time through Franka Control Interface. It allows both real-time control (1kHz) and ros support (in cartesian space and joint space, both in position and velocity).

## 1. Prequisites

Here is what you need to start with:
* Ubuntu Machine with the [Real Time Kernel](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel)
	* Static IP of 192.168.1.XXX (Ex/ 192.168.1.5) and Netmask of 255.255.255.0 for the ethernet connected to the Panda. If you have also have a force torque sensor, the ethernet connected to that needs to be set to a static IP of 192.168.2.XXX (Ex/ 192.168.2.5).
	* [Docker Engine](https://docs.docker.com/engine/install/)
* Franka Emika Panda 7 DOF Robot setup with the [FCI](https://frankaemika.github.io/docs/getting_started.html) and set to static IP of 192.168.1.XXX (Ex/ 192.168.1.3) and Netmask to 255.255.255.0.
	* Robot system version: 4.2.X (FER pandas)
	* Robot / Gripper Server version: 5 / 3


You will optionally need:
* [SpaceMouse Compact or Wireless](https://3dconnexion.com/us/product/spacemouse-compact/)
* [Axio80-M20 Force Torque Sensor](https://www.ati-ia.com/products/ft/ft_models.aspx?id=Axia80-M20) installed on the Panda's End Effector and connected to the host computer via ethernet with IP 192.168.2.2 (or change the IP in src/PandaController/src/ForceTorqueListener.cpp).


Here is what we are going to install:
* ROS Noetic
* Libfranka  version 0.9.2
* franka_ros version 0.10.0
* Various apt/ROS packages 


## 2. Setting up this repository
First clone this repository to your local computer and open a terminal in the repository's directory. All further commands should be run from there.
Run the following command to grab the proper submodules:
```bash
git submodule init
git submodule update
```

The following submodules should have been cloned:
- [spacenavd v1.3](https://github.com/FreeSpacenav/spacenavd)

## 3. Setting Up Your Container

First set up display forwarding:
```bash
xhost +local:
```
Now  build the container image and start the container. Make sure you are in this root directory (NIST_Benchmark). These commands mount on the current directory as the containers file system so any changes you make to the files on your host machine will be mirrored in the container. These commands also allow the containers display to be forwarded to your host machine so that you can see it.
```bash
sudo docker build -t panda-prim-controller .

# Start the container with real-time kernel privileges, allow access to usb devices,  mount onto the current directory, and allow display forwarding. Container is removed once it exits.
sudo docker run --rm -it --privileged --device=/dev/input/event* --cap-add=SYS_NICE --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --net=host panda-prim-controller
```

Add necessary environment variables:
Replace with your Panda's IP 
```bash
export PANDA_IP=192.168.1.3
```

## 4. Compilation:

### Compile non-ROS package (PandaController)

If first time, first configure:
```bash
cd src/PandaController
mkdir build
cd build
cmake ..
cd ../../..
```

Anytime, run:
```bash
cd src/PandaController/build
make install
cd ../../..
```

### Compile ROS package
Compile individually each ros packages:
```bash
# catkin build relaxed_ik --no-notify
catkin build panda_ros_msgs --no-notify
catkin build panda_ros --no-notify
#catkin build dmp_deformations --no-notify
catkin build inputs_ros --no-notify
catkin build controller --no-notify
```


## 5. Running with ROS
1. Use Franka Desktop to unlock the Panda's joints and enable FCI mode.
2. Run `source devel/setup.bash` inside the root directory
3. Start the launch files related to the application:
	* Space mouse input:
	    - Terminal 1: 
			```bash
			spacenavd
			roslaunch inputs_ros space_mouse.launch
			```
	* Controller test:
		- Terminal 1:
			```bash
			roslaunch controller mover_test.launch
			```
		- Terminal 2:	
			```bash
			rosrun controller controller_tester.py
			```


## Updating to Noetic/Python3 Progress
- [x] Update Dockerfile to Noetic and Python3
- [x] Make sure libraries in Dockerfile still work or upgrade as needed.
- [x] Update libfranka to 0.9.2 (This can be done in DockerFile with apt-get).
- [x] Update franka_ros to 0.10.0 (This version can be done in the DockerFile through apt-get. This version should be compatible but I've only tested 0.8.0 before. If we end up  needing 0.8.0, that will need to be downloaded through source code.)
- [x] Fix any CMAKE issues.
- [X] Check if changes in changelog of libfranka and franka_ros show effect code.
- [X] Update python code necessary to get Space Mouse working.
- [ ] Get Force/Torque Sensor working
- [ ] Upgrade RelaxedIK to RangedIK.
- [ ] Figure out which code is relevant to ros_gui and upgrade that.
- [ ] Possibly delete any remaining outdated/unnecessary code.

## Resources
* Noetic/Python3 Migration:
	* https://wiki.ros.org/noetic/Migration
	* https://mil.ufl.edu/docs/software/noetic_migration.html
* SpaceNav ROS:
	* https://github.com/ros-drivers/joystick_drivers/tree/ros1/spacenav_node
* ROS Headers:
	* https://medium.com/@smilesajid14/how-custom-msg-works-in-ros-7d5a14bf5781
* Force Torque Sensor:
	* https://www.ati-ia.com/app_content/Documents/9610-05-Ethernet%20Axia80.pdf
	* https://www.ati-ia.com/products/ft/ft_models.aspx?id=Axia80-M20
* Force Dimension Input:
	* https://www.forcedimension.com/software/sdk



## Troubleshooting/Testing

* To test the space mouse, run each of these commands in their own terminal. The last command will subscribe to the spacemouse topic and the numbers outputted should change as you move the mouse.
	- Terminal 1: 
		```bash
		roscore
		```
	- Terminal 2: 
		```bash
		spacenavd  
		rosrun spacenav_node spacenav_node
		```
	- Terminal 3: 
		```bash
		rostopic echo /spacenav/joy
		```	
		
* To test if the Force/Torque Sensor is properly connected, go to the sensor's IP (ex/ 192.168.1.6) in a webrowser on your computer. If the ATI Configuration page shows up, that means you are properly connected. If not, reference section 4 of the [Sensor Guide](https://www.ati-ia.com/app_content/Documents/9610-05-Ethernet%20Axia80.pdf). Once you can access the ATI site, you can go to Demo and download the Java application to see the values coming from the sensor.





[rosmaster.master][INFO] 2025-01-20 16:22:12,633: publisherUpdate[/panda/hybrid_pose] -> http://hcilab-G7-7590:40957/ []: sec=0.00, ex>
[rosmaster.threadpool][ERROR] 2025-01-20 16:22:12,634: Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rosmaster/threadpool.py", line 218, in run
    result = cmd(*args)
  File "/opt/ros/noetic/lib/python3/dist-packages/rosmaster/master_api.py", line 210, in publisher_update_task
    ret = xmlrpcapi(api).publisherUpdate('/master', topic, pub_uris)
  File "/usr/lib/python3.8/xmlrpc/client.py", line 1109, in __call__
    return self.__send(self.__name, args)
  File "/usr/lib/python3.8/xmlrpc/client.py", line 1450, in __request
    response = self.__transport.request(
  File "/usr/lib/python3.8/xmlrpc/client.py", line 1153, in request
    return self.single_request(host, handler, request_body, verbose)
  File "/usr/lib/python3.8/xmlrpc/client.py", line 1165, in single_request
    http_conn = self.send_request(host, handler, request_body, verbose)
  File "/usr/lib/python3.8/xmlrpc/client.py", line 1278, in send_request
    self.send_content(connection, request_body)
  File "/usr/lib/python3.8/xmlrpc/client.py", line 1308, in send_content
    connection.endheaders(request_body)
  File "/usr/lib/python3.8/http/client.py", line 1251, in endheaders
    self._send_output(message_body, encode_chunked=encode_chunked)
  File "/usr/lib/python3.8/http/client.py", line 1011, in _send_output
    self.send(msg)
  File "/usr/lib/python3.8/http/client.py", line 951, in send
    self.connect()
  File "/usr/lib/python3.8/http/client.py", line 922, in connect
    self.sock = self._create_connection(
  File "/usr/lib/python3.8/socket.py", line 808, in create_connection
    raise err
  File "/usr/lib/python3.8/socket.py", line 796, in create_connection
    sock.connect(sa)
ConnectionRefusedError: [Errno 111] Connection refused