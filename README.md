# PandaFCI
Welcome to PandaFCI! This is a set of interfaces for controlling the Panda robot in real time through Franka Control Interface. It allows both real-time control (1kHz) and ros support (in cartesian space and joint space, both in position and velocity).

## 1. Prequisites

Here is what you need to start with:
* Robot system version: >= 3.0.0, <4.0.0 (FER pandas)
* Robot / Gripper Server version: 3 / 3
* Ubuntu 


Here is what we are going to install:
* ROS Melodic
* Libfranka  version 0.7.0
* Various apt/ROS packages 

## 2. Setting Up Your Container

First set up display forwarding:
```bash
xhost +local:
```
Now  build the container image and start the container. Make sure you are in this root directory (NIST_Benchmark). These commands mount on the current directory as the containers file system so any changes you make to the files on your host machine will be mirrored in the container. These commands also allow the containers display to be forwarded to your host machine so that you can see it.
```bash
sudo docker build -t panda-prim-controller-container .

# Start the container with real-time kernel privileges, mount onto the current directory, and allow display forwarding. Container is removed once it exits.
sudo docker run --rm -it --privileged --cap-add=SYS_NICE --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --net=host panda-prim-controller-container
```

Add necessary environment variables:
Replace with your Panda's IP 
```bash
export PANDA_IP=192.168.1.3
```


## 3. Compilation:

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
catkin build relaxed_ik --no-notify
catkin build panda_ros_msgs --no-notify
catkin build panda_ros --no-notify
#catkin build dmp_deformations --no-notify
catkin build inputs_ros --no-notify
catkin build controller --no-notify
```


## 4. Running with ROS
1. Run `source devel/setup.bash` inside the root directory
2. Start the launch files related to the application:
    * Falcon:
		- Terminal 1: `roslaunch relaxed_ik relaxed_ik_julia.launch`
		- Terminal 2: `roslaunch inputs_ros falcon.launch`
	* Space mouse:
	    - Terminal 1: `roslaunch inputs_ros space_mouse.launch`
