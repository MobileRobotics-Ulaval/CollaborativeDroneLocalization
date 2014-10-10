Collaborative Drone Localization
==============================
3D localization system in 6 DoF based on 2 onboard cameras and a minimal set of markers in a outdoor environment for the [Parrot Ar-Drones 2.0]. The localization algorithm uses this paper : [6DoF Cooperative Localization for Mutually
Observing Robots](http://www2.ift.ulaval.ca/~pgiguere/papers/isrr2013.pdf). The code is inspired by [RPG Monocular Pose Estimator].

#Installation#
##Material required##
This package require:
 + Two [Parrot Ar-Drones 2.0] with visual marker on either side of camera.
 + An unprotected wifi router that does not require an AC power source.
 + Any Linux compatible gamepad joystick. We use the [F710 Wireless Gamepad] from Logitech. We suggest [remapping] the buttons to the desire configuration that suit your own gamepad.

##Dependencies##
This package require Robotic Operating System (ROS). In order to install ROS follow these instructions on the [ROS website]. The package was only tested ROS Hydro on Ubuntu 12.04 . 

The Collaborative Drone Localization also require OpenCV and Eigen. They can be install by following the instruction on the [OpenCV] and [Eigen] website.

To manualy control the drone with a gamepad we use `joy/joy_node`. It can be install and configure following [these instructions](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick).

The [ardrone_autonomy] node is used as a bridge between the Parrot AR-Drone 2.0 SDK and the ROS environment. It can be install by entering the following command. Assuming your are in the src directory of your catkin [workspace].
```shell
cd ~/catkin_ws/src
git clone https://github.com/AutonomyLab/ardrone_autonomy.git -b hydro-devel
cd ~/catkin_ws
catkin_make
```

##Multiple drone patch##
The Collaborative Drone Localization require running multiple drones on the same ROS environment. The ardrone autonomy node doesn't currently support the control of multiple drones without some hacking:
 * First of the ArDrone SDK doesn't support the binding of multiple to same port. To fix that follow these instructions ([source](http://answers.ros.org/question/61125/control-multiple-ar-drones/)):
    + Open `ardrone_autonomy` folder in your workspace.
    + Unzip the `ARDroneSDK/ardrone-sdk-stripped-X.X.X.tgz`
    + Open width a text editor the `ARDroneLib/VP_SDK/VP_Com/vp_com_socket.c`
    + Around line 90 change the code to this:
```c
case VP_COM_SERVER:
    name.sin_addr.s_addr  = INADDR_ANY;
    
    int bind_err = bind( s, (struct sockaddr*)&name, sizeof(struct sockaddr));
    if (bind_err < 0 ){
	//res = VP_COM_ERROR;
	res = VP_COM_OK;
    }
```
 + Change the old `vp_com_socket.c` for the new one in your `ardrone-sdk-stripped-X.X.X.tgz`.
 + Recompile the SDK:

```Shell
catkin_make clean
catkin_make install
```
 * Your ardrone driver should now support multiple drone. In order to control two drones on the same computer must also hack the Parrot Ar-Drone so it connects to a router. To do that follow these instructions on the `ardrone_autonomy` [wiki page](https://github.com/AutonomyLab/ardrone_autonomy/wiki/Multiple-AR-Drones).


##Main installation##

In order to install Collaborative Drone Localization, clone the latest version of our GitHub repository:
```shell
cd catkin_ws/src
git clone https://github.com/MobileRobotics-Ulaval/CollaborativeDroneLocalization.git
cd ..
catkin_make
```

#Launching demo#

The demo launch file launches two instances of `dot_finder` and an instance of `particle_filter`. You need to manually download (http://www.mediafire.com/download/5gz43oigpd3jdod/demo.bag) and play the demo's rosbag using the following commands:
```Shell
rosbag play -l -d 1 demo.bag
```
In another terminal launch the demo's launch file:
```Shell
roslaunch drone_nav demo.launch 
```
In order to watch the demo, run [rqt](http://wiki.ros.org/rqt) and [rviz](http://wiki.ros.org/rviz) the correct perspective for both software are included at the root of this repository. To open the correct perspective in rqt, go to Perspectives->Import... and import "rqt_vision.perspective" at the root of this repository. To open the correct rviz configuration, in rviz go to File->Open Config and open "rviz_config.rviz". 

#Node diagram#

![nodes diagram](http://i.imgur.com/7NptqYb.png)

The above diagram shows the interactions between nodes in the Collaborative Drone Localization. In order to do autonomous flight you need to run multiple instances `ardrone_autonomy`, `dot_finder` and `drone_nav` for the leader and follower. To prevent unwanted interaction between those instances, the leader's and follower's nodes must run in their own [ROS namespace](http://wiki.ros.org/Names). 

##drone_nav##
The `drone_nav` node controls a flying drone either with manual control from a joystick or autonomously via a `geometry_msgs/PoseStamped` message. It subscribes and publishes to the target `ardrone_autonomy` instance. The gamepad controls are (note that you need to have the deadman switch press at all time):
 + Button 0 = Reset drone
 + Button 1 = Launch off
 + Button 2 = Landing
 + Button 3 = Activate Autonomous flight
 + Button 5 = Augment altitude
 + Button 7 = Lower altitude
 + Button 9 = Flat trim
 + Axis 0 = Forward/Backward
 + Axis 1 = Left/Right
 + Axis 2 = Yawn rotation



#### How to launch ####
To run in command line without launch file dot_finder enter the following command:
```shell
rosrun drone_nav dronenav _ctrl:=6 _topic:="/mamba" _goal_radius:=0.75
```
This command will run a `drone_nav` instance that subscribes to a drone in the namespace "mamba" and with the deadman switch set to the button number 6. The same action can be done with the following [launch file](http://wiki.ros.org/roslaunch/XML):

```xml
<launch>
	<node name="drone_nav" pkg="drone_nav" type="drone_nav" output="screen">
		<param name="topic" value="/mamba" />
		<param name="ctrl" value="6" />
		<param name="goal_radius" value="0.75" />
	</node>
</launch>
```

##### Static parameters settings #####
Dot finder require some parameters to be set before launching
 * topic (string)

Name of the drone's namespace. 
 * ctrl (int)

This parameter corresponds to id of the deadman switch button.
 * goal_radius (double)

Radius of the position goal.

##### Subscribed Topics #####
`drone_nav` subscribes to the following topics:
 * (input namespace)/ardrone/navdata (ardrone_autonomy/Navdata)
    
The current state of the drone.

 * /joy ([sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html))

Command from the joystick.

 * (input namespace)/pose ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
    
Relative position from the leader to the follower generate by the `particule_filter`.
  
##### Published Topics #####

`drone_nav` publishes to the following topics:
 * (input namespace)/ardrone/(reset|takeoff|land) ([std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html))

Message to command the reset, takeoof or landing to the drone.

 * (input namespace)/cmd_vel ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))
   
Message to send the velocity command to the drone

 * (input namespace)/goal_marker ([visualization_msgs/Marker](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html))
   
Publish a marker representing the goal in Rviz.

##dot_finder##
The `dot_finder` node extracts the marker positions from the drone's camera using HSV color thresholding.

#### How to launch ####
To run in command line without launch file dot_finder enter the following command:
```shell
rosrun dot_finder dot_finder _ratio:=0.5 _topic:="/mamba"
```
This command will run a `dot_finder` instance that subscribes to a drone in the namespace "mamba" with a ratio of 0.5. The same action can be done with the following [launch file](http://wiki.ros.org/roslaunch/XML):

```xml
<launch>
        <group ns="mamba">
		<node name="dot_finder" pkg="dot_finder" type="dot_finder" output="screen">
			<param name="topic" value="/mamba" />
			<param name="ratio" value="0.5" />
		</node>
        </group>
</launch>
```

##### Static parameters settings #####
The `dot_finder` node require some parameters to be set before launching
 * topic (string)

Name of the drone's namespace. Every topic subscription/publication will replace (input namespace) by the content of this parameter.
 * ratio (float32) suggested = 0.5

At what percentage of a single marker is the point closest to the camera point. A ratio of 0.5 means that the coplanar points are exactly at the center of two orange dots.


##### Dynamic parameters settings #####
The following parameters can be set dynamically during runtime. (Use `rqt_reconfigure` to adjust the parameters).

* Orange(Hue|Sat|Value)(Low|high)

These parameters change the color HSV thresholding. The default parameters are set for an outdoor environment.


* infoToggle (bool, default: True)

Toggle the debugging information add to the visualization image.

* trioToggle (bool, default: True)
 
Toggle the trio (One marker with two oranges dot and one blue dot).

* duoToggle (bool, default: True)

Toggle the duo (two marker with two oranges dot and one blue dot each).

* toggleROI (bool, default: True)

Toggle the Region Of Interest.

* trainingToggle (bool, default: False)

Toggle the data acquisition for MathLab Analysis.

* xROI (int, default: 0, min: 0, max: 640)

X parameter of the Region Of Interest if toggleROI is True.

* yROI (int, default: 0, min: 0, max: 360)

Y parameter of the Region Of Interest if toggleROI is True.

* wROI (int, default: 100, min: 0, max: 640)

Width parameter of the Region Of Interest if toggleROI is True.

* hROI (int, default: 100, min: 0, max: 360)

Height parameter of the Region Of Interest if toggleROI is True.

* dilation_size (double, default: 0.0, min: 0, max: 3)

Dilation force during image processing. Augment this parameter when marker are far from the camera.

* erosion_size (double, default: 0.5, min: 0, max: 3)

Erosion force during image processing. Augment this parameter when marker are close to the camera or image really noisy.

##### Subscribed Topics #####
`dot_finder` subscribes to the following topics:
 * (input namespace)/ardrone/image_raw ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
    
The image from the drone. The markers will be detected from this image.

 * (input namespace)/ardrone/camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))

The camera calibration parameters. It's mainly used to compensate the camera distortion.
  
##### Published Topics #####
`dot_finder` publishes to the following topics:
 * (input namespace)/dots (dot_finder/DuoDot)

The x, y position of every potential marker extracted from the image.
 * (input namespace)/ardrone/image_with_detections ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
   
A debugging visualization image of the computer vision.

##particule_filter##
The `particule_filter` node takes the marker hypothesis of the two cameras, finds the correct hypothesis and compute de pose in 6DoF.

#### How to launch ####
To run in command line without launch file dot_finder enter the following command:
```Shell
rosrun particle_filter particle_filter _leader:="/king" _follower:="/mamba"
```
This command will run a `particle_filter` instance that subscribes to two dot_finders nodes, one in the namespace "king" which is the leader, the other in the namespace "mamba" which is the follower. The same action can be done with the following [launch file](http://wiki.ros.org/roslaunch/XML):

```xml
<launch>
	<node name="particle_filter" pkg="particle_filter" type="particle_filter" output="screen">
		<param name="leader" value="/king" />
		<param name="follower" value="/mamba" />
	</node>
</launch>
```

##### Static parameters settings #####
The `particule_filter` node require some parameters to be set before launching
 * leader (string)

Name of the leader's namespace. Every topic subscription/publication will replace <leader namespace> by the content of this parameter.
 * follower (string)

Name of the follower's namespace. Every topic subscription/publication will replace <follower namespace> by the content of this parameter.

##### Dynamic parameters settings #####
The following parameters can be set dynamically during runtime. (Use `rqt_reconfigure` to adjust the parameters).

* pos_XXXX_led_cam_x (double, default: 0.19, min: 0, max: 2)

These parameters set the distance between the marker and the camera on the drone in meter.
*These values are always positive.*

##### Subscribed Topics #####
`dot_finder` subscribes to the following topics:
 * (leader and follower namespace)/dots (dot_finder/DuoDot)
    
The x, y positions of every marker hypothesis extracted from the image.

 * (leader and follower namespace)/ardrone/imu ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))

Imu information from the drone.

* (leader and follower namespace)/ardrone/image_raw ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
    
The image from the front camera of the drone. Only use for visualization/debugging.

##### Published Topics #####
`dot_finder` publishes to the following topics:
 * (follower namespace)/pose ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))

Relative pose in 6DoF of the follower.

 * (follower namespace)/pose_array_candidates ([geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html))

Every hypothesis of the position of the follower.

 * (follower namespace)/pose ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))

Relative pose in 6DoF of the follower.

 * (leader and follower namespace)/ardrone/image_ROI ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
   
A debugging visualization image of Region of Interest.



[Parrot Ar-Drones 2.0]:http://ardrone2.parrot.com/
[F710 Wireless Gamepad]:http://gaming.logitech.com/en-ca/product/f710-wireless-gamepad
[remapping]:http://wiki.ros.org/joystick_remapper/Tutorials/UsingJoystickRemapper
[RPG Monocular Pose Estimator]:https://github.com/uzh-rpg/rpg_monocular_pose_estimator
[ROS website]:http://wiki.ros.org/hydro/Installation 
[OpenCV]:http://opencv.org/documentation.html
[Eigen]:http://eigen.tuxfamily.org/index.php?title=Main_Page
[workspace]:http://wiki.ros.org/catkin/Tutorials/create_a_workspace
[ardrone_autonomy]:https://github.com/AutonomyLab/ardrone_autonomy


