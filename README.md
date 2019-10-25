README
======

This is the vision package of the Hamburg Bit-Bots.
For standardized cameras such as USB-webcams we use the wolves_image_provider package as image provider.
Alternatively every image source, that publishes sensor_msgs/Image messages (i.e. basler driver for basler cameras) is supported.
Settings considering the vision are set in the visionparams.yaml (bitbots_vision/config/visionparams.yaml).
You do NOT want to enable DEBUG on the robot.
The color calibration files are created with the wolves colorpicker and a rosbag.
The source code of the vision is located in bitbots_vision/src.
In bitbots_vision/models the fcnn models are stored. Due to their size, these are not part of this repository.
After changes in the models or config/color_spaces directory, rebuilding the vision node is required to see these changes in dynamic reconfigure.
To tweak the camera image, use the settings in the image provider.

nodes
---------------------

pkg="bitbots_vision" type="vision.py" name="bitbots_vision"

publishers
---------------------

pub_config topic:'/speak' type:Speak
speak_publisher topic:'vision_config' type:Config
pub_balls topic: config['ROS_ball_msg_topic'] type:BallsInImage
pub_lines topic:config['ROS_line_msg_topic'] type:LineInformationInImage
pub_obstacle topic:config['ROS_obstacle_msg_topic'] type:ObstaclesInImage
pub_goal topic:config['ROS_goal_msg_topic'] type:GoalInImage
pub_ball_fcnn topic:config['ROS_fcnn_img_msg_topic'] type:ImageWithRegionOfInterest
pub_debug_image topic:config['ROS_debug_image_msg_topic'] type:Image
pub_debug_fcnn_image topic:config['ROS_debug_fcnn_image_msg_topic'] type:Image

subscribers
---------------------

image_sub topic: config['ROS_img_msg_topic'] type:Image
callback: _image_callback
head_sub topic: config['ROS_head_joint_msg_topic'] type:JointState
callback: _head_joint_state_callback



pkg="bitbots_vision" type="dynamic_color_space.py" name="bitbots_dynamic_color_space"

publishers
---------------------

pub_color_space topic: vision_config['ROS_dynamic_color_space_msg_topic'] type:ColorSpace

subscribers
---------------------
sub_vision_config_msg topic: 'vision_config' type:Config
callback: vision_config_callback
sub_image_msg topic: vision_config['ROS_img_msg_topic']type:Image
callback: image_callback


Neural Network Models
---------------------

Currently, the models of our neural networks are not available publicly.
Due to their size, they are not included in the repository.
Bit-Bots find them in the Mafiasi NextCloud `robocup-ai/log/`


Vision Tools
------------

In the bitbots_vision_tools directory, special tools for debugging/introspection purposes are provided.


Launchscripts
-------------

To start the vision, use 
```
roslaunch bitbots_vision vision_startup.launch
```

```sim:=true``` does activate simulation time, switch to simulation color settings and deactivate launching of an image provider
```camera:=false``` does deactivate all image providers (e.g. for use with rosbags or in simulation)
```basler:=false```does start wolves image provider instead of the basler camera driver
```dummyball:=true``` does not start the ball detection to save resources
```debug:=true``` does activate publishing of several debug images which can be inspected in the rqt image view
```use_game_settings:=true``` does load additional game settings
 
**bitbots_vision**
- *vision_startup*: starts the vision and a camera image provider
    - Params: 
        - sim [true/FALSE],
        - camera [TRUE/false],
        - basler [TRUE/false],
        - dummyball [true/FALSE],
        - debug [true/FALSE],
        - use_game_settings [true/FALSE]

**bitbots_vision_tools**
- *bitbots_vision_recordbag*: records a rosbag containing the camera image as image_raw and stores it as /tmp/tmpbag.bag  


Compiling
-------------
clone bitbots_vision and [usb_cam](https://github.com/Zcyyy/usb_cam) in your bitbots_mate  
- *compiling*   
Delete the deve, build, logs, and.catkin_tools folders in your workspace，and run  
‘’‘  
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3  
catkin build  
’‘’  

You need to perform a few operations to run the simulation environment,after you source the path,of course  
‘’‘  
roslaunch bitbots_vision vision_startup.launch sim:=true camera:=false
’‘’  


