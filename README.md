# Engagement Detector

ROS package to detect overall users engagement from a robot's ego camera in human-robot interactions.

A more detailed description of the approach can be found in: 
*Del Duchetto, Francesco, Paul Baxter, and Marc Hanheide. "Are you still with me? Continuous Engagement Assessment from a Robot's Point of View." arXiv preprint arXiv:2001.03515 (2020).* [https://arxiv.org/abs/2001.03515](https://arxiv.org/abs/2001.03515)


## Install

### Install package with dependencies

1. Install python catkin util: `pip install catkin_pkg`

2. In a terminal go into the root folder of the package: `cd engagement_detector/`

3. and install: `pip install .`

4. Then download the keras model of the newtork: `./download_model.sh`

5. Now, you can build the package in your catkin workspace (i.e. ![http://wiki.ros.org/catkin/Tutorials/create_a_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)). 

## Launch

`roslaunch engagement_detectors engagement_detector.launch`  


## Real-time visualization of engagement

`rosrun image_view image_view image:=/engagement_detector/out_image` will show the camera image with the engagement serie data plotted on it. Something like this:


Single user example            |  Multi users example 
:-------------------------:|:-------------------------:
![](https://github.com/francescodelduchetto/engagement_detector/blob/master/imgs/gif0.gif "Engagement debug")  |  ![](https://github.com/francescodelduchetto/engagement_detector/blob/master/imgs/gif1.gif "Engagement multiusers debug") 



## Parameters

- `image`: (default `/camera/color/image_raw`) input image
- `debug_image`: (default: `true`) whether to publish the out debug image
- `out_image`: (default: `/engagement_detector/out_image`) the debug image topic

The engagement value is published to the topic `/engagement_detector/value`. On a `GeForce GTX 1060 6GB` GPU the engagement value is published at a rate of about 5 hz.
