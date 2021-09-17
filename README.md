# Engagement Detector

ROS package to detect overall users engagement from a robot's ego camera in human-robot interactions.

The approach is detailed in the journal paper: 
[Del Duchetto F, Baxter P and Hanheide M (2020) Are You Still With Me? Continuous Engagement Assessment From a Robot's Point of View. Front. Robot. AI 7:116. doi: 10.3389/frobt.2020.00116](https://doi.org/10.3389/frobt.2020.00116)


## Install

### Install package with dependencies

1. Install python catkin util: `pip install catkin_pkg`

2. In a terminal go into the root folder of the package: `cd engagement_detector/`

3. and install: `pip install .`

4. Then download the keras model of the newtork: `./download_model.sh`

5. Now, you can build the package in your catkin workspace (i.e. [http://wiki.ros.org/catkin/Tutorials/create_a_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)). 

## Launch

`roslaunch engagement_detector engagement_detector.launch`  

### Engagement value

The predicted engagment value is published on the topic `/engagement_detector/value`.

### Launch parameters

- `image`: (default `/camera/color/image_raw`) input image
- `debug_image`: (default: `true`) whether to publish the out debug image
- `out_image`: (default: `/engagement_detector/out_image`) the debug image topic

## Real-time visualization of engagement

`rosrun image_view image_view image:=/engagement_detector/out_image` will show the camera image with the engagement serie data plotted on it. Something like this:


Single user example            |  Multi users example 
:-------------------------:|:-------------------------:
![](https://github.com/francescodelduchetto/engagement_detector/blob/master/imgs/gif0.gif "Engagement debug")  |  ![](https://github.com/francescodelduchetto/engagement_detector/blob/master/imgs/gif1.gif "Engagement multiusers debug") 


On a `GeForce GTX 1060 6GB` GPU the engagement value is published at a rate of about 5 hz.
