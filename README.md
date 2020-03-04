# Engagement Detector

ROS package to detect overall users engagement from a robot's ego camera during human-robot interactions.

A more detailed description of the approach can be found in: 
*Del Duchetto, Francesco, Paul Baxter, and Marc Hanheide. "Are you still with me? Continuous Engagement Assessment from a Robot's Point of View." arXiv preprint arXiv:2001.03515 (2020).* [https://arxiv.org/abs/2001.03515](https://arxiv.org/abs/2001.03515)



## Launch

`roslaunch engagement_detectors engagement_detector.launch`  


## Real-time visualization of engagement

`rosrun image_view image_view image:=/engagement_detector/out_image` will show the camera image with the engagement serie data plotted on it. Something like this:

![alt text](https://github.com/francescodelduchetto/engagement_detector/blob/master/imgs/gif0.gif "Engagement debug")


## Parameters

- `image: (default /camera/color/image_raw)` input image
- `debug_image: (default: true)` wheter to publish the out debug image
- `out_image: (default: /engagement_detector/out_image)` the debug image topic

The engagement value is published to the topic `/engagement_detector/value`. On a `GeForce GTX 1060 6GB` GPU the engagement value is published at a rate of about 5 hz.


NOTE: This code has been tested on keras==2.2.4, tensorflow-gpu==1.14.0.
