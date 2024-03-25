# Engagement Detector

ROS 2 package to detect overall users engagement from a robot's ego camera in human-robot interactions. 

The approach is detailed in the journal paper: 
[Del Duchetto F, Baxter P and Hanheide M (2020) Are You Still With Me? Continuous Engagement Assessment From a Robot's Point of View. Front. Robot. AI 7:116. doi: 10.3389/frobt.2020.00116](https://doi.org/10.3389/frobt.2020.00116)


## Setup Devcontainer

### Install package with dependencies
1. Make sure you have VSCode installed: https://code.visualstudio.com/download
2. Make sure you have git installed: https://git-scm.com/book/en/v2/Getting-Started-Installing-Git
3. Make sure you have the `Docker` and the `Dev Containers` extension in VSCode installed and working: https://code.visualstudio.com/docs/containers/overview and https://code.visualstudio.com/docs/devcontainers/containers
    * ensure docker is working, i.e. try `docker run --rm hello-world` and check it succeeds for your user

### Environment

**If you are on a Windows PC the following two additional steps are required:**

   - Open a terminal(e.g., window's powershell), type `git config --global core.autocrlf false` and press Enter
   - Make sure docker is running by launching the docker desktop application

Then:

1. `git clone https://github.com/LCAS/engagement_detector.git`
2. `cd engagement_detector && git checkout humble-dev`
3. Open the folder `engagement_detector` with VSCode in the devcontainer.


## Launch

`ros2 run engagement_detector engagement_detector_node`  

### Engagement value

The predicted engagment value is published on the topic `/engagement_detector/value`.

### Launch parameters

- `image`: (default `/camera/color/image_raw`) input image
- `debug_image`: (default: `true`) whether to publish the out debug image
- `out_image`: (default: `/engagement_detector/out_image`) the debug image topic

## Webcam test in devcontainer

The following command will take images from your PC's camera and publish it to the default image topic:
```
ros2 run engagement_detector webcam_node
```

If you want to visualise the `out_image` topic you can do so by:

1. launching the devcontainer VNC window (PORTS tab > right-click on desktop (6080) > Open in Browser > insert password: `vscode`)
2. open a new terminal and launch `rqt`
3. select Visualization > Image View. 

## Real-time visualization of engagement

`rosrun image_view image_view image:=/engagement_detector/out_image` will show the camera image with the engagement serie data plotted on it. Something like this:


Single user example            |  Multi users example 
:-------------------------:|:-------------------------:
![](https://github.com/francescodelduchetto/engagement_detector/blob/master/imgs/gif0.gif "Engagement debug")  |  ![](https://github.com/francescodelduchetto/engagement_detector/blob/master/imgs/gif1.gif "Engagement multiusers debug") 


On a `GeForce GTX 1060 6GB` GPU the engagement value is published at a rate of about 5 hz.
