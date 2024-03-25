# Engagement Detector

ROS 1 package to detect overall users engagement from a robot's ego camera in human-robot interactions. **For ROS 2 version, move to the `humble-dev` branch**.

The approach is detailed in the journal paper: 
[Del Duchetto F, Baxter P and Hanheide M (2020) Are You Still With Me? Continuous Engagement Assessment From a Robot's Point of View. Front. Robot. AI 7:116. doi: 10.3389/frobt.2020.00116](https://doi.org/10.3389/frobt.2020.00116)


## Setup

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
2. Open the folder `engagement_detector` with VSCode in the devcontainer.

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
