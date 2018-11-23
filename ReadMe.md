# Hacking_dbw_mkz

## 1. Introduction
This is a personal practice to hack into Dataspeed ADAS Development Vehicle Kit, which is a semi-open-source project for developing 
self-driving cars and their simulation using ROS and Gazebo.

More basic information and guides to setup the environments can be found in the [official sites](https://bitbucket.org/DataspeedInc/dbw_mkz_ros) and their [official documents](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/4f3e09f7c23e6cb3672092d3c194569a109d884d/ROS_SETUP.md?fileviewer=file-view-default).

However, there is limited guides to make the platform work for us. 
Firstly, it takes some time to understand the structure of the system from rqt_graph or source code directly, especially for a beginner like me. Besides, as observed in the source code [site](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src) (2018.11.23), the source code of the package "dbw_mkz_vision_sim" is not available yet.

This project starts from the most sophisticated demo provided officially from Dataspeed, i.e. "lane_keep_demo.launch" where a car is controlled to keep in lane.

Currently, the project contain code for :
    
    1. Semi-customed launchers and world files that modify the original world or change some parts of the codes into my code.
    2. A simple implementation for twist_controller (the controller that outputs steering, throttle and braking commands). Source codes for the official twist_controller is actually open source.
    3. A MPC implementation for path_follower (the controller that takes in target_path and outputs twist command to twist_controller).

TODO: 

    1. Further hack into the vision system and provide non-trivial lane-detection algorithm.
    2. Provide some "plain" and intuitive big picture on the system. As well as some details.

## 2. Dependencies
The path_following node is actually implemented similar to the Udacity [MPC project](https://github.com/udacity/CarND-MPC-Project) in self-driving car Nanodegree program. 

More importantly, install ipopt library. Personally I recommend following guides also from [udacity](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md). 
