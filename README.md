# Duckiebot ROS Assignment

This repository contains the code and documentation for the ROS-based Duckiebot assignment. We familiarized ourself with ROS and duckiebot functionalities and implemented various functions including odometry using wheel encoders, rotating the robot, and making the Duckiebot follow a specific path while utilizing ROS services and visual feedback via LED lights.

**NOTE:** If you want to develop software that does not use
ROS, check out [this template](https://github.com/duckietown/template-basic).


## Part I: Getting Comfortable with ROS

### 1. Objective

Understand and work with the basic concepts of ROS, such as nodes, topics, services, messages, and bags.

### Tasks Completed
#### ROS Wiki Concepts:

Nodes, Topics, Services, Messages, and Bags were explored and explained in detail.
Communication setup between nodes was established and tested.

### 2. Create a new repository

Create a new repository on github.com while
specifying the newly forked template repository as
a template for your new repository.


### 3. Define dependencies

List the dependencies in the files `dependencies-apt.txt` and
`dependencies-py3.txt` (apt packages and pip packages respectively).


### 4. Place your code

Place your code in the directory `/packages/` of
your new repository.


### 5. Setup launchers

The directory `/launchers` can contain as many launchers (launching scripts)
as you want. A default launcher called `default.sh` must always be present.

If you create an executable script (i.e., a file with a valid shebang statement)
a launcher will be created for it. For example, the script file 
`/launchers/my-launcher.sh` will be available inside the Docker image as the binary
`dt-launcher-my-launcher`.

When launching a new container, you can simply provide `dt-launcher-my-launcher` as
command.
