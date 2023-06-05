# ROS_AutoTaxi (still maintaining)

ROS_AutoTaxi is a self-driving car project developed using ROS Noetic. It simulates an autonomous taxi that performs various tasks such as autonomous navigation, sign detection, traffic light recognition, and lane tracking. The simulation uses the Prius model as the base vehicle.

## Features

- Autonomous navigation: The self-driving car is capable of navigating through the environment autonomously using sensor data and path planning algorithms.
- Sign detection: The car uses image processing techniques to detect and recognize traffic signs, allowing it to respond accordingly.
- Traffic light recognition: The car employs computer vision algorithms to detect and interpret traffic lights, enabling it to make appropriate driving decisions.
- Lane tracking: The car utilizes sensor data to perform lane tracking and maintain its position within the designated lanes.

## Dependencies

This project runs on ROS noetic

To install all ROS dependencies of this project, run command: 
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

This project requires opencv, to install, do the following commands:

    * Install OpenCV using the package manager
    ```bash
    sudo apt-get install libopencv-dev

    ```

    * Install required dependencies
    ```bash
    sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
    ```

    * Clone the OpenCV repository
    ```bash
    git clone https://github.com/opencv/opencv.git
    ```
    * Create a build directory and navigate into it and configure the build
    ```bash
    cd opencv
    mkdir build
    cd build
    cmake ..
    ```

    * Build and install OpenCV
    ```bash
    make -j4 # Use a higher number if you have more CPU cores
    sudo make install
    ```

## Installation
To install this project, follow these steps
    * Clone my repository
    ```bash
    git clone https://github.com/son524lt/ROS_AutoTaxi.git
    ```

    * Make the project
    ```bash
    catkin_make
    ```
## Demo