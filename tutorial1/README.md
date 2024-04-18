# Simple ROS2 Camera Control Tutorial

## Overview
Control a ROS2 camera node remotely using a Qt application. It's designed for developers looking to integrate ROS2 functionalities with custom Qt GUIs.

## Repository Structure
  -  `/pub-cam`: This directory contains the ROS2 package necessary for enabling video streaming from a camera node.
  -  `/qt-app`: Here you'll find the Qt GUI application. This application provides the functionality to start/stop the video streaming and to display the video stream.

## Computation graph

![Computation graph](./res/rosgraph.png)

  - `cam_client`: The node executed within the Qt GUI application
  - `cam_server`: The node to ensure the video streaming

