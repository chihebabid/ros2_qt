// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <opencv2/highgui.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "image_tools/options.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"

/// Convert an OpenCV matrix encoding type to a string format recognized by sensor_msgs::Image.
/**
 * \param[in] mat_type The OpenCV encoding type.
 * \return A string representing the encoding type.
 */
std::string
mat_type2encoding(int mat_type)
{
    switch (mat_type) {
        case CV_8UC1:
            return "mono8";
        case CV_8UC3:
            return "bgr8";
        case CV_16SC1:
            return "mono16";
        case CV_8UC4:
            return "rgba8";
        default:
            throw std::runtime_error("Unsupported encoding type");
    }
}



int main(int argc, char * argv[])
{
    // Pass command line arguments to rclcpp.
    rclcpp::init(argc, argv);

    // Initialize default demo parameters
    bool show_camera = false;
    size_t depth = rmw_qos_profile_default.depth;
    double freq = 5.0;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    size_t width = 320;
    size_t height = 240;
    bool burger_mode = false;
    std::string topic("image");

    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Configure demo parameters with command line options.
    if (!parse_command_options(
            argc, argv, &depth, &reliability_policy, &history_policy, &show_camera, &freq, &width,
            &height, &burger_mode, &topic))
    {
        return 0;
    }

    // Initialize a ROS 2 node to publish images read from the OpenCV interface to the camera.
    auto node = rclcpp::Node::make_shared("cam2image");
    rclcpp::Logger node_logger = node->get_logger();

    RCLCPP_INFO(node_logger, "Publishing data on topic '%s'", topic.c_str());
    // Create the image publisher with our custom QoS profile.
    //auto pub = node->create_publisher<sensor_msgs::msg::Image>(topic, qos);
    image_transport::ImageTransport it(node);

    auto pub = it.advertise("camera", 1);



    // Set a loop rate for our main event loop.
    rclcpp::WallRate loop_rate(freq);

    cv::VideoCapture cap;


    burger_mode=false;

        //
        // Initialize OpenCV video capture stream.

        // open your camera 'device 0'
	  cap.open(0);

        // Set the width and height based on command line arguments.
        cap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
        if (!cap.isOpened()) {
            RCLCPP_ERROR(node_logger, "Could not open video stream");
            return 1;
        }


    // Initialize OpenCV image matrices.
    cv::Mat frame;
    cv::Mat flipped_frame;

    size_t i = 1;

    // Our main event loop will spin until the user presses CTRL-C to exit.
    while (rclcpp::ok()) {
        cap >> frame;
        // Check if the frame was grabbed correctly
        if (!frame.empty()) {
            // Convert to a ROS image
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            // Publish the image message and increment the frame_id.
            RCLCPP_INFO(node_logger, "Publishing image #%zd", i);
            pub.publish(msg);
            ++i;
        } else {
            RCLCPP_INFO(node_logger, "Finished");
            break;
        }
        // Do some work in rclcpp and wait for more to come in.
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}