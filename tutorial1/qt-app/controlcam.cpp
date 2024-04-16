#include <QDebug>
#include "controlcam.h"

ControlCAM::ControlCAM() {
    mNode=rclcpp::Node::make_shared("cam_client");
    mPublisher=mNode->create_publisher<std_msgs::msg::String>("cam/param",10);
    // Subscribe to video topic

    image_transport::ImageTransport it(mNode);
    image_transport::TransportHints hints(mNode.get(),"compressed");
    image_transport::ImageTransport::VoidPtr t;
    mSubscriber=it.subscribe("cam/image",1, [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        mImage = rosImageToQImage(msg);
        emit imageLoaded(mImage);
        },t,&hints);

    std::thread detached_thread{[this](){
        rclcpp::spin(mNode);
    }};
    detached_thread.detach();
}

void ControlCAM::turnON() {
    std_msgs::msg::String msg;
    msg.data="ON";
    mPublisher->publish<std_msgs::msg::String>(msg);
}

void ControlCAM::turnOFF() {
    std_msgs::msg::String msg;
    msg.data="OFF";
    mPublisher->publish<std_msgs::msg::String>(msg);
}


QImage ControlCAM::rosImageToQImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        // Gérer l'erreur de conversion
        return QImage();
    }
    return QImage(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, QImage::Format_RGB888).rgbSwapped();
}



