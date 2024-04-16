#ifndef CONTROLCAM_H
#define CONTROLCAM_H
#include <QObject>
#include <QImage>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <image_transport/image_transport.hpp>
class ControlCAM :public QObject
{
    Q_OBJECT
    QImage mImage;
    std::shared_ptr<rclcpp::Node> mNode;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mPublisher;
    std::shared_ptr<image_transport::ImageTransport> mImageTransport;
    image_transport::Subscriber mSubscriber;

    QImage rosImageToQImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

public:
    ControlCAM();
    void turnON();
    void turnOFF();
signals:
    void imageLoaded(const QImage &);
};

#endif // CONTROLCAM_H
