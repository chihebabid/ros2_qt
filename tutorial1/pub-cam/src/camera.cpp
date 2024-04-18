#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

class MynodeSub : public rclcpp::Node {
public:
    MynodeSub(std::string nom) : Node(nom) {
        mSubscriber = create_subscription<std_msgs::msg::String>(
                "cam/param",
                10,
                std::bind(&MynodeSub::my_callback, this, _1)
        );
        auto qos{rclcpp::SystemDefaultsQoS().get_rmw_qos_profile()};

        mPublisher = image_transport::create_publisher(this, "cam/image", qos);
        /*image_transport::ImageTransport it(this->get);
        image_transport::Publisher pub = it.advertise("camera/image", 1);*/
        mCap.open(0);
        mCap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(320));
        mCap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(240));
        mTimer = create_wall_timer(100ms, std::bind(&MynodeSub::timer_callback, this));

    }

private:
/**
 * @brief Enable/Disable the camera.
 * @param msg The received ROS string message (ON/OFF
 */
    void my_callback(const std_msgs::msg::String::ConstSharedPtr &msg) {
        if (msg->data == "ON") {
            if (mTimer->is_canceled()) {
                mCap.open(0);
                mCap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(320));
                mCap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(240));
                mTimer->reset();
            }
        } else {
            mTimer->cancel();
            mCap.release();
        }
    }

    /*
     * @brief Publish one frame to the topic my_image.
     */
    void timer_callback() {
        if (mCap.isOpened()) {
            mCap >> mFrame;
            if (!mFrame.empty()) {
                auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", mFrame).toImageMsg();
                mPublisher.publish(msg);
            }
        }
    }

    cv::Mat mFrame;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> mSubscriber;
    cv::VideoCapture mCap;
    image_transport::Publisher mPublisher;
    std::shared_ptr<rclcpp::TimerBase> mTimer;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MynodeSub>("cam_server");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}