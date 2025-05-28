#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "ArenaApi.h"

#define IMAGE_TIMEOUT 2000
#define SYSTEM_TIMEOUT 100

class PolarImagePublisher : public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_dolp_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_aolp_;
    rclcpp::TimerBase::SharedPtr timer_;

    Arena::ISystem* system;
    Arena::IDevice* device;

    void grabAndPublish()
    {
        Arena::IImage* pImage = device->GetImage(IMAGE_TIMEOUT);

        size_t width = pImage->GetWidth();
        size_t height = pImage->GetHeight();
        size_t inBitsPerPixel = pImage->GetBitsPerPixel();
        size_t inPixelSize = inBitsPerPixel / 8;
        const uint8_t* pInput = pImage->GetData();

        // DoLP & AoLP 이미지 생성
        cv::Mat dolp_img(height, width, CV_8UC1);
        cv::Mat aolp_img(height, width, CV_8UC1);

        const uint8_t* pIn = pInput;
        for (size_t i = 0; i < height; ++i)
        {
            for (size_t j = 0; j < width; ++j)
            {
                dolp_img.at<uchar>(i, j) = *pIn;
                aolp_img.at<uchar>(i, j) = *(pIn + 1);
                pIn += inPixelSize;
            }
        }

        // ROS 메시지로 퍼블리시
        auto dolp_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", dolp_img).toImageMsg();
        dolp_msg->header.stamp = this->get_clock()->now();
        dolp_msg->header.frame_id = "polar_camera";
        pub_dolp_->publish(*dolp_msg);

        auto aolp_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", aolp_img).toImageMsg();
        aolp_msg->header.stamp = this->get_clock()->now();
        aolp_msg->header.frame_id = "polar_camera";
        pub_aolp_->publish(*aolp_msg);

        device->RequeueBuffer(pImage);
    }

public:
    PolarImagePublisher() : Node("polar_aolp_dolp"), system(nullptr), device(nullptr)
    {
        pub_dolp_ = this->create_publisher<sensor_msgs::msg::Image>("/polar/dolp", 10);
        pub_aolp_ = this->create_publisher<sensor_msgs::msg::Image>("/polar/aolp", 10);

        RCLCPP_INFO(this->get_logger(), "Opening Arena system...");
        system = Arena::OpenSystem();
        system->UpdateDevices(SYSTEM_TIMEOUT);
        auto devices = system->GetDevices();

        if (devices.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No Lucid camera found.");
            rclcpp::shutdown();
        }

        Arena::DeviceInfo selectedDevice = devices[0]; // 단일 카메라 사용 가정
        device = system->CreateDevice(selectedDevice);

        Arena::SetNodeValue<GenICam::gcstring>(device->GetNodeMap(), "PixelFormat", "PolarizedDolpAolp_BayerRG8");
        Arena::SetNodeValue<bool>(device->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
        Arena::SetNodeValue<bool>(device->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

        device->StartStream();

        timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&PolarImagePublisher::grabAndPublish, this));
    }

    ~PolarImagePublisher()
    {
        if (device)
        {
            device->StopStream();
            system->DestroyDevice(device);
        }
        Arena::CloseSystem(system);
    }
};