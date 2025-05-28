
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "ArenaApi.h"
#include "Save/SaveApi.h"

#define _USE_MATH_DEFINES
#include <math.h>

#define TAB1 "  "
#define TAB2 "    "
// pixel format
#define PIXEL_FORMAT BGR8

// image timeout
#define IMAGE_TIMEOUT 2000

// system timeout
#define SYSTEM_TIMEOUT 100

using std::placeholders::_1;

/*
ToDo:
- aolp, dolp, stokes 정보를 효과적으로 담을 수 있는 커스텀 메세지 타입 정의
- 비편광 이미지 정보도 받아올 수 있는 방법 찾아보기
- 연산 시간 줄이기
*/

Arena::DeviceInfo SelectDevice(std::vector<Arena::DeviceInfo>& deviceInfos)
{
	if (deviceInfos.size() == 1)
	{
		std::cout << "\n"
				  << "Only one device detected: " << deviceInfos[0].ModelName() <<  deviceInfos[0].SerialNumber() << deviceInfos[0].IpAddressStr() << ".\n";
		std::cout << "Automatically selecting this device.\n";
		return deviceInfos[0];
	}

	std::cout << "\nSelect device:\n";
	for (size_t i = 0; i < deviceInfos.size(); i++)
	{
		std::cout <<  i + 1 << ". " << deviceInfos[i].ModelName() <<  deviceInfos[i].SerialNumber() <<  deviceInfos[i].IpAddressStr() << "\n";
	}
	size_t selection = 0;

	do
	{
		std::cout <<  "Make selection (1-" << deviceInfos.size() << "): ";
		std::cin >> selection;

		if (std::cin.fail())
		{
			std::cin.clear();
			while (std::cin.get() != '\n')
				;
			std::cout <<  "Invalid input. Please enter a number.\n";
		}
		else if (selection <= 0 || selection > deviceInfos.size())
		{
			std::cout <<  "Invalid device selected. Please select a device in the range (1-" << deviceInfos.size() << ").\n";
		}

	} while (selection <= 0 || selection > deviceInfos.size());

	return deviceInfos[selection - 1];
}

class PolarImagePublisher : public rclcpp::Node
{
    private:
        // ROS2 node
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_img_0_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_img_45_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_img_90_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_img_135_;
        
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_dolp_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_aolp_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_stokes_;
        
        // Arena SDK
        Arena::ISystem* system;
        Arena::IDevice* device;

        void grabAndPublish()
        {
            // Grab image from the camera
            RCLCPP_INFO(this->get_logger(), "Starting image grab...");
            Arena::IImage* pImage = device->GetImage(IMAGE_TIMEOUT);
            size_t width = pImage->GetWidth();
            size_t height = pImage->GetHeight();
            size_t inBitsPerPixel = pImage->GetBitsPerPixel();
            size_t inPixelSize = inBitsPerPixel / 8;
            const uint8_t* pInput = pImage->GetData();
            
            // Define output images
            cv::Mat mono_img_0(height, width, CV_8UC1);
            cv::Mat mono_img_45(height, width, CV_8UC1);
            cv::Mat mono_img_90(height, width, CV_8UC1);
            cv::Mat mono_img_135(height, width, CV_8UC1);

            cv::Mat dolp_img(height, width, CV_32FC1);
            cv::Mat aolp_img(height, width, CV_32FC1);
            cv::Mat stokes_img(height, width, CV_32FC3);
            const uint8_t* pIn = pInput;

            // Fill the output images with data
            for (size_t i = 0; i < height; ++i) {
                auto* ptr0          = mono_img_0.ptr<uchar>(i);
                auto* ptr45         = mono_img_45.ptr<uchar>(i);
                auto* ptr90         = mono_img_90.ptr<uchar>(i);
                auto* ptr135        = mono_img_135.ptr<uchar>(i);
                auto* ptr_dolp      = dolp_img.ptr<uchar>(i);
                auto* ptr_aolp      = aolp_img.ptr<uchar>(i);
                auto* ptr_stokes    = stokes_img.ptr<cv::Vec3b>(i);  // 또는 Vec3f로 변경

                for (size_t j = 0; j < width; ++j) {
                    ptr0[j] = *(pIn + 0);
                    ptr45[j] = *(pIn + 1);
                    ptr90[j] = *(pIn + 2);
                    ptr135[j] = *(pIn + 3);

                    float S0 = (0.5f * (*(pIn + 0) + *(pIn + 1) + *(pIn + 2) + *(pIn + 3))) / 255.0f;
                    float S1 = (*(pIn + 0) - *(pIn + 2)) / 255.0f;
                    float S2 = (*(pIn + 1) - *(pIn + 3)) / 255.0f;

                    float DoLP = (std::sqrt(S1 * S1 + S2 * S2) / S0); // Normalize to [0, 1]
                    float AoLP = 0.5f * std::atan2(S2, S1) * 180.0f / M_PI;

                    ptr_dolp[j] = static_cast<uchar>(DoLP);
                    ptr_aolp[j] = static_cast<uchar>(AoLP);

                    ptr_stokes[j] = cv::Vec3b(S0, S1, S2);
                    pIn += inPixelSize;
                }
            }

            // Convert mono images to RGB format
            cv::Mat raw_img_0;
            cv::Mat raw_img_45;
            cv::Mat raw_img_90;
            cv::Mat raw_img_135;
            cv::cvtColor(mono_img_0, raw_img_0, cv::COLOR_BayerRG2RGB);
            cv::cvtColor(mono_img_45, raw_img_45, cv::COLOR_BayerRG2RGB);
            cv::cvtColor(mono_img_90, raw_img_90, cv::COLOR_BayerRG2RGB);
            cv::cvtColor(mono_img_135, raw_img_135, cv::COLOR_BayerRG2RGB);
            
            // Create ROS2 messages from OpenCV images
            auto raw_img_msg_0      = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", raw_img_0).toImageMsg();
            auto raw_img_msg_45     = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", raw_img_45).toImageMsg();
            auto raw_img_msg_90     = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", raw_img_90).toImageMsg();
            auto raw_img_msg_135    = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", raw_img_135).toImageMsg();
            
            auto stokes_msg         = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC3", stokes_img).toImageMsg();
            auto aolp_msg           = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", aolp_img).toImageMsg();
            auto dopl_msg           = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", dolp_img).toImageMsg();
            
            raw_img_msg_0->header.stamp         = this->get_clock()->now();
            raw_img_msg_0->header.frame_id      = "polar_camera";
            raw_img_msg_45->header.stamp        = this->get_clock()->now();
            raw_img_msg_45->header.frame_id     = "polar_camera";
            raw_img_msg_90->header.stamp        = this->get_clock()->now();
            raw_img_msg_90->header.frame_id     = "polar_camera";
            raw_img_msg_135->header.stamp       = this->get_clock()->now();
            raw_img_msg_135->header.frame_id    = "polar_camera";
            
            stokes_msg->header.stamp            = this->get_clock()->now();
            stokes_msg->header.frame_id         = "polar_camera";
            aolp_msg->header.stamp              = this->get_clock()->now();
            aolp_msg->header.frame_id           = "polar_camera";
            dopl_msg->header.stamp              = this->get_clock()->now();
            dopl_msg->header.frame_id           = "polar_camera";
            
            // Publish the messages
            pub_raw_img_0_->publish(*raw_img_msg_0);
            pub_raw_img_45_->publish(*raw_img_msg_45);
            pub_raw_img_90_->publish(*raw_img_msg_90);
            pub_raw_img_135_->publish(*raw_img_msg_135);
            
            pub_stokes_->publish(*stokes_msg);
            pub_aolp_->publish(*aolp_msg);
            pub_dolp_->publish(*dopl_msg);
            RCLCPP_INFO(this->get_logger(), "Published images and polarization data.");
            
            // Requeue the buffer
            device->RequeueBuffer(pImage);
        }

    public:
        PolarImagePublisher() : Node("polar_all"), system(nullptr), device(nullptr)
        {
            pub_dolp_           = this->create_publisher<sensor_msgs::msg::Image>("/polar/dolp", 10);
            pub_aolp_           = this->create_publisher<sensor_msgs::msg::Image>("/polar/aolp", 10);
            pub_stokes_         = this->create_publisher<sensor_msgs::msg::Image>("/polar/stokes", 10);

            pub_raw_img_0_      = this->create_publisher<sensor_msgs::msg::Image>("/polar/image_raw_0d", 10);
            pub_raw_img_45_     = this->create_publisher<sensor_msgs::msg::Image>("/polar/image_raw_45d", 10);
            pub_raw_img_90_     = this->create_publisher<sensor_msgs::msg::Image>("/polar/image_raw_90d", 10);
            pub_raw_img_135_    = this->create_publisher<sensor_msgs::msg::Image>("/polar/image_raw_135d", 10);
            
            std::cout << "Opening Arena system...\n";
            system = Arena::OpenSystem();
            system->UpdateDevices(SYSTEM_TIMEOUT);
            auto devices = system->GetDevices();
            
            if (devices.empty()) {
                RCLCPP_ERROR(this->get_logger(), "No Lucid camera found.");
                rclcpp::shutdown();
            }

            std::cout << "selecting device...\n";
            Arena::DeviceInfo selectedDeviceInfo = SelectDevice(devices);
            device = system->CreateDevice(selectedDeviceInfo);
            
            std::cout << "setting node...\n";
            Arena::SetNodeValue<GenICam::gcstring>(device->GetNodeMap(), "PixelFormat", "PolarizedAngles_0d_45d_90d_135d_BayerRG8");
            std::cout << "setting stream node...\n";
            Arena::SetNodeValue<bool>(device->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
            Arena::SetNodeValue<bool>(device->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
            
            std::cout << "starting stream...\n";
            device->StartStream();

            timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&PolarImagePublisher::grabAndPublish, this));
        }

        ~PolarImagePublisher()
        {
            if (device) {
            device->StopStream();
            system->DestroyDevice(device);
            }
            Arena::CloseSystem(system);
        }
};