
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

// using std::placeholders::_1; // No longer needed

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

class PolarImageProcessor : public rclcpp::Node
{
    private:
        // Arena SDK
        Arena::ISystem* system;
        Arena::IDevice* device;

        int index = 0;

    public:
        PolarImageProcessor() : Node("polar_processor"), system(nullptr), device(nullptr)
        {
            std::cout << "Opening Arena system...\n";
            system = Arena::OpenSystem();
            system->UpdateDevices(SYSTEM_TIMEOUT);
            auto devices = system->GetDevices();
            
            if (devices.empty()) {
                RCLCPP_ERROR(this->get_logger(), "No Lucid camera found.");
                rclcpp::shutdown();
                return;
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
        }

        void processAndSave()
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
                auto* ptr_dolp      = dolp_img.ptr<float>(i);
                auto* ptr_aolp      = aolp_img.ptr<float>(i);
                auto* ptr_stokes    = stokes_img.ptr<cv::Vec3f>(i);

                for (size_t j = 0; j < width; ++j) {
                    ptr0[j] = *(pIn + 0);
                    ptr45[j] = *(pIn + 1);
                    ptr90[j] = *(pIn + 2);
                    ptr135[j] = *(pIn + 3);

                    // Note: The original S0 calculation might lead to values > 1.0.
                    // Using a standard definition: S0 = 0.5 * (I0 + I45 + I90 + I135)
                    // The values are kept in float and normalized to [0, 1] by dividing by 255.0
                    float i0 = static_cast<float>(*(pIn + 0));
                    float i45 = static_cast<float>(*(pIn + 1));
                    float i90 = static_cast<float>(*(pIn + 2));
                    float i135 = static_cast<float>(*(pIn + 3));

                    float S0 = 0.5f * (i0 + i45 + i90 + i135) / 255.0f;
                    float S1 = (i0 - i90) / 255.0f;
                    float S2 = (i45 - i135) / 255.0f;
                    
                    ptr_stokes[j] = cv::Vec3f(S0, S1, S2);
                    
                    float dolp_val = (S0 > 1e-6) ? (std::sqrt(S1 * S1 + S2 * S2) / S0) : 0.0f;
                    ptr_dolp[j] = dolp_val;

                    // AoLP is calculated in radians and then converted to degrees [0, 180]
                    float aolp_rad = 0.5f * std::atan2(S2, S1);
                    float aolp_deg = aolp_rad * 180.0f / M_PI;
                    if (aolp_deg < 0) {
                        aolp_deg += 180.0f;
                    }
                    ptr_aolp[j] = aolp_deg;
                    
                    pIn += inPixelSize;
                }
            }
            
            // Requeue the buffer
            device->RequeueBuffer(pImage);
            
            // Save images
            std::string output_dir = "output_images";
            // Create directory if it doesn't exist. Note: C++17 filesystem would be better.
            // This is a simple solution using a system call.
            mkdir(output_dir.c_str(), 0777);

            cv::imwrite(output_dir + "/mono_0_"+std::to_string(index)+".png", mono_img_0);
            cv::imwrite(output_dir + "/mono_45_"+std::to_string(index)+".png", mono_img_45);
            cv::imwrite(output_dir + "/mono_90_"+std::to_string(index)+".png", mono_img_90);
            cv::imwrite(output_dir + "/mono_135_"+std::to_string(index)+".png", mono_img_135);

            // For DoLP, values are [0, 1]. Scale to [0, 255] for saving as PNG.
            cv::Mat dolp_save;
            dolp_img.convertTo(dolp_save, CV_8U, 255.0);
            cv::imwrite(output_dir + "/dolp_"+std::to_string(index)+".png", dolp_save);

            // For AoLP, values are [0, 180]. Can be saved as is, will be mapped to [0, 255] range.
            // Using a colormap can be more intuitive for visualization.
            cv::Mat aolp_save;
            aolp_img.convertTo(aolp_save, CV_8U, 255.0 / 180.0);
            cv::Mat aolp_color;
            cv::applyColorMap(aolp_save, aolp_color, cv::COLORMAP_HSV);
            cv::imwrite(output_dir + "/aolp_color_"+std::to_string(index)+".png", aolp_color);
            cv::imwrite(output_dir + "/aolp_gray_"+std::to_string(index)+".png", aolp_save); // also save grayscale

            // For Stokes, it's a 3-channel float image. We can save it as a color image.
            // S0, S1, S2 are in different ranges. We need to normalize them for visualization.
            std::vector<cv::Mat> stokes_channels;
            cv::split(stokes_img, stokes_channels);
            cv::Mat s0_norm, s1_norm, s2_norm;
            // Normalize each channel to [0, 255] to be stored in a 3-channel image
            cv::normalize(stokes_channels[0], s0_norm, 0, 255, cv::NORM_MINMAX, CV_8U);
            cv::normalize(stokes_channels[1], s1_norm, 0, 255, cv::NORM_MINMAX, CV_8U);
            cv::normalize(stokes_channels[2], s2_norm, 0, 255, cv::NORM_MINMAX, CV_8U);

            std::vector<cv::Mat> stokes_to_merge = {s0_norm, s1_norm, s2_norm};
            cv::Mat stokes_save;
            cv::merge(stokes_to_merge, stokes_save);
            cv::imwrite(output_dir + "/stokes_S0_S1_S2_as_RGB_"+std::to_string(index)+".png", stokes_save);

            index++;

            RCLCPP_INFO(this->get_logger(), "Saved images to '%s' directory.", output_dir.c_str());
        }

        ~PolarImageProcessor()
        {
            if (device) {
                device->StopStream();
                system->DestroyDevice(device);
            }
            if (system) {
                Arena::CloseSystem(system);
            }
        }
};