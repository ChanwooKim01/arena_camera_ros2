#include <polarization_all.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PolarImagePublisher>());
  rclcpp::shutdown();
  return 0;
}