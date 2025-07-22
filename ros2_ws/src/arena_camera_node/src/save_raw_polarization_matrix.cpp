#include "save_raw_polarization_matrix.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PolarImageProcessor>();
    if (node && rclcpp::ok()) {
        node->processAndSave();
    }
    rclcpp::shutdown();
    return 0;
}