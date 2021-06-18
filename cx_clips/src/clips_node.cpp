#include "rclcpp/rclcpp.hpp"
#include "clipsmm.h"

int main(int argc, const char** argv) {
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("info"), "CLIPS INITIALIZED!");
    return 0;
}