// Copyright (c) 2024-2025 Andrew O'Shei
//
// This software is released under the BSD-3-Clause License.
// See LICENSE file in the root directory for full license information.
//
// This file is part of the Unitree Go2 ROS2 simulation project.
// Based on CHAMP quadruped controller by Juan Miguel Jimeno.

#include "quadruped_controller.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QuadrupedController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
