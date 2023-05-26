# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    isaac_ros_imu_bmi088_node = ComposableNode(
        package='isaac_ros_imu_bmi088',
        plugin='nvidia::isaac_ros::imu_bmi088::Bmi088Node',
        name='bmi088',
        parameters=[{'imu_frequency': 200,
                     'accel_index': 0,
                     'gyro_index': 1,
                     'iio_buf_size': 64}])

    bmi088_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='bmi088_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            isaac_ros_imu_bmi088_node,
        ],
        output='screen'
    )

    return launch.LaunchDescription([bmi088_container])
