// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "isaac_ros_imu_bmi088/isaac_ros_imu_bmi088_node.hpp"
#include "isaac_ros_nitros_imu_type/nitros_imu.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "gxf/std/timestamp.hpp"
#pragma GCC diagnostic pop


namespace nvidia
{
namespace isaac_ros
{
namespace imu_bmi088
{

using nvidia::gxf::optimizer::GraphIOGroupSupportedDataTypesInfoList;

#define OUTPUT_COMPONENT_KEY_IMU                  "sink/sink"
#define OUTPUT_DEFAULT_TENSOR_FORMAT_IMU          "nitros_imu"
#define OUTPUT_TOPIC_NAME_IMU                     "imu"

constexpr char APP_YAML_FILENAME[] = "config/bmi088_node.yaml";
constexpr char PACKAGE_NAME[] = "isaac_ros_imu_bmi088";

const std::vector<std::pair<std::string, std::string>> EXTENSIONS = {
  {"isaac_ros_gxf", "gxf/lib/serialization/libgxf_serialization.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_isaac_messages.so"},
  {"isaac_ros_gxf", "gxf/lib/libgxf_bmi088_imu.so"},  // bmi088_driver
  {"isaac_ros_gxf", "gxf/lib/libgxf_imu_utils.so"}  // imu_combiner
};
const std::vector<std::string> PRESET_EXTENSION_SPEC_NAMES = {
  "isaac_ros_imu_bmi088"
};
const std::vector<std::string> EXTENSION_SPEC_FILENAMES = {};
const std::vector<std::string> GENERATOR_RULE_FILENAMES = {
  "config/namespace_injector_rule_bmi088.yaml"
};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
const nitros::NitrosPublisherSubscriberConfigMap CONFIG_MAP = {
  {OUTPUT_COMPONENT_KEY_IMU,
    {
      .type = nitros::NitrosPublisherSubscriberType::NEGOTIATED,
      .qos = rclcpp::QoS(10),
      .compatible_data_format = OUTPUT_DEFAULT_TENSOR_FORMAT_IMU,
      .topic_name = OUTPUT_TOPIC_NAME_IMU,
    }
  }
};
#pragma GCC diagnostic pop

Bmi088Node::Bmi088Node(const rclcpp::NodeOptions & options)
: nitros::NitrosNode(options,
    APP_YAML_FILENAME,
    CONFIG_MAP,
    PRESET_EXTENSION_SPEC_NAMES,
    EXTENSION_SPEC_FILENAMES,
    GENERATOR_RULE_FILENAMES,
    EXTENSIONS,
    PACKAGE_NAME),
  imu_frequency_(declare_parameter<int>("imu_frequency", 200)),
  accel_index_(declare_parameter<int>("accel_index", 0)),
  gyro_index_(declare_parameter<int>("gyro_index", 1)),
  iio_buf_size_(declare_parameter<int>("iio_buf_size", 64))
{
  RCLCPP_DEBUG(get_logger(), "[Bmi088Node] Constructor");

  registerSupportedType<nvidia::isaac_ros::nitros::NitrosImu>();

  startNitrosNode();
}

void Bmi088Node::postLoadGraphCallback()
{
  getNitrosContext().setParameterInt32(
    "bmi088_driver", "nvidia::isaac::Bmi088Driver", "accel_frequency", imu_frequency_);
  getNitrosContext().setParameterInt32(
    "bmi088_driver", "nvidia::isaac::Bmi088Driver", "gyro_frequency", imu_frequency_);
  getNitrosContext().setParameterInt32(
    "bmi088_driver", "nvidia::isaac::Bmi088Driver", "accel_index", accel_index_);
  getNitrosContext().setParameterInt32(
    "bmi088_driver", "nvidia::isaac::Bmi088Driver", "gyro_index", gyro_index_);
  getNitrosContext().setParameterInt32(
    "bmi088_driver", "nvidia::isaac::Bmi088Driver", "iio_buf_size", iio_buf_size_);
}

Bmi088Node::~Bmi088Node() {}

}  // namespace imu_bmi088
}  // namespace isaac_ros
}  // namespace nvidia

RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::imu_bmi088::Bmi088Node)
