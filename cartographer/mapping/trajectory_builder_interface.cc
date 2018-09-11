/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/trajectory_builder_interface.h"

#include "cartographer/mapping/internal/2d/local_trajectory_builder_options_2d.h"
#include "cartographer/mapping/internal/3d/local_trajectory_builder_options_3d.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {
namespace {

void PopulatePureLocalizationTrimmerOptions(
    proto::TrajectoryBuilderOptions* const trajectory_builder_options,
    common::LuaParameterDictionary* const parameter_dictionary) {
  constexpr char kDictionaryKey[] = "pure_localization_trimmer";
  if (!parameter_dictionary->HasKey(kDictionaryKey)) return;

  auto options_dictionary = parameter_dictionary->GetDictionary(kDictionaryKey);
  auto* options =
      trajectory_builder_options->mutable_pure_localization_trimmer();
  options->set_max_submaps_to_keep(
      options_dictionary->GetInt("max_submaps_to_keep"));
}

void PopulateRelativePoseOptions(
    proto::InitialTrajectoryPose* const initial_trajectory_pose_options,
    common::LuaParameterDictionary* const parameter_dictionary) {
  constexpr char kDictionaryKey[] = "relative_pose";
  if (!parameter_dictionary->HasKey(kDictionaryKey)) return;

  auto relative_pose_dictionary = parameter_dictionary->GetDictionary(kDictionaryKey);
  auto* relative_pose_options =
		  initial_trajectory_pose_options->mutable_relative_pose();

  if (relative_pose_dictionary->HasKey("translation"))
  {
	  auto translation_dictionary = relative_pose_dictionary->GetDictionary("translation");
	  auto* translation_options = relative_pose_options->mutable_translation();

	  translation_options->set_x(translation_dictionary->GetDouble("x"));
	  translation_options->set_y(translation_dictionary->GetDouble("y"));
	  translation_options->set_z(translation_dictionary->GetDouble("z"));
  }

  if (relative_pose_dictionary->HasKey("rotation"))
  {
	  auto dictionary = relative_pose_dictionary->GetDictionary("rotation");
	  auto* options = relative_pose_options->mutable_rotation();

	  double roll = dictionary->GetDouble("roll");
	  double pitch = dictionary->GetDouble("pitch");
	  double yaw = dictionary->GetDouble("yaw");

	  Eigen::Quaterniond quaternion = cartographer::transform::RollPitchYaw(roll, pitch, yaw);
	  options->set_x(quaternion.x());
	  options->set_y(quaternion.y());
	  options->set_z(quaternion.z());
	  options->set_w(quaternion.w());
  }
}

void PopulateInitialTrajectoryPoseOptions(
    proto::TrajectoryBuilderOptions* const trajectory_builder_options,
    common::LuaParameterDictionary* const parameter_dictionary) {
  constexpr char kDictionaryKey[] = "initial_trajectory_pose";
  if (!parameter_dictionary->HasKey(kDictionaryKey)) return;

  auto options_dictionary = parameter_dictionary->GetDictionary(kDictionaryKey);
  auto* options =
      trajectory_builder_options->mutable_initial_trajectory_pose();

  options->set_timestamp(options_dictionary->GetNonNegativeInt("timestamp"));
  options->set_to_trajectory_id(options_dictionary->GetInt("to_trajectory_id"));

  PopulateRelativePoseOptions(options, &(*options_dictionary));
}

}  // namespace

proto::TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::TrajectoryBuilderOptions options;
  *options.mutable_trajectory_builder_2d_options() =
      CreateLocalTrajectoryBuilderOptions2D(
          parameter_dictionary->GetDictionary("trajectory_builder_2d").get());
  *options.mutable_trajectory_builder_3d_options() =
      CreateLocalTrajectoryBuilderOptions3D(
          parameter_dictionary->GetDictionary("trajectory_builder_3d").get());
  options.set_collate_fixed_frame(
      parameter_dictionary->GetBool("collate_fixed_frame"));
  options.set_collate_landmarks(
      parameter_dictionary->GetBool("collate_landmarks"));
  PopulatePureLocalizationTrimmerOptions(&options, parameter_dictionary);
  PopulateInitialTrajectoryPoseOptions(&options, parameter_dictionary);
  return options;
}

proto::SensorId ToProto(const TrajectoryBuilderInterface::SensorId& sensor_id) {
  proto::SensorId sensor_id_proto;
  switch (sensor_id.type) {
    case TrajectoryBuilderInterface::SensorId::SensorType::RANGE:
      sensor_id_proto.set_type(proto::SensorId::RANGE);
      break;
    case TrajectoryBuilderInterface::SensorId::SensorType::IMU:
      sensor_id_proto.set_type(proto::SensorId::IMU);
      break;
    case TrajectoryBuilderInterface::SensorId::SensorType::ODOMETRY:
      sensor_id_proto.set_type(proto::SensorId::ODOMETRY);
      break;
    case TrajectoryBuilderInterface::SensorId::SensorType::FIXED_FRAME_POSE:
      sensor_id_proto.set_type(proto::SensorId::FIXED_FRAME_POSE);
      break;
    case TrajectoryBuilderInterface::SensorId::SensorType::LANDMARK:
      sensor_id_proto.set_type(proto::SensorId::LANDMARK);
      break;
    case TrajectoryBuilderInterface::SensorId::SensorType::LOCAL_SLAM_RESULT:
      sensor_id_proto.set_type(proto::SensorId::LOCAL_SLAM_RESULT);
      break;
    default:
      LOG(FATAL) << "Unsupported sensor type.";
  }
  sensor_id_proto.set_id(sensor_id.id);
  return sensor_id_proto;
}

TrajectoryBuilderInterface::SensorId FromProto(
    const proto::SensorId& sensor_id_proto) {
  TrajectoryBuilderInterface::SensorId sensor_id;
  switch (sensor_id_proto.type()) {
    case proto::SensorId::RANGE:
      sensor_id.type = TrajectoryBuilderInterface::SensorId::SensorType::RANGE;
      break;
    case proto::SensorId::IMU:
      sensor_id.type = TrajectoryBuilderInterface::SensorId::SensorType::IMU;
      break;
    case proto::SensorId::ODOMETRY:
      sensor_id.type =
          TrajectoryBuilderInterface::SensorId::SensorType::ODOMETRY;
      break;
    case proto::SensorId::FIXED_FRAME_POSE:
      sensor_id.type =
          TrajectoryBuilderInterface::SensorId::SensorType::FIXED_FRAME_POSE;
      break;
    case proto::SensorId::LANDMARK:
      sensor_id.type =
          TrajectoryBuilderInterface::SensorId::SensorType::LANDMARK;
      break;
    case proto::SensorId::LOCAL_SLAM_RESULT:
      sensor_id.type =
          TrajectoryBuilderInterface::SensorId::SensorType::LOCAL_SLAM_RESULT;
      break;
    default:
      LOG(FATAL) << "Unsupported sensor type.";
  }
  sensor_id.id = sensor_id_proto.id();
  return sensor_id;
}

}  // namespace mapping
}  // namespace cartographer
