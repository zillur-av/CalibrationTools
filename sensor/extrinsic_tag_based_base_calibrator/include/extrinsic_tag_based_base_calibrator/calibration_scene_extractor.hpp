// Copyright 2022 Tier IV, Inc.
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

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CALIBRATION_SCENE_EXTRACTOR_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CALIBRATION_SCENE_EXTRACTOR_HPP_

#include <extrinsic_tag_based_base_calibrator/apriltag_detector.hpp>
#include <extrinsic_tag_based_base_calibrator/types.hpp>

#include <string>
#include <vector>

namespace extrinsic_tag_based_base_calibrator
{

class CalibrationSceneExtractor
{
public:
  explicit CalibrationSceneExtractor(
    const ApriltagParameters & apriltag_parameters, bool debug = true)
  : calibration_sensor_detector_(apriltag_parameters),
    external_camera_detector_(apriltag_parameters),
    debug_(debug)
  {
  }

  /*!
   * Sets the intrinsics of the calibration camera prior to process a scene to include the 3d poses
   * of the tags
   * @param[in] parameters The intrinsics of the calibration camera
   */
  void setCalibrationSensorIntrinsics(IntrinsicParameters & parameters);

  /*!
   * Sets the intrinsics of the external camera prior to process a scene to include the 3d poses of
   * the tags
   * @param[in] parameters The intrinsics of the external camera
   */
  void setExternalCameraIntrinsics(IntrinsicParameters & parameters);

  /*!
   * Sets the side-to-side size of the waypoint tags
   * @param[in] parameters The size of the waypoint size
   */
  void setWaypointTagSize(double size);

  /*!
   * Sets the side-to-side size of the wheel tags
   * @param[in] parameters The size of the wheel size
   */
  void setWheelTagSize(double size);

  /*!
   * Sets the side-to-side size of the ground tags
   * @param[in] parameters The size of the ground size
   */
  void setGroundTagSize(double size);

  /*!
   * Sets tag ids of the waypoint tags
   * @param[in] ids A vector containing the id from the waypoint tags
   */
  void setWaypointTagIds(const std::vector<int> & ids);

  /*!
   * Sets tag id of the left wheel
   * @param[in] id A vector containing the id from the waypoint tags
   */
  void setLeftWheelTagId(int ids);

  /*!
   * Sets tag ids of the right wheel
   * @param[in] id A vector containing the id from the waypoint tags
   */
  void setRightWheelTagId(int id);

  /*!
   * Sets tag ids of the ground tags
   * @param[in] ids A vector containing the id from the waypoint tags
   */
  void setGroundTagIds(const std::vector<int> & ids);

  /*!
   * Process a scene, rectifying the images in the scene and obtaining the detections and 3d poses
   * @param[in] calibration_sensor_image_name The single image of the scene corresponding to the
   * calibration camera
   * @param[in] external_camera_image_names The vector containing the images of the scene
   * corresponding to the external camera
   */
  CalibrationScene processScene(
    std::string calibration_sensor_image_name,
    std::vector<std::string> external_camera_image_names);

protected:
  ApriltagDetector calibration_sensor_detector_;
  ApriltagDetector external_camera_detector_;

  double waypoint_tag_size_;
  double wheel_tag_size_;
  double ground_tag_size_;

  std::vector<int> waypoint_tag_ids_;
  int left_wheel_tag_id_;
  int right_wheel_tag_id_;
  std::vector<int> ground_tag_ids_;

  IntrinsicParameters calibration_sensor_intrinsics_;
  IntrinsicParameters external_camera_intrinsics_;
  bool debug_;
};

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CALIBRATION_SCENE_EXTRACTOR_HPP_