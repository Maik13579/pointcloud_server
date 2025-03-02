//From: https://gitlab.kitware.com/keu-computervision/slam/-/blob/feat/ROS2/slam_lib/include/LidarSlam/Enums.h?ref_type=heads
//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Nicolas Cadart (Kitware SAS)
// Creation date: 2020-11-10
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
//==============================================================================
// Edited to better fit the scope of this projet

#pragma once

#include <vector>
#include <string>
#include <map>

namespace LidarSlam
{

//------------------------------------------------------------------------------
//! How to downsample the map
// A voxel grid is used and various downsampling modes
// are possible to select the remaining point in each voxel
enum class SamplingMode
{
  //! Use the first point acquired
  //! Useful for performances issues
  FIRST = 0,

  //! Use the last point acquired
  //! Useful in dynamic environments
  LAST = 1,

  //! Use the point with maximum intensity
  //! The max intensity points can be the most acurate
  MAX_INTENSITY = 2,

  //! Use the closest point to the voxel center
  //! This allows the most uniform sampling but can be biased
  CENTER_POINT = 3,

  //! Use the centroid of the voxel
  //! This smoothes the points (can be useful for planes)
  //! /!\ The sampling process is longer
  CENTROID = 4
};


} // end of LidarSlam namespace
