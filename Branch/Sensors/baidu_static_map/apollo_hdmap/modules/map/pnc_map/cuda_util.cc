/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/map/pnc_map/cuda_util.h"
#include <limits>

#include "modules/common/log.h"

namespace apollo {
namespace pnc_map {

CudaNearestSegment::CudaNearestSegment() {
  segments_ = nullptr;
}

CudaNearestSegment::~CudaNearestSegment() {
  segments_ = nullptr;
}

double distance_square(const apollo::common::math::LineSegment2d& seg, 
    double x, double y) {
  double x1x = x - seg.start().x();
  double y1y = y - seg.start().y();
  double x1x2 = seg.end().x() - seg.start().x();
  double y1y2 = seg.end().y() - seg.start().y();
  double dot = x1x * x1x2 + y1y * y1y2;
  if (dot < 0) {
    return x1x * x1x + y1y * y1y;
  } else if (dot > x1x2 * x1x2 + y1y2 * y1y2) {
    double x2x = x - seg.start().x();
    double y2y = y - seg.start().y();
    return x2x * x2x + y2y * y2y;
  } else {
    double prod = x1x * y1y2 - y1y * x1x2;
    return prod * prod;
  }
}

bool CudaNearestSegment::UpdateLineSegment(
    const std::vector<apollo::common::math::LineSegment2d>& segments) {
  segments_ = &segments;
  size_ = segments.size();
  return true;
}

int CudaNearestSegment::FindNearestSegment(double x, double y) {
  double dis = std::numeric_limits<double>::max();
  int min_index = 0;
  for (auto i=0; i<size_; i++) {
    double tmp = distance_square(segments_->at(i), x, y);
    if (tmp < dis) {
      dis = tmp;
      min_index = i;
    }
  }
  return min_index;
}

}  // namespace pnc_map
}  // namespace apollo
