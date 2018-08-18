/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */
#ifndef VOXBLOX_INTEGRATOR_ICP_H_
#define VOXBLOX_INTEGRATOR_ICP_H_

#include <algorithm>
#include <thread>

#include "voxblox/core/block_hash.h"
#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/utils/approx_hash_array.h"

namespace voxblox {

class ICP {
 public:
  struct Config {
    int iterations = 5;
    FloatingPoint min_match_ratio = 0.5;
    FloatingPoint voxel_size_inv = 10.0;
    size_t num_threads = std::thread::hardware_concurrency();
  };

  ICP(Config config);

  bool runICP(const Layer<TsdfVoxel> *tsdf_layer, const Pointcloud &points,
              const Transformation &T_in, Transformation *T_out);

 private:
  static bool getTransformFromCorrelation(const PointsMatrix &src_demean,
                                          const Point &src_center,
                                          const PointsMatrix &tgt_demean,
                                          const Point &tgt_center,
                                          Transformation *T);

  static bool getTransformFromMatchedPoints(const PointsMatrix &src,
                                            const PointsMatrix &tgt,
                                            Transformation *T);

  void matchPoints(const Layer<TsdfVoxel> *tsdf_layer,
                   const AlignedVector<Pointcloud> &points,
                   const Transformation &T, PointsMatrix *src,
                   PointsMatrix *tgt) const;

  void calcMatches(
      const Layer<TsdfVoxel> *tsdf_layer,
      const AlignedVector<Pointcloud> &points,
      AlignedVector<std::shared_ptr<ThreadSafeIndex>> *index_getters,
      const Transformation &T, Pointcloud *src, Pointcloud *tgt) const;

  void downsampleCloud(const Pointcloud &points, ThreadSafeIndex *index_getter,
                       Pointcloud *points_downsampled);

  bool stepICP(const Layer<TsdfVoxel> *tsdf_layer,
               const AlignedVector<Pointcloud> &points,
               const Transformation &T_in, Transformation *T_out);

  Config config_;

  // uses 2^20 bytes (8 megabytes) of ram per tester
  // A testers false negative rate is inversely proportional to its size
  static constexpr size_t masked_bits_ = 20;
  // only needs to zero the above 8mb of memory once every 10,000 scans
  // (uses an additional 80,000 bytes)
  static constexpr size_t full_reset_threshold_ = 10000;

  // used for fast downsampling
  ApproxHashSet<masked_bits_, full_reset_threshold_, GlobalIndex, LongIndexHash>
      voxel_approx_set_;
};

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_ICP_H_