#ifndef VOXBLOX_INTEGRATOR_MERGE_INTEGRATION_H_
#define VOXBLOX_INTEGRATOR_MERGE_INTEGRATION_H_

#include <algorithm>
#include <utility>
#include <vector>

#include <glog/logging.h>

#include "voxblox/interpolator/interpolator.h"

#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"

namespace voxblox {

static const FloatingPoint kUnitCubeDiagonalLength = std::sqrt(3.0);

template <typename VoxelType>
void mergeVoxelAIntoVoxelB(const VoxelType& voxel_A, VoxelType* voxel_B);

template <>
void mergeVoxelAIntoVoxelB(const TsdfVoxel& voxel_A, TsdfVoxel* voxel_B);

template <>
void mergeVoxelAIntoVoxelB(const OccupancyVoxel& voxel_A,
                           OccupancyVoxel* voxel_B);

template <typename VoxelType>
void mergeBlockAIntoBlockB(const Block<VoxelType>& block_A,
                           Block<VoxelType>* block_B) {
  CHECK_EQ(block_A.voxel_size(), block_B->voxel_size());
  CHECK_EQ(block_A.voxels_per_side(), block_B->voxels_per_side());
  CHECK_NOTNULL(block_B);

  if (!block_A.has_data()) {
    return;
  } else {
    block_B->has_data() = true;
    block_B->updated() = true;

    for (IndexElement voxel_idx = 0;
         voxel_idx < static_cast<IndexElement>(block_B->num_voxels());
         ++voxel_idx) {
      mergeVoxelAIntoVoxelB<VoxelType>(
          block_A.getVoxelByLinearIndex(voxel_idx),
          &(block_B->getVoxelByLinearIndex(voxel_idx)));
    }
  }
}

// Merges layers, when the voxel or block size differs resampling occurs.
template <typename VoxelType>
void mergeLayerAintoLayerB(const Layer<VoxelType>& layer_A,
                           Layer<VoxelType>* layer_B) {
  CHECK_NOTNULL(layer_B);
  // if voxel layout is different resample layer A to match B
  const Layer<VoxelType>* layer_A_ptr;
  Layer<VoxelType> layer_A_resampled(layer_B->voxel_size(),
                                     layer_B->voxels_per_side());

  if ((layer_A.voxel_size() != layer_B->voxel_size()) ||
      (layer_A.voxels_per_side() != layer_B->voxels_per_side())) {
    resampleLayer(layer_A, &layer_A_resampled);
    layer_A_ptr = &layer_A_resampled;
  } else {
    layer_A_ptr = &layer_A;
  }

  BlockIndexList block_idx_list_A;
  layer_A.getAllAllocatedBlocks(&block_idx_list_A);

  for (const BlockIndex& block_idx : block_idx_list_A) {
    typename Block<VoxelType>::ConstPtr block_A_ptr =
        layer_A_ptr->getBlockPtrByIndex(block_idx);
    typename Block<VoxelType>::Ptr block_B_ptr =
        layer_B->getBlockPtrByIndex(block_idx);

    if (!block_B_ptr) {
      block_B_ptr = layer_B->allocateBlockPtrByIndex(block_idx);
    }

    if ((block_A_ptr != nullptr) && (block_B_ptr != nullptr)) {
      mergeBlockAIntoBlockB(*block_A_ptr, block_B_ptr.get());
    }
  }
}

// Performs a 3D transformation on layer A before merging it. See
// transformLayer for details
template <typename VoxelType>
void mergeLayerAintoLayerB(const Layer<VoxelType>& layer_A,
                           const Transformation& T_B_A,
                           Layer<VoxelType>* layer_B,
                           bool use_naive_method = false) {
  Layer<VoxelType> layer_A_transformed(layer_B->voxel_size(),
                                       layer_B->voxels_per_side());

  if (use_naive_method) {
    naiveTransformLayer(layer_A, T_B_A, &layer_A_transformed);
  } else {
    transformLayer(layer_A, T_B_A, &layer_A_transformed);
  }

  mergeLayerAintoLayerB(layer_A_transformed, layer_B);
}

// copies the information stored in layer_in into layer_out resampling the
// data so that it fits the voxel and block size of the output layer
template <typename VoxelType>
void resampleLayer(const Layer<VoxelType>& layer_in,
                   Layer<VoxelType>* layer_out) {
  CHECK_NOTNULL(layer_out);
  transformLayer(layer_in, Transformation(), layer_out);
}

// Similar to transformLayer in functionality, however the system only makes
// use of the forward transform and nearest neighbor interpolation. This will
// result in artifacts and other issues in the result, however it should be
// several orders of magnitude faster.
template <typename VoxelType>
void naiveTransformLayer(const Layer<VoxelType>& layer_in,
                         const Transformation& T_out_in,
                         Layer<VoxelType>* layer_out) {
  BlockIndexList block_idx_list_in;
  layer_in.getAllAllocatedBlocks(&block_idx_list_in);

  Interpolator<VoxelType> interpolator(&layer_in);

  for (const BlockIndex& block_idx : block_idx_list_in) {
    const Block<VoxelType>& input_block = layer_in.getBlockByIndex(block_idx);

    for (IndexElement input_linear_voxel_idx = 0;
         input_linear_voxel_idx <
         static_cast<IndexElement>(input_block.num_voxels());
         ++input_linear_voxel_idx) {
      // find voxel centers location in the output
      const Point voxel_center =
          T_out_in *
          input_block.computeCoordinatesFromLinearIndex(input_linear_voxel_idx);

      const VoxelIndex global_output_voxel_idx =
          getGridIndexFromPoint(voxel_center, layer_out->voxel_size_inv());

      // allocate it in the output
      typename Block<VoxelType>::Ptr output_block =
          layer_out->allocateBlockPtrByIndex(getBlockIndexFromGlobalVoxelIndex(
              global_output_voxel_idx, layer_out->voxels_per_side_inv()));

      if (output_block == nullptr) {
        std::cerr << "invalid block" << std::endl;
      }

      // get the output voxel
      VoxelType& output_voxel =
          output_block->getVoxelByVoxelIndex(getLocalFromGlobalVoxelIndex(
              global_output_voxel_idx, layer_out->voxels_per_side()));

      if (interpolator.getVoxel(voxel_center, &output_voxel, false)) {
        output_block->has_data() = true;
      }
    }
  }
}

// Performs a 3D transform on the input layer and writes the results to the
// output layer. During the transformation resampling occurs so that the voxel
// and block size of the input and output layer can differ.
template <typename VoxelType>
void transformLayer(const Layer<VoxelType>& layer_in,
                    const Transformation& T_out_in,
                    Layer<VoxelType>* layer_out) {
  CHECK_NOTNULL(layer_out);

  // first mark all the blocks in the output layer that may be filled by the
  // input layer (we are conservative here approximating the input blocks as
  // spheres of diameter sqrt(3)*block_size)
  IndexSet block_idx_set;

  BlockIndexList block_idx_list_in;
  layer_in.getAllAllocatedBlocks(&block_idx_list_in);

  for (const BlockIndex& block_idx : block_idx_list_in) {
    const Point c_in =
        getCenterPointFromGridIndex(block_idx, layer_in.block_size());

    // forwards transform of center
    const Point c_out = T_out_in * c_in;

    // Furthest center point of neighboring blocks.
    FloatingPoint offset =
        kUnitCubeDiagonalLength * layer_in.block_size() * 0.5;

    // Add index of all blocks in range to set.
    for (FloatingPoint x = c_out.x() - offset; x < c_out.x() + offset;
         x += layer_out->block_size()) {
      for (FloatingPoint y = c_out.y() - offset; y < c_out.y() + offset;
           y += layer_out->block_size()) {
        for (FloatingPoint z = c_out.z() - offset; z < c_out.z() + offset;
             z += layer_out->block_size()) {
          const Point current_center_out = Point(x, y, z);
          BlockIndex current_idx = getGridIndexFromPoint(
              current_center_out, 1.0f / layer_out->block_size());
          block_idx_set.insert(current_idx);
        }
      }
    }
  }

  // get inverse transform
  const Transformation T_in_out = T_out_in.inverse();

  Interpolator<VoxelType> interpolator(&layer_in);

  // we now go through all the blocks in the output layer and interpolate the
  // input layer at the center of each output voxel position
  for (const BlockIndex& block_idx : block_idx_set) {
    typename Block<VoxelType>::Ptr block =
        layer_out->allocateBlockPtrByIndex(block_idx);

    for (IndexElement voxel_idx = 0;
         voxel_idx < static_cast<IndexElement>(block->num_voxels());
         ++voxel_idx) {
      VoxelType& voxel = block->getVoxelByLinearIndex(voxel_idx);

      // find voxel centers location in the input
      const Point voxel_center =
          T_in_out * block->computeCoordinatesFromLinearIndex(voxel_idx);

      // interpolate voxel
      if (interpolator.getVoxel(voxel_center, &voxel, true)) {
        block->has_data() = true;

        // if interpolated value fails use nearest
      } else if (interpolator.getVoxel(voxel_center, &voxel, false)) {
        block->has_data() = true;
      }
    }

    if (!block->has_data()) {
      layer_out->removeBlock(block_idx);
    }
  }
}

typedef std::pair<voxblox::Layer<voxblox::TsdfVoxel>::Ptr,
                  voxblox::Layer<voxblox::TsdfVoxel>::Ptr>
    AlignedObjectAndErrorLayer;
typedef std::vector<AlignedObjectAndErrorLayer> AlignedObjectAndErrorLayers;

template <typename VoxelType>
void evaluateLayerRmseAtPoses(
    const utils::VoxelEvaluationMode& voxel_evaluation_mode,
    const Layer<VoxelType>& layer,
    const Layer<VoxelType>& merged_object_layer_O,
    const std::vector<Transformation>& transforms_W_O,
    std::vector<utils::VoxelEvaluationDetails>* voxel_evaluation_details_vector,
    std::vector<std::pair<typename voxblox::Layer<VoxelType>::Ptr,
                          typename voxblox::Layer<VoxelType>::Ptr>>*
        aligned_objects_and_error_layers = nullptr) {
  CHECK_NOTNULL(voxel_evaluation_details_vector);
  // Check if world TSDF layer agrees with merged object at all object poses.

  for (size_t i = 0u; i < transforms_W_O.size(); ++i) {
    const Transformation& transform_W_O = transforms_W_O[i];
    typename Layer<VoxelType>::Ptr merged_object_layer_W(
        new Layer<VoxelType>(merged_object_layer_O.voxel_size(),
                             merged_object_layer_O.voxels_per_side()));

    Layer<VoxelType>* error_layer = nullptr;
    if (aligned_objects_and_error_layers != nullptr) {
      CHECK_EQ(aligned_objects_and_error_layers->size(), transforms_W_O.size());

      // Initialize and get ptr to error layer to fill out later.
      (*aligned_objects_and_error_layers)[i].second =
          typename voxblox::Layer<VoxelType>::Ptr(new voxblox::Layer<VoxelType>(
              layer.voxel_size(), layer.voxels_per_side()));
      error_layer = (*aligned_objects_and_error_layers)[i].second.get();

      // Store the aligned object as well.
      (*aligned_objects_and_error_layers)[i].first = merged_object_layer_W;
    }

    // Transform merged object into the world frame.
    transformLayer<VoxelType>(merged_object_layer_O, transform_W_O,
                              merged_object_layer_W.get());

    utils::VoxelEvaluationDetails voxel_evaluation_details;
    // Evaluate the RMSE of the merged object layer in the world layer.
    utils::evaluateLayersRmse(layer, *merged_object_layer_W,
                              voxel_evaluation_mode, &voxel_evaluation_details,
                              error_layer);
    voxel_evaluation_details_vector->push_back(voxel_evaluation_details);
  }
}

template <typename VoxelType>
void evaluateLayerRmseAtPoses(
    const utils::VoxelEvaluationMode& voxel_evaluation_mode,
    const Layer<VoxelType>& layer,
    const Layer<VoxelType>& merged_object_layer_O,
    const std::vector<Eigen::Matrix<float, 4, 4>,
                      Eigen::aligned_allocator<Eigen::Matrix<float, 4, 4>>>&
        transforms_W_O,
    std::vector<utils::VoxelEvaluationDetails>* voxel_evaluation_details_vector,
    std::vector<std::pair<typename voxblox::Layer<VoxelType>::Ptr,
                          typename voxblox::Layer<VoxelType>::Ptr>>*
        aligned_objects_and_error_layers = nullptr) {
  CHECK_NOTNULL(voxel_evaluation_details_vector);
  std::vector<Transformation> kindr_transforms_W_O;
  for (const Eigen::Matrix<float, 4, 4>& transform_W_O : transforms_W_O) {
    kindr_transforms_W_O.emplace_back(transform_W_O);
  }
  evaluateLayerRmseAtPoses(
      voxel_evaluation_mode, layer, merged_object_layer_O, kindr_transforms_W_O,
      voxel_evaluation_details_vector, aligned_objects_and_error_layers);
}

}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_MERGE_INTEGRATION_H_
