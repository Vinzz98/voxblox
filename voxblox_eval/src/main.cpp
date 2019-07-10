#include "main.h"

#include <ros/time.h>
#include <pcl_ros/point_cloud.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "adero_ifm3d_processing");
  ros::NodeHandle nh("~");
  voxblox_eval::Evaluator Evaluator(nh);
  ROS_INFO("main here");
  ros::spin();
  return 0;
}

namespace voxblox_eval {
  Evaluator::Evaluator(ros::NodeHandle& nh) {
    nh_ = nh;
    reference_map_sub_ = nh_.subscribe("/ref_map", 1, &Evaluator::referenceMapCallback, this);
    static_map_sub_ = nh_.subscribe("/voxblox_node/static_map_pointcloud", 1, &Evaluator::staticMapCallback, this);
    dynamic_map_sub_ = nh_.subscribe("/voxblox_node/dynamic_map_pointcloud", 1, &Evaluator::dynamicMapCallback, this);

    total_static_points_ = 0;
    found_static_points_ = 0;
    not_found_static_points_ = 0;
    found_static_share_ = 0;
    not_found_static_share_ = 0;
    total_dynamic_points_ = 0;
    found_dynamic_points_ = 0;
    not_found_dynamic_points_ = 0;
    found_dynamic_share_ = 0;
    not_found_dynamic_share_ = 0;

    ROS_INFO("total static points = %i", total_static_points_);
  }

  void Evaluator::referenceMapCallback(const pcl::PointCloud<pcl::PointXYZRGB>& msg) {
    reference_map_ = msg;
    ROS_INFO("Got reference map");
  }

  void Evaluator::staticMapCallback(const pcl::PointCloud<pcl::PointXYZRGB>& msg) {
    int total_static_points = 0;
    for (pcl::PointXYZRGB point : msg) {
      float min_dist = 1000;
      pcl::PointXYZRGB min_dist_ref_point;
      total_static_points++;
      bool found = false;
      bool already_seen = false;
      for (pcl::PointXYZRGB reference_point : reference_map_) {
        float point_dist = sqrt((point.x - reference_point.x)*(point.x - reference_point.x) + (point.y - reference_point.y)*(point.y - reference_point.y));
        if (point_dist < min_dist) {
          min_dist = point_dist;
          min_dist_ref_point = reference_point;
        }
      }
      if (min_dist <= max_distance_threshold_){
        for (pcl::PointXYZRGB seen_point : seen_points_map_) {
          if (seen_point.x == min_dist_ref_point.x && seen_point.y == min_dist_ref_point.y) {
            already_seen = true;
          }
        }
        if (!already_seen) seen_points_map_.push_back(min_dist_ref_point);
        found = true;
        //ROS_INFO("found");
      }
      if (found) found_static_points_++;
      else not_found_static_points_++;
    }
    total_static_points_ += total_static_points;

    found_static_share_ = (float) found_static_points_ / (float) total_static_points_;
    not_found_static_share_ = (float) not_found_static_points_ / (float) total_static_points_;
    ROS_INFO("found static share: %f", found_static_share_);
    ROS_INFO("not found static share: %f", not_found_static_share_);
    ROS_INFO("reference map size = %i", reference_map_.size());
    ROS_INFO("number of seen points = %i", seen_points_map_.size());
  }

  void Evaluator::dynamicMapCallback(const pcl::PointCloud<pcl::PointXYZRGB>& msg) {
    int total_dynamic_points = 0;
    for (pcl::PointXYZRGB point : msg) {
      total_dynamic_points++;
      float min_dist = 1000;
      bool found = false;
      for (pcl::PointXYZRGB reference_point : reference_map_) {
        float point_dist = sqrt((point.x - reference_point.x)*(point.x - reference_point.x) + (point.y - reference_point.y)*(point.y - reference_point.y));
        if (point_dist < min_dist) {
          min_dist = point_dist;
        }
      }
      if (min_dist <= max_distance_threshold_) {
        found = true;
      }
      if (found) found_dynamic_points_++;
      else not_found_dynamic_points_++;
    }
    total_dynamic_points_ += total_dynamic_points;
    found_dynamic_share_ = (float) found_dynamic_points_ / (float) total_dynamic_points_;
    not_found_dynamic_share_ = (float) not_found_dynamic_points_ / (float) total_dynamic_points_;
    ROS_INFO("dynamic points being static share: %f", found_dynamic_share_);
    ROS_INFO("dynamic points not in static share: %f", not_found_dynamic_share_);
  }

}