#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace voxblox_eval {

class Evaluator {
 public:
  Evaluator(ros::NodeHandle& nh);
  void referenceMapCallback(const pcl::PointCloud<pcl::PointXYZRGB>& msg);
  void staticMapCallback(const pcl::PointCloud<pcl::PointXYZRGB>& msg);
  void dynamicMapCallback(const pcl::PointCloud<pcl::PointXYZRGB>& msg);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber reference_map_sub_;
  ros::Subscriber static_map_sub_;
  ros::Subscriber dynamic_map_sub_;

  pcl::PointCloud<pcl::PointXYZRGB> reference_map_;
  pcl::PointCloud<pcl::PointXYZRGB> static_map_;
  pcl::PointCloud<pcl::PointXYZRGB> dynamic_map_;
  pcl::PointCloud<pcl::PointXYZRGB> seen_points_map_;

  unsigned long int total_static_points_;
  unsigned long int found_static_points_;
  unsigned long int not_found_static_points_;
  float found_static_share_;
  float not_found_static_share_;
  float max_distance_threshold_ = 0.3;

  unsigned long int total_dynamic_points_;
  unsigned long int found_dynamic_points_;
  unsigned long int not_found_dynamic_points_;
  float found_dynamic_share_;
  float not_found_dynamic_share_;
};

}