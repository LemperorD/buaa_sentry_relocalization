#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <mutex>
#include <atomic>
#include <memory>

// external libs
#include <kiss_matcher/KISSMatcher.hpp>
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"

namespace buaa_sentry_relocalization
{

class BuaaSentryRelocalizationNode : public rclcpp::Node
{
public:
  explicit BuaaSentryRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  // ==== Callbacks ====
  void pcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void initialPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void processingPipeline();

  // ==== Helper ====
  bool loadGlobalMap(const std::string & file_name);
  void prepareSmallGICPTarget();
  std::vector<Eigen::Vector3f> convertCloudToVec(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  Eigen::Isometry3d poseMsgToIso(const geometry_msgs::msg::PoseStamped & msg);
  Eigen::Isometry3d isoFromPose(const kiss_matcher::Pose & p);
  void publishPose(const Eigen::Isometry3d & pose);

  // ==== ROS Interfaces ====
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ==== basic variable ====
  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  
  // ==== kiss_matcher variable ====
  float resolution_;
  bool use_quatro_;
  std::vector<Eigen::Vector3f> target_vec;
  std::vector<Eigen::Vector3f> source_vec;
  std::vector<int> src_indices;
  std::vector<int> tgt_indices;
  kiss_matcher::KISSMatcherConfig config_;
  std::unique_ptr<kiss_matcher::KISSMatcher> matcher_;
  kiss_matcher::RegistrationSolution solution_;

  // ==== small_gicp variable ====
  int num_threads_;
  int num_neighbors_;
  float global_leaf_size_;
  float registered_leaf_size_;
  float max_dist_sq_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
  std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    register_;

  // ==== Runtime ====
  std::mutex cloud_mutex_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;
  std::atomic<bool> have_new_cloud_;
  std::atomic<bool> running_reg_;
  Eigen::Isometry3d last_result_;

  geometry_msgs::msg::PoseStamped external_initial_pose_;
  bool has_external_initial_pose_ = false;
};

}  // namespace buaa_sentry_relocalization