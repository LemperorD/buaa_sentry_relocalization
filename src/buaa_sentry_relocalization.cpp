// Copyright 2025 LemperorD
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

#include "buaa_sentry_relocalization/buaa_sentry_relocalization.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>

namespace buaa_sentry_relocalization
{

BuaaSentryRelocalizationNode::BuaaSentryRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("buaa_sentry_relocalization", options),
  have_new_cloud_(false),
  running_reg_(false)
{
  // basic parameter
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  // kiss_matcher param
  this->declare_parameter("resolution", 0.2);
  this->declare_parameter("use_quatro", true);
  // small_gicp param
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);

  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("resolution", resolution_);
  this->get_parameter("use_quatro", use_quatro_);
  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "registered_scan", 5, std::bind(&BuaaSentryRelocalizationNode::pcdCallback, this, std::placeholders::_1));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // Load map
  if (!prior_pcd_file_.empty() && loadGlobalMap(prior_pcd_file_)) {
    RCLCPP_INFO(this->get_logger(), "Loaded global map: %zu points", global_map_->size());
  }

  // Init KISS
  kiss_config_ = kiss_matcher::KISSMatcherConfig(resolution_);
  kiss_config_.use_quatro = use_quatro_;
  matcher_ = std::make_unique<kiss_matcher::KISSMatcher>(kiss_config_);
  if (global_map_) {
    target_vec_ = convertCloudToVec(*global_map_);
    matcher_->setTarget(target_vec_);
  }

  // Init small_gicp
  prepareSmallGICPTarget();
}

void BuaaSentryRelocalizationNode::pcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud);

  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    latest_cloud_ = cloud;
    have_new_cloud_.store(true);
  }

  bool expected = false;
  if (running_reg_.compare_exchange_strong(expected, true)) {
    std::thread(&BuaaSentryRelocalization::processingPipeline, this).detach();
  }
}

void BuaaSentryRelocalization::processingPipeline()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (!latest_cloud_) {
      running_reg_.store(false);
      return;
    }
    cloud = latest_cloud_;
    have_new_cloud_.store(false);
  }

  Eigen::Isometry3d initial_guess = Eigen::Isometry3d::Identity();
  bool have_coarse = false;

  source_vec = convertCloudToVec(*cloud);
  auto result = matcher_->match(source_vec);
  if (result.success) {
    initial_guess = isoFromPose(result.pose);
    have_coarse = true;
  }

  if (!have_coarse && has_external_initial_pose_) {
    initial_guess = poseMsgToIso(external_initial_pose_);
    have_coarse = true;
  }

  // small_gicp refinement
  Eigen::Isometry3d final_pose = initial_guess;
  if (use_small_gicp_ && target_ && target_tree_) {
    pcl::PointCloud<pcl::PointCovariance>::Ptr src_cov(
        new pcl::PointCloud<pcl::PointCovariance>());
    *src_cov = small_gicp::voxelgrid_sampling<pcl::PointCloud<pcl::PointXYZ>,
                                             pcl::PointCloud<pcl::PointCovariance>>(*cloud, global_leaf_size_);
    small_gicp::estimate_covariances_omp(*src_cov, num_neighbors_, num_threads_);
    auto src_tree =
        std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(src_cov);
    small_gicp::SmallGICP gicp;
    gicp.setInputTarget(target_, target_tree_);
    gicp.setInputSource(src_cov, src_tree);
    small_gicp::SmallGICP::Params params;
    params.num_threads = num_threads_;
    params.max_correspondence_distance = max_corr_dist_;
    gicp.setParameters(params);
    auto res = gicp.optimize(initial_guess);
    if (res.converged) {
      final_pose = res.transformation;
    }
  }

  publishPose(final_pose);
  running_reg_.store(false);
}

bool BuaaSentryRelocalization::loadGlobalMap(const std::string & file_name)
{
  global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPCDFile(file_name, *global_map_) < 0) return false;
  std::vector<int> idx;
  pcl::removeNaNFromPointCloud(*global_map_, *global_map_, idx);
  return true;
}

void BuaaSentryRelocalization::prepareSmallGICPTarget()
{
  target_ = small_gicp::voxelgrid_sampling<pcl::PointCloud<pcl::PointXYZ>,
                                           pcl::PointCloud<pcl::PointCovariance>>(
      *global_map_, global_leaf_size_);
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);
  target_tree_ =
      std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
          target_, small_gicp::KdTreeBuilderOMP(num_threads_));
}

std::vector<Eigen::Vector3f>
BuaaSentryRelocalization::convertCloudToVec(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  std::vector<Eigen::Vector3f> out;
  out.reserve(cloud.size());
  for (auto & p : cloud) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
      out.emplace_back(p.x, p.y, p.z);
  }
  return out;
}

Eigen::Isometry3d BuaaSentryRelocalization::poseMsgToIso(const geometry_msgs::msg::PoseStamped & msg)
{
  Eigen::Quaterniond q(msg.pose.orientation.w, msg.pose.orientation.x,
                       msg.pose.orientation.y, msg.pose.orientation.z);
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = q.toRotationMatrix();
  iso.translation() =
      Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  return iso;
}

Eigen::Isometry3d BuaaSentryRelocalization::isoFromPose(const kiss_matcher::Pose & p)
{
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  Eigen::Quaterniond q(p.qw, p.qx, p.qy, p.qz);
  iso.linear() = q.toRotationMatrix();
  return iso;
}

void BuaaSentryRelocalization::publishPose(const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp = now();
  ps.header.frame_id = frame_id_;
  ps.pose = tf2::toMsg(tf2::eigenToTransform(pose));
  pose_pub_->publish(ps);

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header = ps.header;
  tf_msg.child_frame_id = child_frame_id_;
  tf_msg.transform = tf2::toMsg(tf2::eigenToTransform(pose));
  tf_broadcaster_->sendTransform(tf_msg);
}

}  // namespace buaa_sentry_relocalization

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(buaa_sentry_relocalization::BuaaSentryRelocalization)
