#ifndef _ICP_SLAM_
#define _ICP_SLAM_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/registration/gicp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>

#include <icp_slam/SaveMap.h>
#include <icp_slam/data_struct.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.h>

class ICPSlam
{
  using PointType = pcl::PointXYZ;

public:
  ICPSlam();
  ~ICPSlam() = default;

private:
  void limitCloudScanData(
    const pcl::PointCloud<PointType>::Ptr input_ptr, const pcl::PointCloud<PointType>::Ptr& output_ptr,
    const double min_scan_range, const double max_scan_range);
  void downsample(const pcl::PointCloud<PointType>::Ptr input_ptr, const pcl::PointCloud<PointType>::Ptr& output_ptr);

  Pose getCurrentPose();

  void imuCorrect(const ros::Time current_scan_time);

  void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& input_points_ptr_msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  void imuCorrect(Eigen::Matrix4f &pose, const ros::Time stamp);

  geometry_msgs::TransformStamped getTransform(const std::string target_frame, const std::string source_frame);
  void transformPointCloud(
    pcl::PointCloud<PointType>::Ptr input_ptr, pcl::PointCloud<PointType>::Ptr& output_ptr,
    const std::string target_frame, const std::string source_frame);

  bool saveMapService(icp_slam::SaveMapRequest& req, icp_slam::SaveMapResponse& res);

private:
  ros::NodeHandle nh_{};
  ros::NodeHandle pnh_{ "~" };

  ros::Subscriber points_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Publisher icp_aligned_cloud_publisher_;
  ros::Publisher icp_map_publisher_;
  ros::Publisher icp_pose_publisher_;
  ros::Publisher transform_probability_publisher_;

  ros::ServiceServer save_map_service_;

  Pose icp_pose_;
  Pose previous_pose_;

  Eigen::Matrix4f pose_{ Eigen::Matrix4f::Identity() };

  Eigen::Vector3f imu_rotate_vec_;

  ros::Time previous_scan_time_;

  pcl::PointCloud<PointType>::Ptr map_;
  boost::shared_ptr<pcl::Registration<PointType, PointType>> registration_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster broadcaster_;

  bool use_imu_{false};
  std::string base_frame_id_;

  // rosparam
  double min_scan_range_;
  double max_scan_range_;
  double min_add_scan_shift_;

  // voxel grid filter
  double leaf_size_;

  // config for icp omp
  int max_iteration_;
  int correspondence_randomness_;
  double max_correspondence_distance_;
  double euclidean_fitness_epsilon_;
  double ransac_outlier_rejection_threshold_;
  double transformation_epsilon_;

  sensor_msgs::Imu imu_;
  nav_msgs::Odometry odom_;
};

#endif
