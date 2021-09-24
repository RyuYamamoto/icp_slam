#include <icp_slam/icp_slam.h>
#include <icp_slam/icp_slam_utils.h>

ICPSlam::ICPSlam() : tf_listener_(tf_buffer_)
{
  pnh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
  pnh_.param<double>("min_scan_range", min_scan_range_, 5.0);
  pnh_.param<double>("max_scan_range", max_scan_range_, 200.0);
  pnh_.param<double>("min_add_scan_shift", min_add_scan_shift_, 1.0);
  pnh_.param<double>("leaf_size", leaf_size_, 2.0);
  pnh_.param<bool>("use_imu", use_imu_, false);

  pnh_.param<int>("correspondence_randomness", correspondence_randomness_, 20);
  pnh_.param<double>("transformation_epsilon", transformation_epsilon_, 0.01);
  pnh_.param<double>("max_correspondence_distance", max_correspondence_distance_, 1.0);
  pnh_.param<double>("euclidean_fitness_epsilon", euclidean_fitness_epsilon_, 0.1);
  pnh_.param<double>(
    "ransac_outlier_rejection_threshold", ransac_outlier_rejection_threshold_, 1.0);
  pnh_.param<int>("max_iteration", max_iteration_, 20);

  const std::string registration_type = pnh_.param<std::string>("registration_type", "FAST_GICP");
  if (registration_type == "FAST_GICP") {
    boost::shared_ptr<fast_gicp::FastGICP<PointType, PointType>> fast_gicp(
      new fast_gicp::FastGICP<PointType, PointType>);
    const int num_thread = pnh_.param<int>("gicp_num_thread", 0);
    if (0 < num_thread) fast_gicp->setNumThreads(num_thread);
    fast_gicp->setCorrespondenceRandomness(correspondence_randomness_);
    registration_ = fast_gicp;
  } else {
    boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<PointType, PointType>> gicp(
      new pcl::GeneralizedIterativeClosestPoint<PointType, PointType>);
    registration_ = gicp;
  }
  registration_->setMaximumIterations(max_iteration_);
  registration_->setTransformationEpsilon(transformation_epsilon_);
  registration_->setMaxCorrespondenceDistance(max_correspondence_distance_);

  // create subscriber
  points_subscriber_ = nh_.subscribe("points_raw", 1000, &ICPSlam::pointsCallback, this);
  imu_subscriber_ = nh_.subscribe("imu", 10, &ICPSlam::imuCallback, this);

  // create publisher
  scan_matcher_odometry_publisher_ = nh_.advertise<nav_msgs::Odometry>("scan_matcher_odometry", 1);
  icp_aligned_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("alinged_cloud", 1000);
  icp_map_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("icp_map", 1000);
  icp_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("icp_pose", 1000);
  transform_probability_publisher_ = nh_.advertise<std_msgs::Float32>("transform_probability", 1);

  // create service server
  save_map_service_ = pnh_.advertiseService("save_map", &ICPSlam::saveMapService, this);
}

void ICPSlam::imuCorrect(Eigen::Matrix4f& pose, const ros::Time stamp)
{
  static ros::Time prev_stamp = stamp;
  static sensor_msgs::Imu prev_imu = imu_;
  const double sampling_time = (stamp - prev_stamp).toSec();

  Eigen::Vector3f diff_rot;
  diff_rot.x() = (imu_.angular_velocity.x - prev_imu.angular_velocity.x) * sampling_time;
  diff_rot.y() = (imu_.angular_velocity.y - prev_imu.angular_velocity.y) * sampling_time;
  diff_rot.z() = (imu_.angular_velocity.z - prev_imu.angular_velocity.z) * sampling_time;

  imu_rotate_vec_ += diff_rot;

  Pose pose_vec = icp_slam_utils::convertMatrixToPoseVec(pose);

  pose_vec.roll += imu_rotate_vec_.x();
  pose_vec.pitch += imu_rotate_vec_.y();
  pose_vec.yaw += imu_rotate_vec_.z();

  pose = icp_slam_utils::convertPoseVecToMatrix(pose_vec);

  prev_imu = imu_;
  prev_stamp = stamp;
}

Pose ICPSlam::getCurrentPose()
{
  return icp_slam_utils::convertMatrixToPoseVec(pose_);
}

void ICPSlam::limitCloudScanData(
  const pcl::PointCloud<PointType>::Ptr input_ptr, const pcl::PointCloud<PointType>::Ptr& output_ptr,
  const double min_scan_range, const double max_scan_range)
{
  for (auto point : input_ptr->points) {
    const double range = std::hypot(point.x, point.y);
    if (min_scan_range < range && range < max_scan_range) {
      output_ptr->push_back(point);
    }
  }
}

void ICPSlam::downsample(
  const pcl::PointCloud<PointType>::Ptr input_ptr, const pcl::PointCloud<PointType>::Ptr& output_ptr)
{
  pcl::VoxelGrid<PointType> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  voxel_grid_filter.setInputCloud(input_ptr);
  voxel_grid_filter.filter(*output_ptr);
}

void ICPSlam::pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& input_points_ptr_msg)
{
  pcl::PointCloud<PointType>::Ptr points_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr limit_points_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr filtered_scan_ptr(new pcl::PointCloud<PointType>);

  const ros::Time current_scan_time = input_points_ptr_msg->header.stamp;
  const std::string sensor_frame_id = input_points_ptr_msg->header.frame_id;
  pcl::fromROSMsg(*input_points_ptr_msg, *points_ptr);

  limitCloudScanData(points_ptr, limit_points_ptr, min_scan_range_, max_scan_range_);

  if (!map_) {
    pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);
    transformPointCloud(limit_points_ptr, transform_cloud_ptr, base_frame_id_, sensor_frame_id);
    map_.reset(new pcl::PointCloud<PointType>);
    map_->header.frame_id = "map";
    *map_ += *transform_cloud_ptr;
    registration_->setInputTarget(map_);
  }

  pcl::PointCloud<PointType>::Ptr sensor_transform_cloud(new pcl::PointCloud<PointType>);
  transformPointCloud(limit_points_ptr, sensor_transform_cloud, base_frame_id_, sensor_frame_id);
  downsample(sensor_transform_cloud, filtered_scan_ptr);
  registration_->setInputSource(filtered_scan_ptr);

  if (use_imu_)
    imuCorrect(pose_, current_scan_time);

  pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
  registration_->align(*output_cloud, pose_);

  const bool convergenced = registration_->hasConverged();
  const double fitness_score = registration_->getFitnessScore();

  if (!convergenced)
    ROS_WARN("ICP has not Convergenced!");

  pose_ = registration_->getFinalTransformation();

  // publish tf
  icp_pose_ = getCurrentPose();  // convert matrix to vec
  //icp_slam_utils::publishTF(broadcaster_, icp_pose_, current_scan_time, "map", base_frame_id_);

  pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::transformPointCloud(
    *limit_points_ptr, *transform_cloud_ptr,
    pose_ * icp_slam_utils::convertGeometryTransformToMatrix(getTransform(base_frame_id_, sensor_frame_id)));

  previous_scan_time_ = current_scan_time;

  const double delta = std::hypot(icp_pose_.x - previous_pose_.x, icp_pose_.y - previous_pose_.y);
  if (min_add_scan_shift_ <= delta) {
    previous_pose_ = icp_pose_;

    *map_ += *transform_cloud_ptr;
    registration_->setInputTarget(transform_cloud_ptr);

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*map_, map_msg);
    icp_map_publisher_.publish(map_msg);
  }

  sensor_msgs::PointCloud2 aligned_cloud_msg;
  pcl::toROSMsg(*output_cloud, aligned_cloud_msg);
  aligned_cloud_msg.header.stamp = current_scan_time;
  aligned_cloud_msg.header.frame_id = sensor_frame_id;
  icp_aligned_cloud_publisher_.publish(aligned_cloud_msg);

  geometry_msgs::PoseStamped icp_pose_msg;
  icp_pose_msg.header.frame_id = "map";
  icp_pose_msg.header.stamp = current_scan_time;
  icp_pose_msg.pose = icp_slam_utils::convertToGeometryPose(icp_pose_);
  icp_pose_publisher_.publish(icp_pose_msg);

  publishOdometry(icp_pose_, current_scan_time);

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "ICP has converged: " << convergenced << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "delta: " << delta << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

void ICPSlam::publishOdometry(const Pose pose, const ros::Time stamp)
{
  icp_slam_utils::publishTF(broadcaster_, pose, stamp, "odom", base_frame_id_);

  nav_msgs::Odometry odometry;
  geometry_msgs::Pose pose_msg = icp_slam_utils::convertToGeometryPose(pose);

  odometry.header.frame_id = "odom";
  odometry.child_frame_id = base_frame_id_;
  odometry.header.stamp = stamp;
  odometry.pose.pose = pose_msg;

  scan_matcher_odometry_publisher_.publish(odometry);
}

void ICPSlam::transformPointCloud(
  pcl::PointCloud<PointType>::Ptr input_ptr, pcl::PointCloud<PointType>::Ptr& output_ptr,
  const std::string target_frame, const std::string source_frame)
{
  geometry_msgs::TransformStamped sensor_frame_transform = getTransform(target_frame, source_frame);
  const Eigen::Affine3d base_to_sensor_frame_affine = tf2::transformToEigen(sensor_frame_transform);
  const Eigen::Matrix4f base_to_sensor_frame_matrix = base_to_sensor_frame_affine.matrix().cast<float>();
  pcl::transformPointCloud(*input_ptr, *output_ptr, base_to_sensor_frame_matrix);
}

geometry_msgs::TransformStamped ICPSlam::getTransform(const std::string target_frame, const std::string source_frame)
{
  geometry_msgs::TransformStamped frame_transform;
  try {
    frame_transform = tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    frame_transform.header.stamp = ros::Time::now();
    frame_transform.header.frame_id = target_frame;
    frame_transform.child_frame_id = source_frame;
    frame_transform.transform.translation.x = 0.0;
    frame_transform.transform.translation.y = 0.0;
    frame_transform.transform.translation.z = 0.0;
    frame_transform.transform.rotation.w = 1.0;
    frame_transform.transform.rotation.x = 0.0;
    frame_transform.transform.rotation.y = 0.0;
    frame_transform.transform.rotation.z = 0.0;
  }
  return frame_transform;
}

bool ICPSlam::saveMapService(icp_slam::SaveMapRequest& req, icp_slam::SaveMapResponse& res)
{
  pcl::PointCloud<PointType>::Ptr map_cloud(new pcl::PointCloud<PointType>);

  if (req.resolution <= 0.0) {
    map_cloud = map_;
  } else {
    pcl::VoxelGrid<PointType> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(req.resolution, req.resolution, req.resolution);
    voxel_grid_filter.setInputCloud(map_);
    voxel_grid_filter.filter(*map_cloud);
  }

  map_cloud->header.frame_id = "map";
  int ret = pcl::io::savePCDFile(req.path, *map_cloud);
  res.ret = (ret == 0);

  return true;
}

void ICPSlam::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_ = *msg;
}
