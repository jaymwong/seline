#include "seline.h"

Seline::Seline(std::string seline_mode){
  // Obtain the frame names from the parameter server
  nh_.getParam("/seline/ee_model_file", ee_model_file_);
  nh_.getParam("/seline/ee_frame", ee_frame_);
  nh_.getParam("/seline/world_frame", world_frame_);
  nh_.getParam("/seline/camera_optical_frame", camera_optical_frame_);
  nh_.getParam("/seline/point_cloud_topic", point_cloud_topic_);
  nh_.getParam("/seline/gripper_length", gripper_length_);
  nh_.getParam("/seline/ee_crop_theta", ee_crop_theta_);
  nh_.getParam("/seline/ee_crop_max_y", ee_crop_max_y_);
  nh_.getParam("/seline/epsilon_region_on_gripper", epsilon_region_on_gripper_);
  nh_.getParam("/seline/crop_world_plane", crop_world_plane_);
  nh_.getParam("/seline/crop_world_plane_height", crop_world_plane_height_);

  seline_mode_ = seline_mode;

  // Load the model from the models directory
  seline_pkg_dir_ = ros::package::getPath("seline");
  std::string path_to_model = seline_pkg_dir_ + "/models/" + ee_model_file_;
  original_model_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (path_to_model, *original_model_cloud_) != kModelLoadErrorCode){
    std::cout << "Successfully loaded file " << ee_model_file_ << " (" << original_model_cloud_->size () << " points)\n";
  }

  // Create debug publishers and the subscriber to the raw point cloud
  pub_original_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/seline/original_cloud", 1);
  pub_transformed_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/seline/transformed_cloud", 1);
  pub_seg_tracking_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/seline/segmented_tracking_cloud", 1);
  pub_segmented_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/seline/segmented_cloud", 1);
  pub_icp_out_ = nh_.advertise<sensor_msgs::PointCloud2>("/seline/icp_out", 1);
  pub_est_world_frame_ = nh_.advertise<transform_conversions::HomogeneousTransform>("/seline/est_world_frame", 1);

  sub_point_cloud_ = nh_.subscribe(point_cloud_topic_, 1, &Seline::inputCloudCallback, this);
  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
  srv_trigger_new_seed_ = nh_.advertiseService("/seline/trigger_seed", &Seline::triggerSeedCallback, this);
  srv_get_ee_offset_ = nh_.advertiseService("/seline/get_ee_offset", &Seline::getEndEffectorOffsetCallback, this);

  camera_to_ee_ = Eigen::MatrixXd::Identity(4, 4);
  camera_to_icp_ = Eigen::MatrixXd::Identity(4, 4);

  // Create the cropped model cloud as loaded in from above path_to_model
  downsampleInitialModelCloud();
  publishPointCloudXYZ(pub_original_cloud_, *original_model_cloud_, camera_optical_frame_);

  has_seed_ = false;
  has_point_cloud_ = false;

  // Initialize the point cloud pointers
  source_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  target_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  input_cloud_xyz_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  icp_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  ros::Duration(1.0).sleep(); // Wait for TFs to update a bit
}
Seline::~Seline(){}


void Seline::inputCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input){
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  pcl::PCLPointCloud2* input_cloud_pcl = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(input_cloud_pcl);
  pcl::PCLPointCloud2 cloud_filtered;

  pcl_conversions::toPCL(*input, *input_cloud_pcl);
  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(kDefaultPointCloudLeafSize, kDefaultPointCloudLeafSize, kDefaultPointCloudLeafSize);
  sor.filter(cloud_filtered);
  pcl::fromPCLPointCloud2(cloud_filtered, *input_cloud_xyz_);

  if (crop_world_plane_ && lookupKnownTransformations()){
    // First transform the input cloud into the world frame, perform a crop over the Z-axis
    // and lastly transform the cropped cloud back into the camera_optical_frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr tf_world_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*input_cloud_xyz_, *tf_world_cloud, world_to_camera_);

    // Remove all point under the plane
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(tf_world_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(crop_world_plane_height_, 5.0);
    pass.filter(*result_cloud);
    copyPointCloud(*result_cloud, *tf_world_cloud);
    pcl::transformPointCloud(*tf_world_cloud, *input_cloud_xyz_, camera_to_world_);
  }

  has_point_cloud_ = true;
}

// Downsample the initial model cloud loaded from the pcd; The reason we crop the model cloud
// is that ICP will fall into local minima if the points on the model is greather than the
// points of the scene cloud. For this reason, we splice the model cloud along the Z-axis,
// (e.g. taking only points along y from y_min = 0.0, to y_max = ee_crop_max_y_). Next,
// we rotate the model along that z axis again, pitching it backwards by ee_crop_theta_
// and again splice the model along y. We the rotate the model back. This produces a model crop
// that is reasonable when the gripper is near perpendicular to the camera.
void Seline::downsampleInitialModelCloud(){
  pcl::PointCloud<pcl::PointNormal>::Ptr pn (new pcl::PointCloud<pcl::PointNormal>);
  copyPointCloud(*original_model_cloud_, *pn);
  pcl::VoxelGrid<pcl::PointNormal> grid;
  grid.setLeafSize (kDownSampleModelLeafSize, kDownSampleModelLeafSize, kDownSampleModelLeafSize);
  grid.setInputCloud (pn);
  grid.filter (*pn);
  copyPointCloud(*pn, *original_model_cloud_);

  // Passthrough filter to crop out just the front surface of the gripper
  splicePointCloudByAxis(original_model_cloud_, "y", 0.0, ee_crop_max_y_);

  // Rotate the cloud to splice along the base_link of the gripper at angle theta
  Eigen::Matrix4d transform = transform_conversions::euler_matrix(0, -ee_crop_theta_*M_PI/180.0, 0) * transform_conversions::translation_matrix(0.0, 0.0, -gripper_length_/2.0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tf_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*original_model_cloud_, *tf_cloud, transform);
  splicePointCloudByAxis(tf_cloud, "y", 0.0, ee_crop_max_y_);
  copyPointCloud(*tf_cloud, *original_model_cloud_);

  transform = transform_conversions::translation_matrix(0.0, 0.0, gripper_length_/2.0) * transform_conversions::euler_matrix(0, ee_crop_theta_*M_PI/180.0, 0);
  tf_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*original_model_cloud_, *tf_cloud, transform);
  copyPointCloud(*tf_cloud, *original_model_cloud_);
}

// Passthrough filter to crop out just the front surface of the gripper
void Seline::splicePointCloudByAxis(pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud, std::string axis, double min, double max){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (pt_cloud);
  pass.setFilterFieldName (axis);
  pass.setFilterLimits (min, max);
  pass.filter (*cloud_filtered);
  copyPointCloud(*cloud_filtered, *pt_cloud);
}

// Helper function to directly publish a point cloud under the publisher with desired frame_id
void Seline::publishPointCloudXYZ(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, std::string frame_id){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(pcl_pc2, cloud_msg);
  cloud_msg.header.frame_id = frame_id;
  pub.publish(cloud_msg);
}

void Seline::processSeed(Eigen::Matrix4d matrix){
  Eigen::Affine3d seed_transform; // This transform is from the camera to the base of the gripper
  seed_transform.matrix() = matrix;

  // Transform the original cloud using the seed transform
  transformed_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*original_model_cloud_, *transformed_cloud_, seed_transform);
  publishPointCloudXYZ(pub_transformed_cloud_, *transformed_cloud_, camera_optical_frame_);

  // Do not try to do segmentation and ICP if we don't have the scene cloud yet
  if (!has_point_cloud_){
    std::cout << "No point cloud received, skipping processing seed!\n";
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  // Perform radius search using the seeded gripper location
  if (seline_mode_ == "calibrate"){
    Eigen::MatrixXd gripper_center = seed_transform.matrix() * transform_conversions::translation_matrix(0, 0, gripper_length_/2.0);
    transform_conversions::publish_matrix_as_tf(br_, gripper_center , camera_optical_frame_, "gripper_center");
    *segmented_cloud = segmentEndEffectorFromSceneUsingSeed(gripper_center, gripper_length_/2.0 + epsilon_region_on_gripper_);
    publishPointCloudXYZ(pub_segmented_cloud_, *segmented_cloud, camera_optical_frame_);

    // Seed the ICP solution using the transformed cloud and its transformation
    if (!has_seed_){
      copyPointCloud(*transformed_cloud_, *icp_cloud_);
      camera_to_icp_ = seed_transform.matrix();
      has_seed_ = true;
    }

    // Actually do the iterative registration step
    *icp_cloud_ = doIterativeRegistration(icp_cloud_, segmented_cloud);
  }

  if (seline_mode_ == "track"){
    // Now compute the estimated error in the registration to the end effector
    // Note, this is meant to be used after the calibration is registered back into
    // the TF tree. Notably, we use this functionality to do, mid-grasp corrections
    // (e.g. scenarios where the desired end effector position is off, yet we can observe
    // and estimate the error and accommodate the offset).
    *segmented_cloud = segmentEndEffectorFromSceneUsingSeed(seed_transform.matrix(), kSearchEpsilonOnTracking);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tf_world_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*segmented_cloud, *tf_world_cloud, world_to_camera_);

    if (tf_world_cloud->points.size() == 0) { return; }

    // Determine the offset between the desired end effector position and the observed
    // end effector position given by the median of the extracted point cloud
    Eigen::Affine3d desired_ee_pose;
    desired_ee_pose.matrix() = ee_to_world_.inverse();
    Eigen::Affine3d observed_ee_pose = desired_ee_pose;

    for (int i = 0; i < kSearchTrackingIterations; i++){
      observed_offset_ = computePointCloudMedian(tf_world_cloud) - desired_ee_pose.translation();
      observed_ee_pose.translation() = desired_ee_pose.translation() + observed_offset_;

      Eigen::Matrix4d adjusted_camera_seed = world_to_camera_.inverse() * observed_ee_pose.matrix();
      *segmented_cloud = segmentEndEffectorFromSceneUsingSeed(adjusted_camera_seed, kSearchEpsilonOnTracking);
      pcl::transformPointCloud(*segmented_cloud, *tf_world_cloud, world_to_camera_);
    }

    // Be wary of the full position: the position facing the camera (in our case y), may be
    // nonsense since the median of the point cloud refers to the surface of the observed points,
    // therefore seline is not aware of the depth component.
    transform_conversions::publish_matrix_as_tf(br_, observed_ee_pose.matrix() , world_frame_, "observed_ee_pose");
    publishPointCloudXYZ(pub_seg_tracking_cloud_, *tf_world_cloud, world_frame_);
  }
}

// Segments the end effector point cloud by using the translation component of the transformation
// provided and with nearest neighbor radius search
pcl::PointCloud<pcl::PointXYZ> Seline::segmentEndEffectorFromSceneUsingSeed(Eigen::MatrixXd seed_transform, float radius){
  auto translation = seed_transform.col(3); // Pull off the translation component
  pcl::PointXYZ search_point;
  search_point.x = translation[0];
  search_point.y = translation[1];
  search_point.z = translation[2];

  // Perform a nearest neightbor search with the search point
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (input_cloud_xyz_);

  pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud = (new pcl::PointCloud<pcl::PointXYZ>)->makeShared();
  if ( kdtree.radiusSearch (search_point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
    segmented_cloud->width = pointIdxRadiusSearch.size();
    segmented_cloud->height = 1;
    segmented_cloud->is_dense = true;

    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      segmented_cloud->points.push_back(input_cloud_xyz_->points[ pointIdxRadiusSearch[i] ]);
    }
  }
  return *segmented_cloud;
}

// Computes the median of a point cloud
Eigen::Vector3d Seline::computePointCloudMedian(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  std::vector<double> cloud_x, cloud_y, cloud_z;
  for (int i = 0; i < cloud->points.size(); i++){
    pcl::PointXYZ pt = cloud->points[i];
    cloud_x.push_back(pt.x);
    cloud_y.push_back(pt.y);
    cloud_z.push_back(pt.z);
  }
  return computePointCloudMedian(cloud_x, cloud_y, cloud_z);
}

Eigen::Vector3d Seline::computePointCloudMedian(std::vector<double> cluster_pt_x, std::vector<double> cluster_pt_y, std::vector<double> cluster_pt_z){
  Eigen::Vector3d median;
  sort(cluster_pt_x.begin(), cluster_pt_x.end());
  sort(cluster_pt_y.begin(), cluster_pt_y.end());
  sort(cluster_pt_z.begin(), cluster_pt_z.end());

  median(0) = cluster_pt_x.at(cluster_pt_x.size()/2);
  median(1) = cluster_pt_y.at(cluster_pt_y.size()/2);
  median(2) = cluster_pt_z.at(cluster_pt_z.size()/2);
  return median;
}


bool Seline::triggerSeedCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp){
  has_seed_ = false;
  std::cout << "Successfully reset seed!\n";
  return true;
}

bool Seline::getEndEffectorOffsetCallback(seline::GetEndEffectorOffset::Request &req, seline::GetEndEffectorOffset::Response &resp){
  resp.frame_id = world_frame_;

  for (int i = 0; i < kOffsetVectorSize; i++){
    resp.offset[i] = observed_offset_(i);
  }
  return true;
}


// Continuously calls ICP and uses the solution as a seed for the next
pcl::PointCloud<pcl::PointXYZ> Seline::doIterativeRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations (kICPIterations);
  icp.setInputSource (source_cloud);
  icp.setInputTarget (target_cloud);
  icp.align (*cloud_out);

  if (icp.hasConverged ()) {
    camera_to_icp_ = icp.getFinalTransformation ().cast<double>() * camera_to_icp_;
    transform_conversions::publish_matrix_as_tf(br_, camera_to_icp_ , camera_optical_frame_, "est_camera_to_tool");
  }
  else{
    PCL_ERROR ("\nICP has not converged.\n");
  }
  publishPointCloudXYZ(pub_icp_out_, *cloud_out, camera_optical_frame_);
  return *cloud_out;
}


bool Seline::lookupKnownTransformations(){
  try{
    camera_to_ee_ = tf2::transformToEigen(tf_buffer_.lookupTransform(camera_optical_frame_, ee_frame_, ros::Time(0))).matrix();
    ee_to_world_ = tf2::transformToEigen(tf_buffer_.lookupTransform(ee_frame_, world_frame_, ros::Time(0))).matrix();
    world_to_camera_ = tf2::transformToEigen(tf_buffer_.lookupTransform(world_frame_, camera_optical_frame_, ros::Time(0))).matrix();
    camera_to_world_ = tf2::transformToEigen(tf_buffer_.lookupTransform(camera_optical_frame_, world_frame_, ros::Time(0))).matrix();
    return true;
  }
  catch(...){
    std::cout << "Unable to look up transformations!\n";
    return false;
  }
}


void Seline::processEstimatedTransformations(){
  // For debugging, ensure that the ee_frame -> world frame looks correct
  transform_conversions::publish_matrix_as_tf(br_, ee_to_world_ , ee_frame_, "initial_world_frame");

  // Back out the transformations to estimate the camera_to_world
  Eigen::MatrixXd est_camera_to_world = camera_to_icp_ * ee_to_world_;
  transform_conversions::publish_matrix_as_tf(br_, est_camera_to_world , camera_optical_frame_, "est_world_frame");
  transform_conversions::HomogeneousTransform est_world_frame;
  est_world_frame.source_frame = camera_optical_frame_;
  est_world_frame.frame_id = world_frame_;
  est_world_frame.transform = transform_conversions::eigen4d_matrix_to_array(est_camera_to_world);
  pub_est_world_frame_.publish(est_world_frame);
}

void Seline::runOnce(){
  if (lookupKnownTransformations()){
    processSeed(camera_to_ee_);
    processEstimatedTransformations();
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "seline");
  Seline *node = new Seline(argv[1]);
  ros::Rate *loop_rate = new ros::Rate(kDefaultLoopRate);
  while (ros::ok()){
    node->runOnce();
    loop_rate->sleep();
    ros::spinOnce();
  }
  return 0;
}
