#include "seline.h"

Seline::Seline(std::string seline_mode){
  // Obtain the frame names from the parameter server
  nh_.getParam("/seline/ee_model_file", ee_model_file_);
  nh_.getParam("/seline/ee_frame", ee_frame_);
  nh_.getParam("/seline/world_frame", world_frame_);
  nh_.getParam("/seline/manipulator_frame", manipulator_frame_);
  nh_.getParam("/seline/camera_optical_frame", camera_optical_frame_);
  nh_.getParam("/seline/point_cloud_topic", point_cloud_topic_);
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

  // Using the loaded point cloud, compute its properties (min/max values in each dimension)
  original_model_cloud_properties_ = athena::pointcloud::computePointCloudMinMax(original_model_cloud_);
  gripper_length_ = original_model_cloud_properties_.max_point.z - original_model_cloud_properties_.min_point.z;
  gripper_to_surface_ = (original_model_cloud_properties_.max_point.y - original_model_cloud_properties_.min_point.y)/2.0;

  // Create debug publishers and the subscriber to the raw point cloud
  pub_original_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/seline/original_cloud", 1);
  pub_transformed_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/seline/transformed_cloud", 1);
  pub_seg_tracking_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/seline/segmented_tracking_cloud", 1);
  pub_segmented_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/seline/segmented_cloud", 1);
  pub_icp_out_ = nh_.advertise<sensor_msgs::PointCloud2>("/seline/icp_out", 1);
  pub_ee_track_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/seline/grasp_cloud", 1);
  pub_est_world_frame_ = nh_.advertise<athena_transform::HomogeneousTransform>("/seline/est_world_frame", 1);
  pub_grasp_obj_marker_ = nh_.advertise<visualization_msgs::Marker>("/seline/grasped_obj_geometry", 1);

  sub_point_cloud_ = nh_.subscribe(point_cloud_topic_, 1, &Seline::inputCloudCallback, this);
  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
  srv_trigger_new_seed_ = nh_.advertiseService("/seline/trigger_seed", &Seline::triggerSeedCallback, this);
  srv_get_ee_offset_ = nh_.advertiseService("/seline/get_ee_offset", &Seline::getEndEffectorOffsetCallback, this);

  camera_to_ee_.matrix() = Eigen::MatrixXd::Identity(4, 4);
  camera_to_icp_.matrix() = Eigen::MatrixXd::Identity(4, 4);

  // Create the cropped model cloud as loaded in from above path_to_model
  downsampleInitialModelCloud();
  athena::pointcloud::publishPointCloudXYZ(pub_original_cloud_, *original_model_cloud_, camera_optical_frame_);

  has_seed_ = false;
  has_point_cloud_ = false;
  has_grasped_obj_ = false;

  // Create the initial grasped object visuzliation marker
  grasped_obj_marker_.header.frame_id = world_frame_;
  grasped_obj_marker_.header.stamp = ros::Time();
  grasped_obj_marker_.ns = "grasped_object";
  grasped_obj_marker_.id = 128;
  grasped_obj_marker_.type = visualization_msgs::Marker::CUBE;

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

  // Save resources when in tracking mode by not computing anything until the
  // end effector is within the view of the camera
  pcl::PointCloud<pcl::PointXYZ>::Ptr tf_world_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  if (!lookupKnownTransformations()) { std::cout << "No transforms. Exiting.\n"; return; }
  pcl::transformPointCloud(*input_cloud_xyz_, *tf_world_cloud, world_to_camera_.matrix());

  if (seline_mode_ == "track"){
    PointCloudProperties result = athena::pointcloud::computePointCloudMinMax(tf_world_cloud);

    Eigen::Affine3d ee_world;
    ee_world.matrix() = manipulator_to_world_.matrix().inverse();
    double end_effector_z = ee_world.translation().z();

    // std::cout << result.min_point.z << " " << result.max_point.z << "\n";
    // std::cout << "End effector z: " << end_effector_z << "\n";
    if (end_effector_z < result.min_point.z || end_effector_z > result.max_point.z){
      ROS_INFO("End effector out of view. Exiting.");
      has_point_cloud_ = false;
      return;
    }
  }

  if (crop_world_plane_ && lookupKnownTransformations()){
    // First transform the input cloud into the world frame, perform a crop over the Z-axis
    // and lastly transform the cropped cloud back into the camera_optical_frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Remove all point under the plane
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(tf_world_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(crop_world_plane_height_, kMaxCropDistance);
    pass.filter(*result_cloud);
    copyPointCloud(*result_cloud, *tf_world_cloud);
    pcl::transformPointCloud(*tf_world_cloud, *input_cloud_xyz_, camera_to_world_.matrix());
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
  Eigen::Matrix4d transform = athena::transform::euler_matrix(0, -ee_crop_theta_*M_PI/180.0, 0) * athena::transform::translation_matrix(0.0, 0.0, -gripper_length_/2.0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tf_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*original_model_cloud_, *tf_cloud, transform);
  splicePointCloudByAxis(tf_cloud, "y", 0.0, ee_crop_max_y_);
  copyPointCloud(*tf_cloud, *original_model_cloud_);

  transform = athena::transform::translation_matrix(0.0, 0.0, gripper_length_/2.0) * athena::transform::euler_matrix(0, ee_crop_theta_*M_PI/180.0, 0);
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

// @param: seed_transform - This transform is from the camera to the base of the gripper
bool Seline::processSeed(Eigen::Affine3d seed_transform){
  // Transform the original cloud using the seed transform
  transformed_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*original_model_cloud_, *transformed_cloud_, seed_transform);
  athena::pointcloud::publishPointCloudXYZ(pub_transformed_cloud_, *transformed_cloud_, camera_optical_frame_);

  // Do not try to do segmentation and ICP if we don't have the scene cloud yet
  if (!has_point_cloud_){
    std::cout << "No point cloud received, skipping processing seed!\n";
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  // Perform radius search using the seeded gripper location
  if (seline_mode_ == "calibrate"){
    Eigen::MatrixXd gripper_center = seed_transform.matrix() * athena::transform::translation_matrix(0, 0, gripper_length_/2.0);
    athena::transform::publish_matrix_as_tf(br_, gripper_center , camera_optical_frame_, "gripper_center");
    *segmented_cloud = segmentEndEffectorFromSceneUsingSeed(gripper_center, gripper_length_/2.0 + epsilon_region_on_gripper_);
    athena::pointcloud::publishPointCloudXYZ(pub_segmented_cloud_, *segmented_cloud, camera_optical_frame_);

    // Seed the ICP solution using the transformed cloud and its transformation
    if (!has_seed_){
      copyPointCloud(*transformed_cloud_, *icp_cloud_);
      camera_to_icp_.matrix() = seed_transform.matrix();
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

    if (segmented_cloud->points.size() < kMinEndEffectorTrackingPoints){
      seed_transform.matrix() = seed_transform.matrix() * athena::transform::translation_matrix(kCameraFrameSearchDistance, 0, 0);
      *segmented_cloud = segmentEndEffectorFromSceneUsingSeed(seed_transform.matrix(), kSearchEpsilonOnTracking);
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr tf_world_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*segmented_cloud, *tf_world_cloud, world_to_camera_.matrix());

    if (tf_world_cloud->points.size() == 0) { return false; }

    // Determine the offset between the desired end effector position and the observed
    // end effector position given by the median of the extracted point cloud
    Eigen::Affine3d desired_ee_pose;
    desired_ee_pose.matrix() = ee_to_world_.matrix().inverse();
    Eigen::Affine3d observed_ee_pose = desired_ee_pose;

    PointCloudProperties cloud_props = athena::pointcloud::computePointCloudMinMax(tf_world_cloud);
    Eigen::Vector3d median = athena::pointcloud::computePointCloudMedian(tf_world_cloud);

    // The offset in the x direction is directly the difference detween the cloud's median and the desired ee translation
    observed_offset_ = median - desired_ee_pose.translation();

    // The offset in the y direction is the surface of the point cloud offset by the width of the gripper
    observed_offset_.y() = (cloud_props.max_point.y - gripper_to_surface_) - desired_ee_pose.translation().y();
    observed_offset_.z() = 0.0; // Do not trust the median point in the z-direction

    observed_ee_pose.translation() = desired_ee_pose.translation() + observed_offset_;
    // Eigen::Matrix4d adjusted_camera_seed = world_to_camera_.inverse() * observed_ee_pose.matrix();
    // *segmented_cloud = segmentEndEffectorFromSceneUsingSeed(adjusted_camera_seed, kSearchEpsilonOnTracking);
    // pcl::transformPointCloud(*segmented_cloud, *tf_world_cloud, world_to_camera_);

    athena::transform::publish_matrix_as_tf(br_, observed_ee_pose.matrix() , world_frame_, "observed_ee_pose");
    athena::pointcloud::publishPointCloudXYZ(pub_seg_tracking_cloud_, *tf_world_cloud, world_frame_);

    has_grasped_obj_ = processGraspedObject(observed_ee_pose.matrix());

    if (!has_grasped_obj_){ // If there's no object clear the marker
      grasped_obj_marker_.action = visualization_msgs::Marker::DELETEALL;
      pub_grasp_obj_marker_.publish(grasped_obj_marker_);
    }

  }
  return true;
}

bool Seline::processGraspedObject(Eigen::Matrix4d observed_ee_frame){
  // Obtain the transformation to the tip of the gripper as well as the observed manipulation frame
  Eigen::Matrix4d observed_manipulator = observed_ee_frame * ee_to_manipulator_.matrix();
  Eigen::Matrix4d ee_grasp_reference = observed_ee_frame * athena::transform::translation_matrix(0, 0, gripper_length_);
  Eigen::Matrix4d cam_ee_reference = world_to_camera_.inverse() * ee_grasp_reference;
  athena::transform::publish_matrix_as_tf(br_, observed_manipulator, world_frame_, "observed_manipulator_link");

  pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  *segmented_cloud = segmentEndEffectorFromSceneUsingSeed(cam_ee_reference, kSearchEpsilonOnGraspedObject);
  pcl::transformPointCloud(*segmented_cloud, *segmented_cloud, world_to_camera_.matrix());
  if (segmented_cloud->points.size() == 0){ return false; }

  // Spice out the cloud based on the gripper geometry and grasp frames
  double max_gripper_offset = 1.75*gripper_to_surface_;
  manipulator_translation_ = athena::transform::translation_from_matrix(observed_manipulator);
  splicePointCloudByAxis(segmented_cloud, "z", 0, manipulator_translation_.z());
  splicePointCloudByAxis(segmented_cloud, "y", manipulator_translation_.y(), manipulator_translation_.y()+max_gripper_offset);
  splicePointCloudByAxis(segmented_cloud, "x", manipulator_translation_.x()-max_gripper_offset, manipulator_translation_.x()+max_gripper_offset);

  if (segmented_cloud->points.size() == 0){ return false; }
  segmented_cloud = athena::pointcloud::getMaxEuclideanClusterFromPointCloud(segmented_cloud, 0.01);

  if (segmented_cloud->points.size() == 0){ return false; }
  athena::pointcloud::publishPointCloudXYZ(pub_ee_track_cloud_, *segmented_cloud, world_frame_);

  // Obtain the point cloud geometries from the object in the hand
  PointCloudProperties result = athena::pointcloud::computePointCloudMinMax(segmented_cloud);
  // std::cout << "Min point: " << result.min_point.x << " " << result.min_point.y << " " << result.min_point.z << "\n";
  // std::cout << "Max point: " << result.max_point.x << " " << result.max_point.y << " " << result.max_point.z << "\n";

  grasped_obj_geometry_.width = result.max_point.x - result.min_point.x;
  grasped_obj_geometry_.depth = result.max_point.y - result.min_point.y;
  grasped_obj_geometry_.length = result.max_point.z - result.min_point.z;

  // std::cout << "Object width: " << width  << "\n";
  // std::cout << "Object depth: " << depth  << "\n";
  // std::cout << "Object length: " << length << "\n";

  // Create a visualization marker for the grasped object geometry
  grasped_obj_marker_.action = visualization_msgs::Marker::MODIFY;
  grasped_obj_marker_.pose.position.x = result.min_point.x + grasped_obj_geometry_.width/2.0;
  grasped_obj_marker_.pose.position.y = result.min_point.y + grasped_obj_geometry_.depth/2.0 - observed_offset_.x();
  grasped_obj_marker_.pose.position.z = result.min_point.z + grasped_obj_geometry_.length/2.0;
  grasped_obj_marker_.pose.orientation.w = 1.0;
  grasped_obj_marker_.scale.x = grasped_obj_geometry_.width;
  grasped_obj_marker_.scale.y = grasped_obj_geometry_.depth;
  grasped_obj_marker_.scale.z = grasped_obj_geometry_.length;
  grasped_obj_marker_.color.r = 1.0;
  grasped_obj_marker_.color.g = 0.0;
  grasped_obj_marker_.color.b = 0.0;
  grasped_obj_marker_.color.a = 0.5;
  pub_grasp_obj_marker_.publish(grasped_obj_marker_);

  return true;
}

// Segments the end effector point cloud by using the translation component of the transformation
// provided and with nearest neighbor radius search
pcl::PointCloud<pcl::PointXYZ> Seline::segmentEndEffectorFromSceneUsingSeed(Eigen::MatrixXd seed_transform, float radius){
  Eigen::Vector4d translation = seed_transform.col(3); // Pull off the translation component
  pcl::PointXYZ search_point;
  search_point.x = translation.x();
  search_point.y = translation.y();
  search_point.z = translation.z();

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

  resp.has_grasped_obj = has_grasped_obj_;
  if (has_grasped_obj_){
    resp.grasped_obj_geometry[0] = grasped_obj_geometry_.width;
    resp.grasped_obj_geometry[1] = grasped_obj_geometry_.depth;
    resp.grasped_obj_geometry[2] = grasped_obj_geometry_.length;

    resp.observed_manipulator_origin[0] = manipulator_translation_.x();
    resp.observed_manipulator_origin[1] = manipulator_translation_.y();
    resp.observed_manipulator_origin[2] = manipulator_translation_.z();

    resp.grasped_obj_origin[0] = grasped_obj_marker_.pose.position.x;
    resp.grasped_obj_origin[1] = grasped_obj_marker_.pose.position.y;
    resp.grasped_obj_origin[2] = grasped_obj_marker_.pose.position.z;
    resp.grasped_obj_marker = grasped_obj_marker_;
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
    camera_to_icp_.matrix() = icp.getFinalTransformation ().cast<double>() * camera_to_icp_.matrix();
    athena::transform::publish_matrix_as_tf(br_, camera_to_icp_ , camera_optical_frame_, "est_camera_to_tool");
  }
  else{
    PCL_ERROR ("\nICP has not converged.\n");
  }
  athena::pointcloud::publishPointCloudXYZ(pub_icp_out_, *cloud_out, camera_optical_frame_);
  return *cloud_out;
}


bool Seline::lookupKnownTransformations(){
  try{
    camera_to_ee_ = tf2::transformToEigen(tf_buffer_.lookupTransform(camera_optical_frame_, ee_frame_, ros::Time(0)));
    ee_to_world_ = tf2::transformToEigen(tf_buffer_.lookupTransform(ee_frame_, world_frame_, ros::Time(0)));
    ee_to_manipulator_ = tf2::transformToEigen(tf_buffer_.lookupTransform(ee_frame_, manipulator_frame_, ros::Time(0)));
    manipulator_to_world_ = tf2::transformToEigen(tf_buffer_.lookupTransform(manipulator_frame_, world_frame_, ros::Time(0)));
    world_to_camera_ = tf2::transformToEigen(tf_buffer_.lookupTransform(world_frame_, camera_optical_frame_, ros::Time(0)));
    camera_to_world_ = tf2::transformToEigen(tf_buffer_.lookupTransform(camera_optical_frame_, world_frame_, ros::Time(0)));
    return true;
  }
  catch(...){
    std::cout << "Unable to look up transformations!\n";
    return false;
  }
}


void Seline::processEstimatedTransformations(){
  // For debugging, ensure that the ee_frame -> world frame looks correct
  athena::transform::publish_matrix_as_tf(br_, ee_to_world_ , ee_frame_, "initial_world_frame");

  // Back out the transformations to estimate the camera_to_world
  Eigen::MatrixXd est_camera_to_world = camera_to_icp_.matrix() * ee_to_world_.matrix();
  athena::transform::publish_matrix_as_tf(br_, est_camera_to_world , camera_optical_frame_, "est_world_frame");
  athena_transform::HomogeneousTransform est_world_frame;
  est_world_frame.source_frame = camera_optical_frame_;
  est_world_frame.frame_id = world_frame_;
  est_world_frame.transform = athena::transform::eigen4d_matrix_to_array(est_camera_to_world);
  pub_est_world_frame_.publish(est_world_frame);
}

void Seline::runOnce(){
  if (lookupKnownTransformations() && processSeed(camera_to_ee_) ){
    processEstimatedTransformations();
  }
  else {
    ros::Duration(0.25).sleep(); // Downgrade the node to 4 Hz on failures
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
