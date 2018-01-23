#include "seline.h"

Seline::Seline(){

  std::string ee_model = "robotiq_85_base_link_fine.pcd";
  seline_pkg_dir_ = ros::package::getPath("seline");
  std::string path_to_model = seline_pkg_dir_ + "/models/" + ee_model;

  original_model_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (path_to_model, *original_model_cloud_) != kModelLoadErrorCode){
    std::cout << "Successfully loaded file " << ee_model << " (" << original_model_cloud_->size () << " points)\n";
  }

  pub_original_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/seline/original_cloud", 1);
  pub_transformed_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/seline/transformed_cloud", 1);
  pub_segmented_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/seline/segmented_cloud", 1);
  pub_icp_out_ = nh_.advertise<sensor_msgs::PointCloud2>("/seline/icp_out", 1);

  sub_point_cloud_ = nh_.subscribe("/camera/depth_registered/points", 1, &Seline::inputCloudCallback, this);

  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

  downsampleInitialModelCloud();

  camera_optical_frame_ = "camera_rgb_optical_frame";
  world_frame_ = "world";
  ee_frame_ = "tool0";
  camera_to_ee_ = Eigen::MatrixXd::Identity(4, 4);
  world_to_camera_ = Eigen::MatrixXd::Identity(4, 4);
  camera_to_icp_ = Eigen::MatrixXd::Identity(4, 4);

  has_seed_ = false;
  has_point_cloud_ = false;

  source_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  target_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  input_cloud_xyz_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  icp_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);


  publishPointCloudXYZ(pub_original_cloud_, *original_model_cloud_, camera_optical_frame_);

  ros::Duration(1.0).sleep(); // Wait for TFs to update a bit

}
Seline::~Seline(){

}


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
  has_point_cloud_ = true;
}

// Downsample the initial model cloud loaded from the pcd;
void Seline::downsampleInitialModelCloud(){
  pcl::PointCloud<pcl::PointNormal>::Ptr pn (new pcl::PointCloud<pcl::PointNormal>);
  copyPointCloud(*original_model_cloud_, *pn);
  pcl::VoxelGrid<pcl::PointNormal> grid;
  grid.setLeafSize (kDownSampleModelLeafSize, kDownSampleModelLeafSize, kDownSampleModelLeafSize);
  grid.setInputCloud (pn);
  grid.filter (*pn);
  copyPointCloud(*pn, *original_model_cloud_);

  // Passthrough filter to crop out just the front surface of the gripper
  splicePointCloudByAxis(original_model_cloud_, "y", 0.0, kEndEffectorCropMaxY);


  // Rotate the cloud to splice along the base_link of the gripper at angle theta
  Eigen::Matrix4d transform = transform_conversions::euler_matrix(0, -kEndEffectorCropTheta*M_PI/180.0, 0) * transform_conversions::translation_matrix(0.0, 0.0, -kGripperLength/2.0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tf_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*original_model_cloud_, *tf_cloud, transform);
  splicePointCloudByAxis(tf_cloud, "y", 0.0, kEndEffectorCropMaxY);
  copyPointCloud(*tf_cloud, *original_model_cloud_);

  transform = transform_conversions::euler_matrix(0, kEndEffectorCropTheta*M_PI/180.0, 0) * transform_conversions::translation_matrix(0.0, 0.0, kGripperLength/2.0);
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
  //std::cout << "Transformed cloud: " << transformed_cloud_->size() << " points\n";

  // Do not try to do segmentation and ICP if we don't have the scene cloud yet
  if (!has_point_cloud_){
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  *segmented_cloud = segmentEndEffectorFromSceneUsingSeed(seed_transform.matrix());
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


// Perform radius search using the seeded gripper location
pcl::PointCloud<pcl::PointXYZ> Seline::segmentEndEffectorFromSceneUsingSeed(Eigen::MatrixXd seed_transform){
  Eigen::MatrixXd gripper_center = seed_transform * transform_conversions::translation_matrix(0, 0, kGripperLength/2.0);
  transform_conversions::publish_matrix_as_tf(br_, gripper_center , camera_optical_frame_, "gripper_center");
  auto translation = gripper_center.col(3); // Pull off the translation component
  pcl::PointXYZ search_point;
  search_point.x = translation[0];
  search_point.y = translation[1];
  search_point.z = translation[2];

  // Perform a nearest neightbor search with the search point
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  float radius = kGripperLength/2.0 + kEpsilonRegionOnGripper;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (input_cloud_xyz_);

  pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud = (new pcl::PointCloud<pcl::PointXYZ>)->makeShared();
  if ( kdtree.radiusSearch (search_point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
    segmented_cloud->width = pointIdxRadiusSearch.size();
    segmented_cloud->height = 1;
    segmented_cloud->is_dense = true;

    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      segmented_cloud->points.push_back(input_cloud_xyz_->points[ pointIdxRadiusSearch[i] ]);
      // std::cout << "    "  <<   input_cloud_xyz_->points[ pointIdxRadiusSearch[i] ].x
      //           << " " << input_cloud_xyz_->points[ pointIdxRadiusSearch[i] ].y
      //           << " " << input_cloud_xyz_->points[ pointIdxRadiusSearch[i] ].z
      //           << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }
  }
  return *segmented_cloud;
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
    // std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    camera_to_icp_ = icp.getFinalTransformation ().cast<double>() * camera_to_icp_;
    transform_conversions::publish_matrix_as_tf(br_, camera_to_icp_ , camera_optical_frame_, "camera_to_icp");

  }
  else{
    PCL_ERROR ("\nICP has not converged.\n");
  }
  publishPointCloudXYZ(pub_icp_out_, *cloud_out, camera_optical_frame_);
  return *cloud_out;
}


// Create the world_to_camera and camera_to_ee transformations
void Seline::lookupTransformations(){
  geometry_msgs::TransformStamped transformStamped;
  transformStamped = tf_buffer_.lookupTransform(world_frame_, camera_optical_frame_, ros::Time(0));
  world_to_camera_ = tf2::transformToEigen(transformStamped).matrix();
  transformStamped = tf_buffer_.lookupTransform(camera_optical_frame_, ee_frame_, ros::Time(0));
  camera_to_ee_ = tf2::transformToEigen(transformStamped).matrix();
}

void Seline::runOnce(){

  lookupTransformations();
  processSeed(camera_to_ee_);

  // Debugging push out the original point cloud and the transformed point cloud
  // publishPointCloudXYZ(pub_original_cloud_, *original_model_cloud_, camera_optical_frame_);
  // publishPointCloudXYZ(pub_transformed_cloud_, *transformed_cloud_, camera_optical_frame_);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "seline");
  Seline *node = new Seline();
  ros::Rate *loop_rate = new ros::Rate(kDefaultLoopRate);
  while (ros::ok()){
    node->runOnce();
    loop_rate->sleep();
    ros::spinOnce();
  }
  return 0;
}
