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
  sub_point_cloud_ = nh_.subscribe("/camera/depth_registered/points", 1, &Seline::inputCloudCallback, this);

  downsampleInitialModelCloud();

  camera_optical_frame_ = "camera_rgb_optical_frame";
  transformation_matrix_ = Eigen::MatrixXf::Identity(4, 4);
  has_seed_ = false;

  source_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  target_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  publishPointCloudXYZ(pub_original_cloud_, *original_model_cloud_, camera_optical_frame_);


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
}

void Seline::downsampleInitialModelCloud(){
  pcl::PointCloud<pcl::PointNormal>::Ptr pn (new pcl::PointCloud<pcl::PointNormal>);
  copyPointCloud(*original_model_cloud_, *pn);
  pcl::VoxelGrid<pcl::PointNormal> grid;
  grid.setLeafSize (kDownSampleModelLeafSize, kDownSampleModelLeafSize, kDownSampleModelLeafSize);
  grid.setInputCloud (pn);
  grid.filter (*pn);
  copyPointCloud(*pn, *original_model_cloud_);
}

void Seline::publishPointCloudXYZ(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, std::string frame_id){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);

  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(pcl_pc2, cloud_msg);
  cloud_msg.header.frame_id = frame_id;
  pub.publish(cloud_msg);
}

void Seline::processSeed(Eigen::Matrix4f matrix){
    Eigen::Affine3f seed_transform;
    seed_transform.matrix() = matrix;

    auto translation = seed_transform.translation();
    pcl::PointXYZ search_point;
    search_point.x = translation[0];
    search_point.y = translation[1];
    search_point.z = translation[2];

    // std::cout << seed_transform.matrix() << "\n";

    // Transform the original cloud using the seed transform
    transformed_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*original_model_cloud_, *transformed_cloud_, seed_transform);
    std::cout << "Transformed cloud: " << transformed_cloud_->size() << " points\n";
    publishPointCloudXYZ(pub_transformed_cloud_, *transformed_cloud_, camera_optical_frame_);
}

// Continuously calls ICP and uses the solution as a seed for the next
void Seline::doIterativeRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  double kIterations = 1;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations (kIterations);
  icp.setInputSource (source_cloud);
  icp.setInputTarget (target_cloud);
  icp.align (*cloud_out);
  icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function

  if (icp.hasConverged ())
    {
      std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
      //auto transformation_matrix = icp.getFinalTransformation ().cast<double>() * transformation_matrix;
      //std::cout << transformation_matrix << "\n";
      //*cloud_model = *cloud_out;
    }
    else
    {
      PCL_ERROR ("\nICP has not converged.\n");
    }

}

void Seline::runOnce(){

  // Debugging push out the original point cloud and the transformed point cloud
  publishPointCloudXYZ(pub_original_cloud_, *original_model_cloud_, camera_optical_frame_);
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
