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

  camera_optical_frame_ = "camera_rgb_optical_frame";

  Eigen::Affine3f seed_transform;
  seed_transform.matrix() = Eigen::MatrixXf::Identity(4, 4);


  pcl::PointCloud<pcl::PointNormal>::Ptr pn (new pcl::PointCloud<pcl::PointNormal>);
  copyPointCloud(*original_model_cloud_, *pn);

  pcl::VoxelGrid<pcl::PointNormal> grid;
  grid.setLeafSize (0.0075, 0.0075, 0.0075);
  grid.setInputCloud (pn);
  grid.filter (*pn);
  copyPointCloud(*pn, *original_model_cloud_);




  // TODO: A test... display the original pointcloud a little bit...
  seed_transform.translation() = Eigen::Vector3f(1.0, 0.2, 1.0);

  std::cout << seed_transform.matrix() << "\n";

  transformed_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*original_model_cloud_, *transformed_cloud_, seed_transform);

  std::cout << "Transformed cloud: " << transformed_cloud_->size() << " points\n";


  publishPointCloudXYZ(pub_original_cloud_, *original_model_cloud_, camera_optical_frame_);
  publishPointCloudXYZ(pub_transformed_cloud_, *transformed_cloud_, camera_optical_frame_);

  source_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  target_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);


}
Seline::~Seline(){

}

void Seline::publishPointCloudXYZ(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, std::string frame_id){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);

  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(pcl_pc2, cloud_msg);
  cloud_msg.header.frame_id = frame_id;
  std::cout << "Publish in " << frame_id << "\n";
  pub.publish(cloud_msg);
}

void Seline::runOnce(){
  publishPointCloudXYZ(pub_original_cloud_, *original_model_cloud_, camera_optical_frame_);
  publishPointCloudXYZ(pub_transformed_cloud_, *transformed_cloud_, camera_optical_frame_);
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
