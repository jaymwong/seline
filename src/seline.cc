#include "seline.h"

Seline::Seline(){

  std::string ee_model = "robotiq85_base_link.pcd";
  seline_pkg_dir_ = ros::package::getPath("seline");
  std::string path_to_model = seline_pkg_dir_ + "/models/" + ee_model;

  model_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (path_to_model, *model_cloud_) != kModelLoadErrorCode){
    std::cout << "Successfully loaded file " << ee_model << " (" << model_cloud_->size () << " points)\n";
  }

  //pcl::PointCloud<pcl::PointXYZ>

  Eigen::Affine3f seed_transform;
  seed_transform.matrix() = Eigen::MatrixXf::Identity(4, 4);

  // TODO: A test... display the original pointcloud a little bit...
  seed_transform.translation() = Eigen::Vector3f(1,2,3);

  std::cout << seed_transform.matrix() << "\n";

}
Seline::~Seline(){

}

int main(int argc, char** argv){
  ros::init(argc, argv, "seline");
  Seline *node = new Seline();
  ros::Rate *loop_rate = new ros::Rate(kDefaultLoopRate);
  while (ros::ok()){
    //node->runOnce();
    loop_rate->sleep();
    ros::spinOnce();
  }
  return 0;
}
