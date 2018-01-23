#ifndef SELINE_H
#define SELINE_H

#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Core>

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#define kDefaultLoopRate 1

#define kModelLoadErrorCode -1
#define kDownSampleModelLeafSize 0.005
#define kDefaultPointCloudLeafSize 0.01

class Seline{
  public:
    Seline();
    ~Seline();

    void runOnce();

  private:
    ros::NodeHandle nh_;

    std::string seline_pkg_dir_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr original_model_cloud_; // cloud as loaded from .pcd; no transform applied
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_xyz_;


    pcl::PointXYZ seeded_point_;
    bool has_seed_;
    Eigen::Matrix4f transformation_matrix_; // camera -> ee

    ros::Publisher pub_original_cloud_, pub_transformed_cloud_;
    ros::Subscriber sub_point_cloud_;

    std::string camera_optical_frame_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_, target_cloud_; // the model cloud, and the scene cloud

    void downsampleInitialModelCloud();
    void processSeed(Eigen::Matrix4f matrix);
    void publishPointCloudXYZ(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, std::string frame_id);
    void doIterativeRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud);


    void inputCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);

};

#endif
