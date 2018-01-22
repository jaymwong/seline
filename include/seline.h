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


#define kDefaultLoopRate 1

#define kModelLoadErrorCode -1
#define kDownSampleModelLeafSize 0.005

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

    ros::Publisher pub_original_cloud_, pub_transformed_cloud_;

    std::string camera_optical_frame_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_, target_cloud_; // the model cloud, and the scene cloud

    void publishPointCloudXYZ(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, std::string frame_id);
};

#endif
