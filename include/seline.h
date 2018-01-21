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


#define kDefaultLoopRate 1

#define kModelLoadErrorCode -1

class Seline{
  public:
    Seline();
    ~Seline();

  private:
    ros::NodeHandle nh_;

    std::string seline_pkg_dir_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr original_model_cloud_; // cloud as loaded from .pcd; no transform applied

    ros::Publisher pub_original_cloud_, pub_transformed_cloud_;

    std::string camera_optical_frame_;

    void publishPointCloudXYZ(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, std::string frame_id);
};

#endif
