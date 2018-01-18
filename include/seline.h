#ifndef SELINE_H
#define SELINE_H

#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Core>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#define kDefaultLoopRate 1

#define kModelLoadErrorCode -1

class Seline{
  public:
    Seline();
    ~Seline();

  private:
    ros::NodeHandle nh_;

    std::string seline_pkg_dir_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud_;
};

#endif
