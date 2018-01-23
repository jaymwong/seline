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
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>
#include <transform/conversions.h>

#define kDefaultLoopRate 25

#define kModelLoadErrorCode -1
#define kICPIterations 1
#define kDownSampleModelLeafSize 0.005  // in meters
#define kDefaultPointCloudLeafSize 0.005 // in meters
#define kGripperLength 0.15748  // in meters from the base to the finger tip

// How much epsilon to add on top of the gripper size (e.g. kGripperLength/2.0)
// To be used as a radius on which we grab points off the scene cloud. Too large
// will pull unnecessary points, too small and we may not be able to fall into
// the basin of convergence for ICP, due to lacking sufficient scene points
#define kEpsilonRegionOnGripper 0.05

#define kEndEffectorCropTheta 10 // in degrees
#define kEndEffectorCropMaxY 0.2 // in meters

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


    bool has_seed_, has_point_cloud_;

    pcl::PointXYZ seeded_point_;
    Eigen::Matrix4d camera_to_ee_, world_to_camera_, camera_to_icp_, ee_to_world_;

    ros::Publisher pub_original_cloud_, pub_transformed_cloud_, pub_segmented_cloud_, pub_icp_out_;
    ros::Subscriber sub_point_cloud_;

    std::string camera_optical_frame_, ee_frame_, world_frame_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_, target_cloud_; // the model cloud and the scene cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener *tf_listener_;
    tf::TransformBroadcaster br_;

    void lookupTransformations();
    void downsampleInitialModelCloud();
    void splicePointCloudByAxis(pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud, std::string axis, double min, double max);

    void processSeed(Eigen::Matrix4d matrix);
    void publishPointCloudXYZ(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, std::string frame_id);
    pcl::PointCloud<pcl::PointXYZ> segmentEndEffectorFromSceneUsingSeed(Eigen::MatrixXd seed_transform);
    pcl::PointCloud<pcl::PointXYZ> doIterativeRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud);


    void inputCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);

};

#endif
