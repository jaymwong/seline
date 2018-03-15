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
#include <transform_conversions/HomogeneousTransform.h>
#include <std_srvs/Trigger.h>

#define kDefaultLoopRate 25

#define kModelLoadErrorCode -1
#define kICPIterations 1
#define kDownSampleModelLeafSize 0.005  // in meters
#define kDefaultPointCloudLeafSize 0.005 // in meters

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

    bool has_seed_, has_point_cloud_, crop_world_plane_;
    ros::ServiceServer srv_trigger_new_seed_;

    double gripper_length_;
    double ee_crop_theta_, ee_crop_max_y_, epsilon_region_on_gripper_;
    double crop_world_plane_height_;

    pcl::PointXYZ seeded_point_;
    Eigen::Matrix4d camera_to_ee_, camera_to_icp_, ee_to_world_;
    Eigen::Matrix4d world_to_camera_, camera_to_world_;

    ros::Publisher pub_original_cloud_, pub_transformed_cloud_, pub_segmented_cloud_, pub_icp_out_;
    ros::Publisher pub_est_world_frame_;
    ros::Subscriber sub_point_cloud_;

    std::string ee_model_file_, point_cloud_topic_;
    std::string camera_optical_frame_, ee_frame_, world_frame_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_, target_cloud_; // the model cloud and the scene cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener *tf_listener_;
    tf::TransformBroadcaster br_;

    bool lookupKnownTransformations();
    void processEstimatedTransformations();
    void downsampleInitialModelCloud();
    void splicePointCloudByAxis(pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud, std::string axis, double min, double max);

    void processSeed(Eigen::Matrix4d matrix);
    void publishPointCloudXYZ(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, std::string frame_id);
    pcl::PointCloud<pcl::PointXYZ> segmentEndEffectorFromSceneUsingSeed(Eigen::MatrixXd seed_transform);
    pcl::PointCloud<pcl::PointXYZ> doIterativeRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud);


    void inputCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
    bool triggerSeedCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);


};

#endif
