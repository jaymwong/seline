#ifndef SELINE_H
#define SELINE_H

#include <athena/pointcloud/utils.h>
#include <athena/transform/conversions.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include <athena_transform/HomogeneousTransform.h>
#include <std_srvs/Trigger.h>

#include <seline/GetEndEffectorOffset.h>

#define kDefaultLoopRate 25

#define kModelLoadErrorCode -1
#define kICPIterations 1
#define kDownSampleModelLeafSize 0.005  // in meters
#define kDefaultPointCloudLeafSize 0.005 // in meters

#define kSearchEpsilonOnTracking 0.045 // in meters
#define kSearchEpsilonOnGraspedObject 0.175 // in meters
#define kMaxCropDistance 5.0  // in meters
#define kOffsetVectorSize 3

#define kMinEndEffectorTrackingPoints 50
#define kCameraFrameSearchDistance 0.05 // in meters

struct GraspedObjectGeometry{
  double width, depth, length;
};

class Seline{
  public:
    Seline(std::string seline_mode);
    ~Seline();

    void runOnce();

  private:
    ros::NodeHandle nh_;

    std::string seline_pkg_dir_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr original_model_cloud_; // cloud as loaded from .pcd; no transform applied
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_xyz_;

    bool has_seed_, has_point_cloud_, crop_world_plane_, has_grasped_obj_;
    ros::ServiceServer srv_trigger_new_seed_, srv_get_ee_offset_;

    double gripper_length_;     // Distance from base to gripper tip (z-axis)
    double gripper_to_surface_; // Distance from base to the surface (y-axis)

    double ee_crop_theta_, ee_crop_max_y_, epsilon_region_on_gripper_;
    double crop_world_plane_height_;

    PointCloudProperties original_model_cloud_properties_;

    visualization_msgs::Marker grasped_obj_marker_;
    GraspedObjectGeometry grasped_obj_geometry_;

    Eigen::Vector3d observed_offset_;
    Eigen::Matrix4d observed_offset_matrix_;
    Eigen::Vector3d manipulator_translation_;

    pcl::PointXYZ seeded_point_;
    Eigen::Affine3d camera_to_ee_, camera_to_icp_, ee_to_world_, ee_to_manipulator_;
    Eigen::Affine3d world_to_camera_, camera_to_world_, manipulator_to_world_;

    ros::Publisher pub_original_cloud_, pub_transformed_cloud_, pub_segmented_cloud_, pub_icp_out_;
    ros::Publisher pub_est_world_frame_, pub_seg_tracking_cloud_, pub_ee_track_cloud_;
    ros::Publisher pub_grasp_obj_marker_, pub_est_eye_to_hand_frame_;
    ros::Subscriber sub_point_cloud_;

    std::string ee_model_file_, point_cloud_topic_;
    std::string camera_optical_frame_, ee_frame_, world_frame_, manipulator_frame_;
    std::string seline_mode_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_, target_cloud_; // the model cloud and the scene cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener *tf_listener_;
    tf::TransformBroadcaster br_;

    bool lookupKnownTransformations();
    void processEstimatedTransformations();
    void downsampleInitialModelCloud();
    void splicePointCloudByAxis(pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud, std::string axis, double min, double max);

    bool processGraspedObject(Eigen::Matrix4d observed_ee_frame);
    bool processSeed(Eigen::Affine3d seed_transform);
    pcl::PointCloud<pcl::PointXYZ> segmentEndEffectorFromSceneUsingSeed(Eigen::MatrixXd seed_transform, float radius);
    pcl::PointCloud<pcl::PointXYZ> doIterativeRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud);

    void inputCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
    bool triggerSeedCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
    bool getEndEffectorOffsetCallback(seline::GetEndEffectorOffset::Request &req, seline::GetEndEffectorOffset::Response &resp);



};

#endif
