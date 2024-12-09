#ifndef SIMPLE_LO_HPP
#define SIMPLE_LO_HPP

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <cmath>

// Add Eigen and PCL headers
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

// Add TF broadcaster
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

class SimpleLO
{
public:
    SimpleLO(ros::NodeHandle& nh);
    ~SimpleLO() = default;

private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    
    // Updated scan matching function using PCL
    bool matchScans(const std::vector<float>& current_scan, 
                   const std::vector<float>& previous_scan,
                   double& delta_x, double& delta_y, double& delta_theta);
    
    // Helper functions for point cloud processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertScanToPointCloud(
        const std::vector<float>& scan_points);
    void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    // ROS interface
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher pose_pub_;
    
    // Store previous scan as PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;
    bool first_scan_;
    
    // Current pose using Eigen
    Eigen::Vector3d current_pose_;  // (x, y, theta)
    Eigen::Matrix4f last_transform_;
    
    // ICP object
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    
    // Parameters
    double max_range_;
    double min_range_;
    double max_angular_correction_deg_;
    
    // ICP parameters
    double max_correspondence_distance_;
    int max_iterations_;
    double transformation_epsilon_;
    double fitness_epsilon_;
    double voxel_grid_size_;
    
    // Add TF broadcaster
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // Add frame IDs
    std::string odom_frame_id_;
    std::string base_frame_id_;
    
    // Helper function for TF broadcasting
    void publishTransform(const ros::Time& timestamp);
};

#endif // SIMPLE_LO_HPP 