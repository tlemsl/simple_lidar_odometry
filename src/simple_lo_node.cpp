#include <pcl_conversions/pcl_conversions.h>

#include <simple_lo/simple_lo.hpp>

SimpleLO::SimpleLO(ros::NodeHandle& nh)
    : nh_(nh),
      first_scan_(true),
      current_pose_(Eigen::Vector3d::Zero()),
      last_transform_(Eigen::Matrix4f::Identity()),
      previous_cloud_(new pcl::PointCloud<pcl::PointXYZ>()) {
    // Get basic parameters
    nh_.param("max_range", max_range_, 30.0);
    nh_.param("min_range", min_range_, 0.1);
    nh_.param("max_angular_correction_deg", max_angular_correction_deg_, 20.0);

    // Get ICP parameters
    nh_.param("max_correspondence_distance", max_correspondence_distance_, 0.5);
    nh_.param("max_iterations", max_iterations_, 30);
    nh_.param("transformation_epsilon", transformation_epsilon_, 1e-8);
    nh_.param("fitness_epsilon", fitness_epsilon_, 1e-6);
    nh_.param("voxel_grid_size", voxel_grid_size_, 0.05);

    // Get frame IDs from parameters
    nh_.param<std::string>("odom_frame", odom_frame_id_, "odom");
    nh_.param<std::string>("base_frame", base_frame_id_, "base_link");

    // Configure ICP
    icp_.setMaxCorrespondenceDistance(max_correspondence_distance_);
    icp_.setMaximumIterations(max_iterations_);
    icp_.setTransformationEpsilon(transformation_epsilon_);
    icp_.setEuclideanFitnessEpsilon(fitness_epsilon_);

    // Setup ROS interface
    scan_sub_ = nh_.subscribe("/scan", 1, &SimpleLO::scanCallback, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 50);
}

void SimpleLO::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    // Convert current scan to point cloud
    std::vector<float> current_points;
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        if (scan->ranges[i] >= min_range_ && scan->ranges[i] <= max_range_) {
            float angle = scan->angle_min + i * scan->angle_increment;
            current_points.push_back(scan->ranges[i] * cos(angle));
            current_points.push_back(scan->ranges[i] * sin(angle));
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud =
        convertScanToPointCloud(current_points);

    // Filter the point cloud
    filterPointCloud(current_cloud);

    if (first_scan_) {
        *previous_cloud_ = *current_cloud;
        first_scan_ = false;
        return;
    }

    // Align clouds using ICP
    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    icp_.setInputSource(current_cloud);
    icp_.setInputTarget(previous_cloud_);
    icp_.align(aligned_cloud);

    if (icp_.hasConverged()) {
        // Get the transformation
        Eigen::Matrix4f transform = icp_.getFinalTransformation();

        // Extract delta movement
        double delta_x = transform(0, 3);
        double delta_y = transform(1, 3);
        double delta_theta = atan2(transform(1, 0), transform(0, 0));

        // Update pose
        current_pose_[0] +=
            delta_x * cos(current_pose_[2]) - delta_y * sin(current_pose_[2]);
        current_pose_[1] +=
            delta_x * sin(current_pose_[2]) + delta_y * cos(current_pose_[2]);
        current_pose_[2] += delta_theta;

        // Publish odometry
        nav_msgs::Odometry odom;
        odom.header.stamp = scan->header.stamp;
        odom.header.frame_id = odom_frame_id_;
        odom.child_frame_id = base_frame_id_;

        odom.pose.pose.position.x = current_pose_[0];
        odom.pose.pose.position.y = current_pose_[1];
        odom.pose.pose.position.z = 0.0;

        // Convert theta to quaternion
        double half_theta = current_pose_[2] / 2.0;
        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = sin(half_theta);
        odom.pose.pose.orientation.w = cos(half_theta);

        odom_pub_.publish(odom);

        // Publish pose
        geometry_msgs::PoseStamped pose;
        pose.header = odom.header;
        pose.pose = odom.pose.pose;
        pose_pub_.publish(pose);

        // Publish transform
        // publishTransform(scan->header.stamp);

        // Update previous cloud
        *previous_cloud_ = *current_cloud;
    } else {
        ROS_WARN("ICP did not converge!");
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SimpleLO::convertScanToPointCloud(
    const std::vector<float>& scan_points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < scan_points.size(); i += 2) {
        pcl::PointXYZ point;
        point.x = scan_points[i];
        point.y = scan_points[i + 1];
        point.z = 0.0;
        cloud->push_back(point);
    }

    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = true;

    return cloud;
}

void SimpleLO::filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // Remove NaN points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    // Voxel grid filtering for downsampling
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(voxel_grid_size_, voxel_grid_size_,
                           voxel_grid_size_);
    voxel_grid.filter(*cloud);
}

void SimpleLO::publishTransform(const ros::Time& timestamp) {
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = timestamp;
    transform_stamped.header.frame_id = odom_frame_id_;
    transform_stamped.child_frame_id = base_frame_id_;

    // Set translation
    transform_stamped.transform.translation.x = current_pose_[0];
    transform_stamped.transform.translation.y = current_pose_[1];
    transform_stamped.transform.translation.z = 0.0;

    // Set rotation
    double half_theta = current_pose_[2] / 2.0;
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = sin(half_theta);
    transform_stamped.transform.rotation.w = cos(half_theta);

    // Broadcast transform
    tf_broadcaster_.sendTransform(transform_stamped);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_lo_node");
    ros::NodeHandle nh("~");

    SimpleLO simple_lo(nh);

    ros::spin();
    return 0;
}