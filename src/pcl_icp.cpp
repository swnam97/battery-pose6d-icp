#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

typedef pcl::PointXYZ PointT;

class ICPAlignment
{
public:
    ICPAlignment() : first_callback_(true) // initialize first_callback_ flag as true
    {
        // ROS subscriptions and publishers setup
        point_cloud_sub_ = nh_.subscribe("/clustered_battery_pack", 1, &ICPAlignment::pointCloudCallback, this);
        aligned_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/aligned_cloud", 1);
        target_pcd_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/target_pcd", 1);

        // Load reference cloud
        if (pcl::io::loadPCDFile<PointT>("/home/seungwon/catkin_ws/src/rel_pose_estimator/models/niro_stl_filtered.pcd", *reference_cloud_) == -1)
        {
            ROS_ERROR("Couldn't read reference_cloud.pcd file.");
            return;
        }
        ROS_INFO("Loaded reference cloud with %lu points", reference_cloud_->points.size());

        // Set up a periodic timer for publishing the target PCD
        timer_ = nh_.createTimer(ros::Duration(0.5), &ICPAlignment::publishTargetPCD, this);

        // Initialize initial transformation matrix as identity
        current_transform_ = Eigen::Matrix4f::Identity();
    }

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::PointCloud<PointT>::Ptr input_cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*msg, *input_cloud);
        ROS_INFO("Received target cloud with %lu points", input_cloud->points.size());

        // Apply initial transformation only for the first callback
        if (first_callback_)
        {
            initial_transform_ = Eigen::Matrix4f::Identity();
            // initial_transform_(0, 3) = -0.0254;
            // initial_transform_(1, 3) = -0.0046;
            // initial_transform_(2, 3) = -0.6420;
            initial_transform_(0, 3) = -0.1054;
            initial_transform_(1, 3) = -0.0846;
            initial_transform_(2, 3) = -0.5420;

            // Example quaternion values to extract yaw
            // float qx = -0.0016201, qy = 0.0002435, qz = -0.3849002, qw = 0.92295673;
            float qx = 0.0, qy = 0.0, qz = 0.25882, qw = 0.96593;
            double roll, pitch, yaw;
            quaternionToRPY(qx, qy, qz, qw, roll, pitch, yaw);
            ROS_INFO("Initial Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);

            initial_transform_(0, 0) = cos(yaw);
            initial_transform_(0, 1) = -sin(yaw);
            initial_transform_(1, 0) = sin(yaw);
            initial_transform_(1, 1) = cos(yaw);

            // Update the transformation matrix with initial transformation only once
            current_transform_ = initial_transform_;
            first_callback_ = false;
        }

        // Apply the current transformation to the input cloud
        pcl::PointCloud<PointT>::Ptr transformed_input_cloud(new pcl::PointCloud<PointT>);
        pcl::transformPointCloud(*input_cloud, *transformed_input_cloud, current_transform_);

        // Perform ICP
        pcl::PointCloud<PointT>::Ptr aligned_cloud(new pcl::PointCloud<PointT>);
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setInputSource(transformed_input_cloud);
        icp.setInputTarget(reference_cloud_);
        icp.setMaximumIterations(50);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-8);

        icp.align(*aligned_cloud);

        if (icp.hasConverged())
        {
            ROS_INFO("ICP has converged with score: %f", icp.getFitnessScore());
            ROS_INFO_STREAM("Transformation matrix:\n" << icp.getFinalTransformation());

            // Update current transformation matrix for the next iteration
            current_transform_ = icp.getFinalTransformation() * current_transform_;

            // Publish the aligned cloud
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*aligned_cloud, output);
            output.header = msg->header;
            aligned_cloud_pub_.publish(output);
        }
        else
        {
            ROS_WARN("ICP did not converge.");
        }
    }

    void publishTargetPCD(const ros::TimerEvent&)
    {
        sensor_msgs::PointCloud2 target_pcd_msg;
        pcl::toROSMsg(*reference_cloud_, target_pcd_msg);
        target_pcd_msg.header.frame_id = "camera_rgb_optical_frame";
        target_pcd_pub_.publish(target_pcd_msg);
        ROS_INFO_ONCE("Published target PCD to /target_pcd topic");
    }

    void quaternionToRPY(double qx, double qy, double qz, double qw, double &roll, double &pitch, double &yaw)
    {
        tf2::Quaternion q(qx, qy, qz, qw);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
    }

    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher aligned_cloud_pub_;
    ros::Publisher target_pcd_pub_;
    ros::Timer timer_;

    pcl::PointCloud<PointT>::Ptr reference_cloud_{new pcl::PointCloud<PointT>};
    bool first_callback_;
    Eigen::Matrix4f current_transform_;
    Eigen::Matrix4f initial_transform_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_icp");
    ICPAlignment icp_alignment;
    ros::spin();
    return 0;
}