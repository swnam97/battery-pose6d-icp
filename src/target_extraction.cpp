#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ PointT;

class TargetExtraction
{
public:
    TargetExtraction()
    {
        // ROS 노드 초기화 및 구독자와 퍼블리셔 설정
        point_cloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &TargetExtraction::pointCloudCallback, this);
        filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_battery_pack", 1);
        clustered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/clustered_battery_pack", 1);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // ROS 메시지를 PCL 포인트 클라우드로 변환
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*msg, *cloud);
        ROS_INFO("Received point cloud with %lu points", cloud->points.size());

        // 깊이 기반 필터링 (z 값 범위 설정)
        pcl::PointCloud<PointT>::Ptr filtered_cloud_z(new pcl::PointCloud<PointT>);
        pcl::PassThrough<PointT> pass_z;
        pass_z.setInputCloud(cloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(0.5, 1.0); // 배터리팩의 z 값 범위, 실제 환경에 맞게 조정 필요
        pass_z.filter(*filtered_cloud_z);

        // x 값 범위 설정
        pcl::PointCloud<PointT>::Ptr filtered_cloud_xy(new pcl::PointCloud<PointT>);
        pcl::PassThrough<PointT> pass_x;
        pass_x.setInputCloud(filtered_cloud_z);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(-1.0, 1.0); // x 좌표의 범위 (-1m ~ +1m)
        pass_x.filter(*filtered_cloud_xy);

        // y 값 범위 설정
        pcl::PassThrough<PointT> pass_y;
        pass_y.setInputCloud(filtered_cloud_xy);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-1.0, 1.0); // y 좌표의 범위 (-1m ~ +1m)
        pass_y.filter(*filtered_cloud_xy);

        // 필터링된 포인트 클라우드를 ROS 메시지로 변환하여 퍼블리시
        sensor_msgs::PointCloud2 filtered_output;
        pcl::toROSMsg(*filtered_cloud_xy, filtered_output);
        filtered_output.header = msg->header; // 원본 메시지의 헤더를 유지하여 시간 동기화
        filtered_cloud_pub_.publish(filtered_output);

        // 필터링 이후, 다운샘플링 추가
        pcl::PointCloud<PointT>::Ptr downsampled_cloud(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> voxel_grid;
        voxel_grid.setInputCloud(filtered_cloud_xy);
        // voxel_grid.setLeafSize(0.02f, 0.02f, 0.02f); // 2cm 간격으로 다운샘플링
        voxel_grid.setLeafSize(0.005f, 0.005f, 0.005f); // 2cm 간격으로 다운샘플링
        voxel_grid.filter(*downsampled_cloud);
        ROS_INFO("After downsampling: %lu points", downsampled_cloud->points.size());

        // 클러스터링을 위한 KD 트리 설정
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(downsampled_cloud);

        // 유클리드 클러스터링 설정
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(0.08); // 클러스터링 거리 허용 범위 (m 단위, 필요시 조정)
        ec.setMinClusterSize(1000);    // 최소 클러스터 크기 (포인트 개수)
        ec.setMaxClusterSize(20000);   // 최대 클러스터 크기 (포인트 개수)
        ec.setSearchMethod(tree);
        ec.setInputCloud(downsampled_cloud);
        ec.extract(cluster_indices);
        ROS_INFO("Number of clusters found: %lu", cluster_indices.size());

        // 가장 큰 클러스터 선택
        pcl::PointCloud<PointT>::Ptr largest_cluster(new pcl::PointCloud<PointT>);
        if (!cluster_indices.empty())
        {
            std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
            largest_cluster->width = it->indices.size();
            largest_cluster->height = 1;
            largest_cluster->is_dense = true;
            largest_cluster->points.resize(it->indices.size());

            for (size_t i = 0; i < it->indices.size(); ++i)
                largest_cluster->points[i] = downsampled_cloud->points[it->indices[i]];
            ROS_INFO("Largest cluster size: %lu points", largest_cluster->points.size());
        }
        else
        {
            ROS_WARN("No clusters found!");
        }

        // 클러스터링된 포인트 클라우드를 ROS 메시지로 변환하여 퍼블리시
        sensor_msgs::PointCloud2 clustered_output;
        pcl::toROSMsg(*largest_cluster, clustered_output);
        clustered_output.header = msg->header; // 원본 메시지의 헤더를 유지하여 시간 동기화
        clustered_cloud_pub_.publish(clustered_output);
        ROS_INFO("Published clustered point cloud");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher filtered_cloud_pub_;
    ros::Publisher clustered_cloud_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_extraction");
    TargetExtraction target_extraction;
    ros::spin();
    return 0;
}
