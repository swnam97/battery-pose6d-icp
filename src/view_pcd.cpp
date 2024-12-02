#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    // Load the PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/seungwon/catkin_ws/src/rel_pose_estimator/models/niro2_filtered.pcd", *cloud) == -1)
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/seungwon/catkin_ws/src/rel_pose_estimator/models/niro_stl.pcd", *cloud) == -1)
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/seungwon/catkin_ws/src/rel_pose_estimator/models/niro_stl_filtered.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read the PCD file.\n");
        return -1;
    }

    // Create a PCLVisualizer object
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCD Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    // Add a coordinate system for visualization (1.0 is the scale of the axes)
    viewer->addCoordinateSystem(1.0);

    // Keep the viewer open until 'q' is pressed
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }

    return 0;
}
