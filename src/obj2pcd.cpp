#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>  // Make sure to include this for saving as .pcd
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/uniform_sampling.h>

int main()
{
    // Declare the mesh
    pcl::PolygonMesh mesh;

    PCL_ERROR("Fake error 1. \n");

    // Load the OBJ file
    if (pcl::io::loadOBJFile("src/rel_pose_estimator/models/niro_modified.obj", mesh) == -1)
    // if (pcl::io::loadOBJFile("src/rel_pose_estimator/models/niro.obj", mesh) == -1)
    {
        PCL_ERROR("Failed to load OBJ file.\n");
        return -1;
    }

    PCL_ERROR("Fake error 2. \n");

    // Convert mesh to point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

    // Downsample the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::UniformSampling<pcl::PointXYZ> sampler;
    sampler.setInputCloud(cloud);
    sampler.setRadiusSearch(0.01); // Set sampling radius
    sampler.filter(*cloud_sampled);

    // Save the result as a PCD file
    pcl::io::savePCDFileASCII("src/rel_pose_estimator/models/niro_modified.pcd", *cloud_sampled);
    // pcl::io::savePCDFileASCII("src/rel_pose_estimator/models/niro.pcd", *cloud_sampled);
    // pcl::io::savePCDFileASCII("src/rel_pose_estimator/obj/test.pcd", *cloud);

    return 0;
}
