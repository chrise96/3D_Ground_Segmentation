#include <chrono>
#include "ground_plane_fitting.h"

/*
 * @brief Extract ground and non ground points from a *.pcd point cloud
 * using the Ground Plane Fitting (GPF) algorithm.
 */
int main(void) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::PCDReader reader;
    reader.read("input.pcd", *cloud_in); // TODO also implement *.laz format
    std::cerr << "Read Cloud Data Points Size: " << cloud_in->points.size() << std::endl;

    std::cout << "Start Ground Plane Fitting algorithm." << std::endl;

    auto startTime = std::chrono::steady_clock::now();

    GroundPlaneFit fs;
    fs.mainLoop(cloud_in, notground_points, ground_points);

    auto endTime = std::chrono::steady_clock::now();
    auto ellapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cerr << "GPF segmentation complete in " << ellapsedTime.count() << "ms." << std::endl; 
    std::cerr << "GPF ground points: " << ground_points->points.size() << std::endl;
    std::cerr << "GPF non ground points: " << notground_points->points.size() << std::endl;

    // Save points belonging to ground and non ground surface
    pcl::PCDWriter writer;
    ground_points->width = 1;
    ground_points->height = ground_points->points.size();
    writer.write("ground.pcd", *ground_points);

    notground_points->width = 1;
    notground_points->height = notground_points->points.size();
    writer.write("notground.pcd", *notground_points);

    return 0;
}
