/*
 * @brief Extract ground and non ground points from a *.las point cloud
 * using the Ground Plane Fitting (GPF) algorithm.
 */

#include <chrono>
#include <lasreader.hpp>
#include <laswriter.hpp>
#include "ground_plane_fitting.h"

void loadLas(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string path);

int main(void) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>());

    // TODO read laz file (compressed *.las file)

    // Read the las point cloud file
    loadLas(cloud_in, "filtered_2386_9702.las");

    std::cerr << "Read Cloud Data Points Size: " << cloud_in->points.size() << std::endl;

    std::cout << "Start Ground Plane Fitting algorithm." << std::endl;

    auto startTime = std::chrono::steady_clock::now();

    GroundPlaneFit fs;
    fs.run(cloud_in, notground_points, ground_points);

    auto endTime = std::chrono::steady_clock::now();
    auto ellapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cerr << "GPF segmentation complete in " << ellapsedTime.count() << "ms." << std::endl; 
    std::cerr << "GPF ground points: " << ground_points->points.size() << std::endl;
    std::cerr << "GPF non ground points: " << notground_points->points.size() << std::endl;

    // Save the point clouds
    pcl::PCDWriter writer;
    ground_points->width = 1;
    ground_points->height = ground_points->points.size();
    writer.write("ground.pcd", *ground_points);

    notground_points->width = 1;
    notground_points->height = notground_points->points.size();
    writer.write("notground.pcd", *notground_points);

    return 0;
}

void loadLas(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string path)
{
    LASreadOpener lasreadopener;
    lasreadopener.set_file_name(path.c_str());
    if (!lasreadopener.active())
    {
        return;
    }
    LASreader* lasreader = lasreadopener.open();
    cloud->resize(lasreader->header.number_of_point_records);
    size_t step = 0;
    pcl::PointXYZI *pt;
    while (lasreader->read_point())
    {
        pt = &cloud->at(step);
        pt->x = lasreader->point.get_x();
        pt->y = lasreader->point.get_y();
        pt->z = lasreader->point.get_z();
        pt->intensity = lasreader->point.get_intensity();

        ++step;
    }
    lasreader->close();
    delete lasreader;

}