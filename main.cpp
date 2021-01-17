/*
 * @brief Extract ground and non ground points from a *.las point cloud
 * using the Ground Plane Fitting (GPF) algorithm.
 */

#include <chrono>
#include <lasreader.hpp>
#include <laswriter.hpp>
#include "ground_plane_fitting.h"

void loadLas(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string path);
void saveLas(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string path);

int main(void) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>());

    /*
     * Read *.las or *.laz file
     *
     * LASlib includes the LASzip compression library that provides lossless compression
     * for LAS data. This allows you to read or write *.laz (LASzip compressed *.las
     * files) directly like you would an uncompressed .las file using the same LASreader
     * and LASwriter classes.
     */
    loadLas(cloud_in, "input.laz");

    std::cout << "Read Cloud Data Points Size: " << cloud_in->points.size() << std::endl;

    std::cout << "Start Ground Plane Fitting algorithm." << std::endl;

    auto startTime = std::chrono::steady_clock::now();

    GroundPlaneFit fs;
    fs.mainLoop(cloud_in, notground_points, ground_points);

    auto endTime = std::chrono::steady_clock::now();
    auto ellapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "GPF segmentation complete in " << ellapsedTime.count() << "ms." << std::endl; 
    std::cout << "GPF ground points: " << ground_points->points.size() << std::endl;
    std::cout << "GPF non ground points: " << notground_points->points.size() << std::endl;

    // Save the point clouds
    saveLas(notground_points, "notground.las");
    saveLas(ground_points, "ground.las");

    return 0;
}

void loadLas(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string path) {
    LASreadOpener lasreadopener;
    lasreadopener.set_file_name(path.c_str());
    if (!lasreadopener.active()) {
        return;
    }

    LASreader* lasreader = lasreadopener.open();
    cloud->resize(lasreader->header.number_of_point_records);
    size_t step = 0;
    pcl::PointXYZI *pt;
    while (lasreader->read_point()) {
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

void saveLas(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string path) {
    LASwriteOpener laswriteopener;
    laswriteopener.set_file_name(path.c_str());
    laswriteopener.set_format(LAS_TOOLS_FORMAT_LAS); // NOTE: We can also use "LAS_TOOLS_FORMAT_LAZ"
    if (!laswriteopener.active()) {
        return;
    }

    LASheader lasheader;
    lasheader.point_data_format = 1;
    lasheader.point_data_record_length = 35;
    lasheader.x_scale_factor = 0.0001;
    lasheader.y_scale_factor = 0.0001;
    lasheader.z_scale_factor = 0.0001;

    // Init point
    LASpoint laspoint;
    laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, &lasheader);

    LASwriter* laswriter = laswriteopener.open(&lasheader);
    if (laswriter == 0) {
        std::cerr << "Could not open laswriter" << std::endl;
        return;
    }

    for (auto it = cloud->begin(); it != cloud->end(); ++it) {
        laspoint.set_x(it->x);
        laspoint.set_y(it->y);
        laspoint.set_z(it->z);
        laspoint.set_intensity(it->intensity);
        laswriter->write_point(&laspoint);
        laswriter->update_inventory(&laspoint);
    }

    // Update the header
    laswriter->update_header(&lasheader, TRUE);
    laswriter->close();
    delete laswriter;
}

