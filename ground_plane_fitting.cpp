#include "ground_plane_fitting.h"

GroundPlaneFit::GroundPlaneFit() {
    num_seg_ = 1; // Divide evenly the point cloud into a number of segments // TODO Write this implementation
    num_iter_ = 3; // Number of iterations
    num_lpr_ = 250; // number of points used to estimate the LPR

    th_seeds_ = 1.2; // Threshold for points to be considered initial seeds
    th_dist_ = 0.3; // Threshold distance from the plane

    sensor_height_ = 2.0; // LiDAR sensor height to ground
}

GroundPlaneFit::~GroundPlaneFit() {}

/*
 * @brief Use the set of seed points to estimate the initial plane model 
 * of the ground surface.
 */
model_t GroundPlaneFit::estimatePlane(const pcl::PointCloud<pcl::PointXYZRGB>& seed_points) {
    Eigen::Matrix3f cov_matrix(3, 3);
    Eigen::Vector4f points_mean;
    model_t model;

    // Compute covariance matrix in single pass
    pcl::computeMeanAndCovarianceMatrix(seed_points, cov_matrix, points_mean);

    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_matrix, Eigen::DecompositionOptions::ComputeFullU);

    // Use the least singular vector as normal
    model.normal_n = (svd.matrixU().col(2));

    // Mean ground seeds value
    Eigen::Vector3f seeds_mean = points_mean.head<3>();

    // According to normal.T * [x,y,z] = -d
    model.d = -(model.normal_n.transpose() * seeds_mean)(0, 0);

    return model;
}

/*
 * @brief Extract a set of seed points with low height values.
 */
void GroundPlaneFit::extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr seed_points) {   
    
    // Sort on z-axis values (sortOnHeight)
    std::vector<pcl::PointXYZRGB> cloud_sorted((*cloud_in).points.begin(), (*cloud_in).points.end());
    sort(cloud_sorted.begin(), cloud_sorted.end(), 
        [](pcl::PointXYZRGB p1, pcl::PointXYZRGB p2) {
            // Similar to defining a bool function
            return p1.z < p2.z;
        }
    );

    // Negative outlier error point removal.
    // As there might be some error mirror reflection under the ground
    std::vector<pcl::PointXYZRGB>::iterator it = cloud_sorted.begin();
    for(size_t i = 0; i < cloud_sorted.size(); ++i) {
        // We define the outlier threshold -1.5 times the height of the LiDAR sensor
        if (cloud_sorted[i].z < -1.5 * sensor_height_) {
            it++;
        } else {
            // Points are in incremental order. Therefore, break loop if here
            break;
        }
    }
    // Remove the outlier points
    cloud_sorted.erase(cloud_sorted.begin(), it);

    // Find the Lowest Point Representive (LPR) of the sorted point cloud
    double LPR_height = 0.;
    for(int i = 0; i < num_lpr_; i ++) {
        LPR_height += cloud_sorted[i].z;
    }
    LPR_height /= num_lpr_;

    // Iterate, filter for height less than LPR_height + th_seeds_
    (*seed_points).clear();
    for (size_t i = 0; i < cloud_sorted.size(); ++i) {
        if (cloud_sorted[i].z < LPR_height + th_seeds_) {
            (*seed_points).points.push_back(cloud_sorted[i]);
        }
    }
}

/**
 * @brief Ground Removal based on Ground Plane Fitting (GPF).
 * @refer 
 *   Fast Segmentation of 3D Point Clouds: A Paradigm on LiDAR Data for 
 *   Autonomous Vehicle Applications (ICRA, 2017) 
 */
void GroundPlaneFit::mainLoop(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr notground_points,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_points) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr seed_points(new pcl::PointCloud<pcl::PointXYZRGB>());

    /// 1. Extract initial ground seeds
    extractInitialSeeds(cloud_in, seed_points);

    /// 2. Ground plane fit mainloop
    // The points belonging to the ground surface are used as seeds
    // for the refined estimation of a new plane model and the process 
    // repeats for num_iter_ number of times
    for (int i = 0; i < num_iter_; ++i) {
        /// 3. Estimate the plane model
        model_t model = estimatePlane(*seed_points);

        ground_points->clear();
        notground_points->clear();

        // Pointcloud to matrix
        Eigen::MatrixXf points_matrix(cloud_in->points.size(), 3);
        size_t j = 0u;
        for (auto p : (*cloud_in).points) {
            points_matrix.row(j++) << p.x, p.y, p.z;
        }

        // Ground plane model
        Eigen::VectorXf result = points_matrix * model.normal_n;

        // Threshold filter: N^T xi + d = dist < th_dist ==> N^T xi < th_dist - d
        double th_dist_d_ = th_dist_ - model.d;

        /// 4. Identify which of the points belong to the ground and non ground
        for (int k = 0; k < result.rows(); ++k) {
            if (result[k] < th_dist_d_) {
                // TODO think about a more optimized code for this part
                pcl::PointXYZRGB point;
                point.x = cloud_in->points[k].x;
                point.y = cloud_in->points[k].y;
                point.z = cloud_in->points[k].z;
                point.r = cloud_in->points[k].r;
                point.g = cloud_in->points[k].g;
                point.b = cloud_in->points[k].b;
                ground_points->points.push_back(point);
            }
            else {
                pcl::PointXYZRGB point;
                point.x = cloud_in->points[k].x;
                point.y = cloud_in->points[k].y;
                point.z = cloud_in->points[k].z;
                point.r = cloud_in->points[k].r;
                point.g = cloud_in->points[k].g;
                point.b = cloud_in->points[k].b;

                notground_points->points.push_back(point);
            }
        }
    }
}