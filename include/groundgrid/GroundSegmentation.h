/*
Copyright 2023 Dahlem Center for Machine Learning and Robotics, Freie Universität Berlin

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions
and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or other materials provided
with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

// ros
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

// Pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "velodyne_pointcloud/point_types.h"

// Grid map
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/GridMapCvConverter.hpp>

// Config
#include <groundgrid/GroundGridConfig.h>


namespace groundgrid {
class GroundSegmentation {
  public:
    typedef velodyne_pointcloud::PointXYZIR PCLPoint;

    GroundSegmentation() {
      file_csv.open("/root/catkin_ws/src/bags/output_timeseries.csv", std::ios::app);
    };
    void init(ros::NodeHandle& nodeHandle, const size_t dimension, const float& resolution);
    pcl::PointCloud<PCLPoint>::Ptr filter_cloud(const pcl::PointCloud<PCLPoint>::Ptr cloud, const PCLPoint& cloudOrigin, const geometry_msgs::TransformStamped& mapToBase, grid_map::GridMap &map);
    void insert_cloud(const pcl::PointCloud<PCLPoint>::Ptr cloud, const size_t start, const size_t end, const PCLPoint& cloudOrigin, std::vector<std::pair<size_t, grid_map::Index> >& point_index, std::vector<std::pair<size_t, grid_map::Index> >& ignored, std::vector<size_t>& outliers, grid_map::GridMap &map);
    void setConfig(const groundgrid::GroundGridConfig& config);
    // section defines the section  of the map to process (0: top-left, 1: top-right, 2: bottom-left, 3: bottom-right)
    // used for parallel execution
    void detect_ground_patches(grid_map::GridMap &map, unsigned short section) const;
    template<int S> void detect_ground_patch(grid_map::GridMap &map, size_t i, size_t j) const;
    void spiral_ground_interpolation(grid_map::GridMap &map, const geometry_msgs::TransformStamped &toBase) const;
    void interpolate_cell(grid_map::GridMap &map, const size_t x, const size_t y) const;

protected:
    groundgrid::GroundGridConfig mConfig;
    grid_map::Matrix expectedPoints;

    // velodyne 128: Average distance in rad on the unit circle of the appr. 220k points per round/128 Lasers

    // Vertical angle spacing between beams
    //    - For the Ouster O1 with 128 rings:
    //          - FOV = 45°   -->   45/127
    //          - This gives 0.3543 degrees ...
    //          - ... which is 0.00614 radians
    const float verticalPointAngDist = 0.00614;

    // Minimum distance (SQUARED) for the points to be considered
    //    - I'd set it to 2m
    //    - When squared, becomes 4.0
    const float param_minDistSquared = 4.0f;

    // // Parameters to perform the padding around the car. For now not used
    // float parameter_ZeroPaddingLeft = 2.5;
    // float parameter_ZeroPaddingRight = 2.0;
    // float parameter_ZeroPaddingFront = 2.5;
    // float parameter_ZeroPaddingBehind = 4.0;






    /************************************ PARAMETERS FROM CONFIG ************************************/

    // Original: "point_count_cell_variance_threshold" = 10
    // If number of points per cell is lower, the variance is computed as an average of the neighbouring patch
    const int param_PointPerCellThresholdForVariance = 5;                                           // Controllato, giusto metterlo a 5 per avere
                                                                                                    // delle celle sia di front che dietro

    // Original: "max_ring" = 1024
    // If point is out of the interval, it is not considered for the ground segmentation
    const int param_MinRing = 70;
    const int param_MaxRing = 128;

    // Original: "distance_factor" = 0.0001
    // Original: "minimum_distance_factor" = 0.0005
    // Original: "minimum_distance_factor*10" = 0.005
    // Parameters for the computation of the variance (normal, minimum, maximum)
    const double param_OffsetCoefficientForVariance = 0.0002;                                     // Giusto compromesso sembra 0.0075 moltiplicato
                                                                                                  // per la sq_dist
    const double param_MinimumVarianceThr = 0.000001;
    const double param_MaximumVarianceThr = 0.00065;
    const double param_PowerDistForVariance = -1.65;

    // Original: "miminum_point_height_threshold" = 0.3
    // Original: "miminum_point_height_obstacle_threshold" = 0.1
    // Boh non so serve per cose strane, li lascio come stanno dai che è meglio
    const double param_PointHeightThrForGround = 0.3;
    const double param_PointHeightThrForObstacle = 0.1;

    // Original: "outlier_tolerance" = 0.1
    // Needed to estimate outliers (when below the ground level)
    const double param_OutlierTolerance = 1.0;

    // Original: hard-coded = 0.5
    // Original: was "outlier_tolerance" but I divided them = 0.1
    // Needed to exit if the estimation is going too much upwards with a high previous confidence
    const double param_OldConfidenceThreshold = 0.95;
    const double param_EstimationUpwardTolerance = 0.5;

    // Original: "ground_patch_detection_minimum_point_count_threshold" = 0.25
    // In percentage, the number of points needed in a patch in order to accept it as "interesting"
    const double param_PatchPercentageThrForFiltering = 0.4;                                          // Dopo aver cambiato il verticalCoso, rimane
                                                                                                      // perfetto al 50% (40% per margine)
    // Original: "patch_size_change_distance" = 20
    // The distance at which the patch size becomes 5x5 instead of 3x3
    const double param_DistanceChangePatchSize = 25;

    // Original: "occupied_cells_decrease_factor" = 0.8 (not really but I changed the implementation)
    // Original: "occupied_cells_point_count_factor" = 20
    // The decrease factor for the confidence, when a cell is interpolated.
    // The decrease factor for the number of points per block, in order to compute the confidence
    const double param_ConfidenceDecreaseFactorInterpolation = 1.0;
    const double param_ConfidenceDecreaseFactorPointNumber = 40;



    const float param_OldMemory = 3.0;



    mutable std::ofstream file_csv;

};
}
