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

#include <groundgrid/GroundSegmentation.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros/console.h>

#include <chrono>
#include <unordered_map>
#include <algorithm>
#include <thread>

using namespace groundgrid;


void GroundSegmentation::init(ros::NodeHandle& nodeHandle, const size_t dimension, const float& resolution){
    const size_t cellCount = std::round(dimension/resolution);
    expectedPoints.resize(cellCount, cellCount);

    for(size_t i=0; i<cellCount; ++i){
        for(size_t j=0; j<cellCount; ++j){

            // Compute the distance (in meters)
            const double dist = resolution * 
                std::hypot(
                    static_cast<double>(i) - static_cast<double>(cellCount) / 2.0,
                    static_cast<double>(j) - static_cast<double>(cellCount) / 2.0
            );

            // Skip, if the distance is smaller than the resolution
            if (dist < resolution*2) {
                expectedPoints(i, j) = 100.0; // avoid division by zero at the center
                continue;
            }

            // Compute the "short" distance
            const double dist_short = dist-resolution;
            // Compute the "big" angle (to the short distance)
            const double big_angle = std::atan2(param_CarLidarHeight, dist_short);
            // Compute the "small" angle (to the longer distance)
            const double small_angle = std::atan2(param_CarLidarHeight, dist);

            // Compute the vertical angle relative to the cell
            const double angle_vertical = big_angle-small_angle;
            // Compute the horizontal angle relative to the cell
            const double angle_horizontal = 2 * std::atan2(resolution/2,dist);

            // Compute the expected vertical and horizontal points
            const double horizontal_points = angle_horizontal * horizontalPointAngDist;
            const double vertical_points = angle_vertical * verticalPointAngDist;

            expectedPoints(i, j) = horizontal_points*vertical_points;
        }
    }
    Eigen::initParallel();
}



/*

    This is the main function. Through this function, all the others are called.

        - New layers are initialized on the Gridmap
            - Before: 
                - points
                - ground 
                - groundpatch 
                - minGroundHeight
                - maxGroundHeight
            - Added now:
                - groundCandidates
                - planeDist
                - m2
                - meanVariance
                - pointsRaw
                - variance

        - Calls "self->detect_ground_patches()"
            - Which in turn calls "self->detect_ground_patch < S > "
                - S = the parameter defining the width of the grid for the estimation

            - Step 1: ignore not-populated areas
                - done for the whole SxS patch, NOT for the single grid
                - uses the parameter "ground_patch_detection_minimum_point_count_threshold"

            - Step 2: compute "variance" and "estimated ground level"
                - variance is either
                    - of the single cell, if it has enough points (param_PointPerCellThresholdForVariance)
                    - an average of the whole patch, otherwise
                    - PAY ATTENTION!! IT IS NOT SCALED WRT TO DISTANCE OR EXPECTED
                - estimated ground level is
                    - the weighted average of the lowest point of the cells in the patch
                    - weighted w.r.t. the number of points in the cell VS total pts in the patch
                - variance threshold is:
                    - at least (param_MinimumVarianceThr)^2
                    - maximum (param_MaximumVarianceThr)^2
                    - normally (param_VarianceDistanceMultiplier)^2 * squared_distance

            - Step 3: if old confidence was high, and the estimation rises, do not update

            - Step 4: update stuff
                - if   ( var<var_thr )   AND   (the cell is well-observed, using "ground_patch_detection_minimum_point_count_threshold")
                    - update confidence (using "occupied_cells_point_count_factor") -->  STRANGE FORMULA, CHECK IT
                    - update ground height
                - elif   (the ground estimate was going down)
                    - update anyways

*/

pcl::PointCloud<GroundSegmentation::PCLPoint>::Ptr GroundSegmentation::filter_cloud(const pcl::PointCloud<PCLPoint>::Ptr cloud, const PCLPoint& cloudOrigin, const geometry_msgs::TransformStamped& mapToBase, grid_map::GridMap &map)
{
    auto start = std::chrono::steady_clock::now();
    static double avg_insertion_time = 0.0;
    static double avg_detection_time = 0.0;
    static double avg_segmentation_time = 0.0;
    static unsigned int time_vals = 0;

    pcl::PointCloud<PCLPoint>::Ptr filtered_cloud (new pcl::PointCloud<PCLPoint>);
    filtered_cloud->points.reserve(cloud->points.size());

    map.add("groundCandidates", 0.0);
    map.add("planeDist", 0.0);
    map.add("m2", 0.0);
    map.add("meanVariance", 0.0);
    // Layer to have a flag if ground or not
    map.add("isGround", 0.0);
    // raw point count layer for the evaluation
    map.add("pointsRaw", 0.0);



    map.add("boolVisualize", 0.0);
    map.add("distVisualize", 0.0);
    map.add("MaxVar", 0.0);

    // file_csv << "period" << "\n";



    map["groundCandidates"].setZero();
    map["points"].setZero();
    map["minGroundHeight"].setConstant(std::numeric_limits<float>::max());
    map["maxGroundHeight"].setConstant(std::numeric_limits<float>::min());

    map.add("variance", 0.0);
    static const grid_map::Matrix& ggv = map["variance"];
    static grid_map::Matrix& gpl = map["points"];
    static grid_map::Matrix& ggl = map["ground"];
    const auto& size = map.getSize();
    const size_t threadcount = mConfig.thread_count;


    std::vector<std::pair<size_t, grid_map::Index> > point_index;
    point_index.reserve(cloud->points.size());
    std::vector<std::vector<std::pair<size_t, grid_map::Index> > > point_index_list;
    point_index_list.resize(threadcount);

    // Collect all outliers for the outlier detection evaluation
    std::vector<size_t> outliers;
    std::vector<std::vector<size_t> > outliers_list;
    outliers_list.resize(threadcount);

    // store ignored points to re-add them afterwards
    std::vector<std::pair<size_t, grid_map::Index> > ignored;
    std::vector<std::vector<std::pair<size_t, grid_map::Index> > > ignored_list;
    ignored_list.resize(threadcount);

    // Divide the point cloud into threadcount sections for threaded calculations
    std::vector<std::thread> threads;

    for(size_t i=0; i<threadcount; ++i){
        const size_t start = std::floor((i*cloud->points.size())/threadcount);
        const size_t end = std::ceil(((i+1)*cloud->points.size())/threadcount);
        threads.push_back(std::thread(&GroundSegmentation::insert_cloud, this, cloud, start, end, std::cref(cloudOrigin), std::ref(point_index_list[i]), std::ref(ignored_list[i]),
                                      std::ref(outliers_list[i]), std::ref(map)));
    }

    // wait for results
    std::for_each(threads.begin(), threads.end(), std::mem_fn(&std::thread::join));

    // join results
    for(const auto& point_index_part : point_index_list)
        point_index.insert(point_index.end(), point_index_part.begin(), point_index_part.end());
    for(const auto& outlier_index_part : outliers_list)
        outliers.insert(outliers.end(), outlier_index_part.begin(), outlier_index_part.end());
    for(const auto& ignored_part : ignored_list)
        ignored.insert(ignored.end(), ignored_part.begin(), ignored_part.end());


    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    const double milliseconds = elapsed_seconds.count() * 1000;
    avg_insertion_time = (milliseconds + time_vals * avg_insertion_time)/(time_vals+1);
    ROS_DEBUG_STREAM("ground point rasterization took " << milliseconds << "ms (avg " << avg_insertion_time << " ms)");

    start = std::chrono::steady_clock::now();

    // Divide the grid map into four section for threaded calculations
    threads.clear();
    for(unsigned short section=0; section<4; ++section)
        threads.push_back(std::thread(&GroundSegmentation::detect_ground_patches, this, std::ref(map), section));

    // wait for results
    std::for_each(threads.begin(), threads.end(), std::mem_fn(&std::thread::join));
    end = std::chrono::steady_clock::now();
    elapsed_seconds = end-start;
    avg_detection_time = (elapsed_seconds.count() * 1000 + time_vals * avg_detection_time)/(time_vals+1);
    ROS_DEBUG_STREAM("ground patch detection took " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms (avg " << avg_detection_time << " ms)");
    ++time_vals;

    start = std::chrono::steady_clock::now();
    spiral_ground_interpolation(map, mapToBase);
    end = std::chrono::steady_clock::now();
    ROS_DEBUG_STREAM("ground interpolation took " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms");

    start = std::chrono::steady_clock::now();
    map["points"].setConstant(0.0);

    // Re-add ignored points
    point_index.insert(point_index.end(), ignored.begin(), ignored.end());


    // Debugging statistics
    const double& min_dist_fac = param_MinimumVarianceThr*5;
    const double& min_point_height_thres = param_PointHeightThrForGround;
    const double& min_point_height_obs_thres = param_PointHeightThrForObstacle;

    for(const std::pair<size_t, grid_map::Index>& entry : point_index)
    {
        const PCLPoint& point = cloud->points[entry.first];
        const grid_map::Index& gi = entry.second;
        const double& groundheight = ggl(gi(0),gi(1));

        // copy the points intensity because it get's overwritten for evaluation purposes
        const float& variance = ggv(gi(0),gi(1));

        if(size(0) <= gi(0)+3 || size(1) <= gi(1)+3)
            continue;

        const float dist = std::hypot(point.x-cloudOrigin.x, point.y-cloudOrigin.y);
        const double tolerance = std::max(std::min((min_dist_fac*dist)/variance * min_point_height_thres, min_point_height_thres), min_point_height_obs_thres);

        if(tolerance+groundheight < point.z){ // non-ground points
            PCLPoint& segmented_point = filtered_cloud->points.emplace_back(point);
            segmented_point.intensity = 99;
            gpl(gi(0),gi(1)) += 1.0f;
        }
        else{
            PCLPoint& segmented_point = filtered_cloud->points.emplace_back(point); // ground point
            segmented_point.intensity = 49;
        }
    }

    // Re-add outliers to cloud
   for(size_t i : outliers){
        const PCLPoint& point = cloud->points[i];
        PCLPoint& segmented_point = filtered_cloud->points.emplace_back(point); //ground point
        segmented_point.intensity = 49;
    }

    end = std::chrono::steady_clock::now();
    elapsed_seconds = end-start;
    avg_segmentation_time = (elapsed_seconds.count() * 1000 + (time_vals-1) * avg_segmentation_time)/time_vals;
    ROS_DEBUG_STREAM("point cloud segmentation took " << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "ms (avg " << avg_segmentation_time << " ms)");

    return filtered_cloud;
}


void GroundSegmentation::insert_cloud(const pcl::PointCloud<PCLPoint>::Ptr cloud, const size_t start, const size_t end, const PCLPoint& cloudOrigin, std::vector<std::pair<size_t, grid_map::Index> >& point_index,
                                      std::vector<std::pair<size_t, grid_map::Index> >& ignored, std::vector<size_t>& outliers, grid_map::GridMap &map)
{
    static const grid_map::Matrix& ggp = map["groundpatch"];

    static grid_map::Matrix& gpr = map["pointsRaw"];
    static grid_map::Matrix& gpl = map["points"];
    static grid_map::Matrix& ggl = map["ground"];
    static grid_map::Matrix& gmg = map["groundCandidates"];
    static grid_map::Matrix& gmm = map["meanVariance"];
    static grid_map::Matrix& gmx = map["maxGroundHeight"];
    static grid_map::Matrix& gmi = map["minGroundHeight"];
    static grid_map::Matrix& gmd = map["planeDist"];
    static grid_map::Matrix& gm2 = map["m2"];

    const auto& size = map.getSize();

    point_index.reserve(end-start);

    for(size_t i = start; i < end; ++i)
    {
        const PCLPoint& point = cloud->points[i];
        const auto& pos = grid_map::Position(point.x,point.y);
        const float sqdist = std::pow(point.x-cloudOrigin.x, 2.0) + std::pow(point.y-cloudOrigin.y, 2.0);

        bool toSkip=false;

        grid_map::Index gi;
        map.getIndex(pos, gi);

        if(!map.isInside(pos))
            continue;

        // point count map used for evaluation
        gpr(gi(0), gi(1)) += 1.0f;


        if((param_MinRing <= point.ring && point.ring <= param_MaxRing) || sqdist < param_minDistSquared){
            ignored.push_back(std::make_pair(i, gi));
            continue;
        }

        // Outlier detection test
        const float oldgroundheight = ggl(gi(0), gi(1));
        if(point.z < oldgroundheight-0.2){

            // get direction
            PCLPoint vec;
            vec.x = point.x - cloudOrigin.x;
            vec.y = point.y - cloudOrigin.y;
            vec.z = point.z - cloudOrigin.z;

            float len = std::sqrt(std::pow(vec.x, 2.0f) + std::pow(vec.y, 2.0f) + std::pow(vec.z, 2.0f));
            vec.x /= len;
            vec.y /= len;
            vec.z /= len;

            // check for occlusion
            for(int step=3; (std::pow(step*vec.x, 2.0) + std::pow(step*vec.y, 2.0) + std::pow(step*vec.z,2.0)) < std::pow(len,2.0) && vec.z < -0.01f; ++step){
                grid_map::Index intersection, pointPosIndex;
                grid_map::Position intersecPos(step*(vec.x)+cloudOrigin.x, step*(vec.y)+cloudOrigin.y);
                map.getIndex(intersecPos, intersection);

                // Check if inside map borders
                if(intersection(0) <= 0 || intersection(1) <= 0 || intersection(0) >= size(0)-1 || intersection(1) >= size(1)-1)
                    continue;

                // check if known ground occludes the line of sight
                const auto& block = ggp.block<3,3>(std::max(intersection(0)-1, 2), std::max(intersection(1)-1,2));
                if(block.sum() > mConfig.min_outlier_detection_ground_confidence && ggp(intersection(0),intersection(1)) > 0.01f && ggl(intersection(0),intersection(1)) >= step*vec.z+cloudOrigin.z+param_OutlierTolerance){
                    outliers.push_back(i);
                    toSkip=true;
                    break;
                }
            }
        }


        if(toSkip)
            continue;


        float &groundheight = gmg(gi(0),gi(1));
        float &mean = gmm(gi(0), gi(1));


        float planeDist = 0.0;
        point_index.push_back(std::make_pair(i, gi));

        float &points = gpl(gi(0), gi(1));
        float &maxHeight = gmx(gi(0),gi(1));
        float &minHeight = gmi(gi(0),gi(1));
        float &planeDistMap = gmd(gi(0),gi(1));
        float &m2 = gm2(gi(0),gi(1));

        planeDist = point.z - cloudOrigin.z;
        groundheight = (point.z + points * groundheight)/(points+1.0);

        if(mean == 0.0)
            mean = planeDist;
        if(!std::isnan(planeDist)){
            float delta = planeDist - mean;
            mean += delta/(points+1);
            planeDistMap = (planeDist + points * planeDistMap)/(points+1.0);
            m2 += delta * (planeDist - mean);
        }

        maxHeight = std::max(maxHeight, point.z);
        minHeight =std::min(minHeight, point.z-0.0001f); // to make sure maxHeight > minHeight
        points += 1.0;
    }
}


void GroundSegmentation::detect_ground_patches(grid_map::GridMap &map, unsigned short section) const
{
    const grid_map::Matrix& gcl = map["groundCandidates"];
    const static auto& size = map.getSize();
    const static float resolution = map.getResolution();
    static const grid_map::Matrix& gm2 = map["m2"];
    static const grid_map::Matrix& gpl = map["points"];
    static grid_map::Matrix& ggv = map["variance"];
    // calculate variance
    ggv = gm2.array().cwiseQuotient(gpl.array()+std::numeric_limits<float>::min());

    int patch_radius = std::floor(std::max(param_PatchSizeSmall, param_PatchSizeBig)/2);

    int cols_start = patch_radius + section % 2 * (gcl.cols() / 2 - patch_radius);
    int rows_start = section >= 2 ? gcl.rows() / 2 : patch_radius;
    int cols_end   = gcl.cols() / 2 + section % 2 * (gcl.cols() / 2 - patch_radius);
    int rows_end   = section >= 2 ? gcl.rows() - patch_radius : gcl.rows() / 2;

    for(int i=cols_start; i<cols_end; ++i){
        for(int j=rows_start; j<rows_end; ++j){
            const float sqdist = (std::pow(i-(size(0)/2.0),2.0) + std::pow(j-(size(1)/2.0), 2.0)) * std::pow(resolution,2.0);

            if(sqdist <= std::pow(param_DistanceChangePatchSize, 2.0))
                detect_ground_patch<3>(map, i, j);
            else
                detect_ground_patch<5>(map, i, j);
        }
    }
}


template <int S> void GroundSegmentation::detect_ground_patch(grid_map::GridMap& map, size_t i, size_t j) const
{
    static grid_map::Matrix& ggl = map["ground"];
    static grid_map::Matrix& ggp = map["groundpatch"];
    static grid_map::Matrix& ggv = map["variance"];
    static grid_map::Matrix& map_isGround = map["isGround"];
    float& value_isGround = map_isGround(i, j);
    static const grid_map::Matrix& gmi = map["minGroundHeight"];
    static const grid_map::Matrix& gpl = map["points"];
    static const auto& size = map.getSize();
    static const float resolution = map.getResolution();
    const int center_idx = std::floor(S/2);




    static grid_map::Matrix& g_average = map["groundCandidates"];




    float& visual_bool = map["boolVisualize"](i, j);
    static grid_map::Matrix& distance_map = map["distVisualize"];
    static const grid_map::Matrix& gplr = map["pointsRaw"];





    const auto& pointsBlock = gpl.block<S,S>(i-center_idx,j-center_idx);
    const auto& pointsRawBlock = gplr.block<S,S>(i-center_idx,j-center_idx);
    const float sqdist = (std::pow(i-(size(0)/2.0),2.0) + std::pow(j-(size(1)/2.0), 2.0)) * std::pow(resolution,2.0);
    const float& expectedPointCountperLaserperCell = expectedPoints(i,j);
    const float& pointsblockSum = pointsBlock.sum();
    float& oldConfidence = ggp(i,j);
    float& oldGroundheight = ggl(i,j);

    // early skipping of (almost) empty areas
    if (pointsRawBlock.sum() < param_PatchPercentageThrForFiltering * expectedPoints.block<S,S>(i-center_idx,j-center_idx).sum())
        return;

    // calculation of variance threshold
    // limit the value to the defined minimum and 10 times the defined minimum
    const float varThresholdsq = std::min(
                                    std::pow(sqdist, param_PowerDistForVariance) + param_OffsetCoefficientForVariance, 
                                    param_MaximumVarianceThr
                                );
    const auto& varblock = ggv.block<S,S>(i-center_idx,j-center_idx);
    const auto& minblock = gmi.block<S,S>(i-center_idx, j-center_idx);
    // const float& variance = varblock(center_idx,center_idx);
    const float& localmin = minblock.minCoeff();
    const float maxVar = pointsBlock(center_idx,center_idx) >= param_PointPerCellThresholdForVariance ?
                            varblock(center_idx,center_idx) : pointsBlock.cwiseProduct(varblock).sum()/pointsblockSum;

    visual_bool = (center_idx,center_idx) >= param_PointPerCellThresholdForVariance ? 1.0 : 0.0;




    // // Ground computation method: directly num_points average
    // const float groundlevel = pointsBlock.cwiseProduct(minblock).sum()/pointsblockSum;

    // Ground computation method: either value of the block, or num_points average
    const float groundlevel = pointsBlock(center_idx,center_idx) >= param_PointPerCellThresholdForVariance ?
                            minblock(center_idx,center_idx) : pointsBlock.cwiseProduct(minblock).sum()/pointsblockSum;

    // // Ground computation method: new based on confidence (very bad)
    // const auto& confidenceblock = ggp.block<S,S>(i-center_idx, j-center_idx);
    // const float groundlevel = confidenceblock.cwiseProduct(minblock).sum()/confidenceblock.sum();

    // // Ground computation method: average but NOT weighted
    // const float groundlevel = minblock.sum()/(S*S);




    // const float groundDiff = std::max((groundlevel - oldGroundheight) * (2.0f*oldConfidence), 1.0f);

    // // Do not update known high confidence estimations upward
    // if(oldConfidence > param_OldConfidenceThreshold && groundlevel >= oldGroundheight + param_EstimationUpwardTolerance)
    //     return;

    if(varThresholdsq > maxVar && maxVar > 0) {
            float newConfidence = std::min(pointsblockSum/param_ConfidenceDecreaseFactorPointNumber, 1.0);
            // calculate ground height
            oldGroundheight = (groundlevel*newConfidence + oldConfidence*oldGroundheight*param_OldMemory)/(newConfidence+oldConfidence*param_OldMemory);
            // update confidence
            oldConfidence = std::min((newConfidence + oldConfidence*param_OldMemory)/(1.0+param_OldMemory), 1.0);
            // signal that the cell is ground
            value_isGround = 1.0;
    }
    else if(localmin < oldGroundheight){
        // update ground height
        oldGroundheight = localmin;
        // update confidence
        oldConfidence = std::min(oldConfidence + 0.1f, 0.5f);
    }
}


void GroundSegmentation::spiral_ground_interpolation(grid_map::GridMap &map, const geometry_msgs::TransformStamped &toBase) const
{
    static grid_map::Matrix& ggl = map["ground"];
    static grid_map::Matrix& gvl = map["groundpatch"];
    const auto& map_size = map.getSize();
    const auto& center_idx = map_size(0)/2-1;

    gvl(center_idx,center_idx) = 1.0f;
    geometry_msgs::PointStamped ps;
    ps.header.frame_id = "base_link";
    tf2::doTransform(ps,ps,toBase);

    // Set center to current vehicle height
    ggl(center_idx,center_idx) = ps.point.z;






    static grid_map::Matrix& visual_map = map["boolVisualize"];
    static grid_map::Matrix& distance_map = map["distVisualize"];
    static grid_map::Matrix& variance_map = map["variance"];
    static const grid_map::Matrix& gpl = map["points"];
    static grid_map::Matrix& map_MaxVar = map["MaxVar"];






    if (!file_csv.is_open()) {
        return;
    }






    // // Extract robot pose from transform
    // tf2::Vector3 base_pos(toBase.transform.translation.x,
    //                         toBase.transform.translation.y,
    //                         toBase.transform.translation.z);
    // tf2::Quaternion base_q;
    // tf2::fromMsg(toBase.transform.rotation, base_q);
    // double roll, pitch, yaw;
    // tf2::Matrix3x3(base_q).getRPY(roll, pitch, yaw);
    // float cos_yaw = std::cos(yaw);
    // float sin_yaw = std::sin(yaw);

    // // Define the corners of the rectangular box (in vehicle frame)
    // std::vector<tf2::Vector3> corners_vehicle = {
    //     tf2::Vector3(parameter_ZeroPaddingFront,  parameter_ZeroPaddingLeft,  0),
    //     tf2::Vector3(parameter_ZeroPaddingFront, -parameter_ZeroPaddingRight, 0),
    //     tf2::Vector3(-parameter_ZeroPaddingBehind, -parameter_ZeroPaddingRight, 0),
    //     tf2::Vector3(-parameter_ZeroPaddingBehind,  parameter_ZeroPaddingLeft,  0)
    // };

    // // Transform corners to map frame and get bounding box
    // float min_x = std::numeric_limits<float>::infinity();
    // float max_x = -min_x;
    // float min_y = min_x;
    // float max_y = -min_x;

    // for (const auto& corner : corners_vehicle) {
    //     float mx = base_pos.x() + cos_yaw * corner.x() - sin_yaw * corner.y();
    //     float my = base_pos.y() + sin_yaw * corner.x() + cos_yaw * corner.y();
    //     min_x = std::min(min_x, mx);
    //     max_x = std::max(max_x, mx);
    //     min_y = std::min(min_y, my);
    //     max_y = std::max(max_y, my);
    // }

    // // Loop through bounding box with 0.2m step
    // for (float x = min_x; x <= max_x; x += 0.2f) {
    //     for (float y = min_y; y <= max_y; y += 0.2f) {
    //         grid_map::Position pos(x, y);
    //         grid_map::Index idx;
    //         if (!map.isInside(pos) || !map.getIndex(pos, idx)) continue;

    //         ggl(idx(0), idx(1)) = ps.point.z;
    //         gvl(idx(0), idx(1)) = 1.0f;
    //     }
    // }






    // // Extract robot pose from transform
    // tf2::Vector3 base_pos = tf2::Vector3(toBase.transform.translation.x,
    //                                      toBase.transform.translation.y,
    //                                      toBase.transform.translation.z);
    // tf2::Quaternion base_q;
    // tf2::fromMsg(toBase.transform.rotation, base_q);
    // double roll, pitch, yaw;
    // tf2::Matrix3x3(base_q).getRPY(roll, pitch, yaw);
    // // Precompute sin/cos for rotation
    // float cos_yaw = std::cos(yaw);
    // float sin_yaw = std::sin(yaw);
    // // Define step and boundaries
    // float step = 0.15;
    // float lower_limit = -1.0;
    // float upper_limit = 60.0;
    // float x = 0;

    // while ((lower_limit<=x)&&(x<=upper_limit)) {

    //     // Rotate and translate into map frame
    //         // Only x
    //     float map_x = base_pos.x() + cos_yaw * x;
    //     float map_y = base_pos.y() + sin_yaw * x;
    //         // Oblique with y
    //     // float map_x = base_pos.x() + cos_yaw * x - sin_yaw * x;
    //     // float map_y = base_pos.y() + sin_yaw * x + cos_yaw * x;
    //         // Increase the step
    //     x += step;

    //     // Get index from position
    //     grid_map::Position pos(map_x, map_y);
    //     grid_map::Index idx;
    //     if (!map.isInside(pos)) {
    //         break;
    //     } else {
    //         if (!map.getIndex(pos, idx)) {
    //             continue;
    //         }
    //     }

    //     // Compute distance and variance
    //     const float distance = std::sqrt(std::pow((map_x-base_pos.x()), 2.0) + std::pow((map_y-base_pos.y()), 2.0));
    //     const float variance_1 = variance_map(idx(0), idx(1));

    //     // Get the maxvar
    //     float maxVar = map_MaxVar(idx(0), idx(1));

    //     // Intialize variances
    //     float variance_3 = -1.0;
    //     float variance_5 = -1.0;
    //     // Initialize rows and columns
    //     const int rows = variance_map.rows();
    //     const int cols = variance_map.cols();

    //     // Compute variance_3 only if inside bounds
    //     if (idx(0) >= 1 && idx(0) + 1 < rows &&
    //         idx(1) >= 1 && idx(1) + 1 < cols) {
    //         const auto block_3 = variance_map.block<3, 3>(idx(0) - 1, idx(1) - 1);
    //         const auto& pointsBlock_3 = gpl.block<3, 3>(idx(0) - 1, idx(1) - 1);
    //         variance_3 = pointsBlock_3.array().cwiseProduct(block_3.array()).sum() / pointsBlock_3.sum();
    //     }

    //     // Compute variance_5 only if inside bounds
    //     if (idx(0) >= 2 && idx(0) + 2 < rows &&
    //         idx(1) >= 2 && idx(1) + 2 < cols) {
    //         const auto& block_5 = variance_map.block<5, 5>(idx(0) - 2, idx(1) - 2);
    //         const auto& pointsBlock_5 = gpl.block<5, 5>(idx(0) - 2, idx(1) - 2);
    //         variance_5 = pointsBlock_5.array().cwiseProduct(block_5.array()).sum() / pointsBlock_5.sum();
    //     }

    //     visual_map(idx(0), idx(1)) = 1.0;
    //     // distance_map(idx(0), idx(1)) = distance;

    //     // Write a single line to the CSV
    //     file_csv << distance << "," << variance_1 << "," << variance_3 << "," << variance_5 << "," << maxVar << "\n";
    // }






    for(int i=center_idx-1; i>=static_cast<int>(param_BlockSizeInterpolation/2); --i) {
        // rectangle_pos = x,y position of rectangle top left corner
        int rectangle_pos = i;

        // rectangle side length
        int side_length = (center_idx-rectangle_pos)*2;

        // top and left side
        for(short side=0; side<2; ++side){
            for(int pos=rectangle_pos; pos<rectangle_pos+side_length; ++pos){
                const int x = side%2 ? pos : rectangle_pos;
                const int y = side%2 ? rectangle_pos : pos;

                interpolate_cell(map, x, y);
            }
        }

        // bottom and right side
        rectangle_pos += side_length;
        for(short side=0; side<2; ++side){
            for(int pos=rectangle_pos; pos>=rectangle_pos-side_length; --pos){
                int x = side%2 ? pos : rectangle_pos;
                int y = side%2 ? rectangle_pos : pos;

                interpolate_cell(map, x, y);
            }
        }
    }
}



void GroundSegmentation::interpolate_cell(grid_map::GridMap &map, const size_t x, const size_t y) const
{
    // // "isGround" says if it has been identified as ground or not
    // static grid_map::Matrix& map_isGround = map["isGround"];
    // if (map_isGround(x,y) > 0.5)
    //     return;

    static const auto& center_idx = map.getSize()(0)/2-1;
    // "groundpatch" layer contains confidence values
    static grid_map::Matrix& gvl = map["groundpatch"];
    const auto& gvlblock = gvl.block<param_BlockSizeInterpolation,param_BlockSizeInterpolation>(x-param_BlockSizeInterpolation/2,y-param_BlockSizeInterpolation/2);
    // "ground" contains the ground height values
    static grid_map::Matrix& ggl = map["ground"];

    float& height = ggl(x,y);
    float& confidence = gvl(x,y);
    const float& gvlSum = gvlblock.sum() + std::numeric_limits<float>::min(); // avoid a possible div by 0
    const float avg = (gvlblock.cwiseProduct
                        (ggl.block<param_BlockSizeInterpolation,param_BlockSizeInterpolation>
                        (x-param_BlockSizeInterpolation/2, y-param_BlockSizeInterpolation/2))
                      ).sum()/gvlSum;

    height = (1.0f-confidence) * avg + confidence * height;

    // Only update confidence in cells above min distance
    if ((std::pow((float)x-center_idx, 2.0) + std::pow((float)y-center_idx, 2.0)) * std::pow(map.getResolution(), 2.0f) > param_minDistSquared) {
        confidence = std::max(confidence*param_ConfidenceDecreaseFactorInterpolation, 0.001);
    }
}


void GroundSegmentation::setConfig(const groundgrid::GroundGridConfig &config)
{
    mConfig = config;
}
