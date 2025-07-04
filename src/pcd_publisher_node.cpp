#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <velodyne_pointcloud/point_types.h>  // ⬅️ Include correct point type

#include <boost/filesystem.hpp>
#include <vector>
#include <string>
#include <algorithm>

namespace fs = boost::filesystem;

std::vector<std::string> getPcdFiles(const std::string& folder_path) {
    std::vector<std::string> files;
    if (!fs::exists(folder_path) || !fs::is_directory(folder_path)) {
        ROS_ERROR("Invalid folder path: %s", folder_path.c_str());
        return files;
    }

    for (const auto& entry : fs::directory_iterator(folder_path)) {
        if (entry.path().extension() == ".pcd") {
            files.push_back(entry.path().string());
        }
    }

    std::sort(files.begin(), files.end());
    return files;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd_publisher_node");
    ros::NodeHandle nh("~");

    std::string folder;
    double rate;
    nh.param<std::string>("folder", folder, "/root/catkin_ws/src/bags/PCDs_ground_testing/");
    nh.param<double>("rate", rate, 1.0);

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/pcd_points", 1);

    std::vector<std::string> pcd_files = getPcdFiles(folder);
    size_t index = 0;
    ros::Rate loop(rate);

    while (ros::ok() && index < pcd_files.size()) {
        pcl::PointCloud<velodyne_pointcloud::PointXYZIR> cloud;  // ⬅️ Use correct point type
        if (pcl::io::loadPCDFile<velodyne_pointcloud::PointXYZIR>(pcd_files[index], cloud) == -1) {
            ROS_WARN("Failed to load %s", pcd_files[index].c_str());
            ++index;
            continue;
        }

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header.frame_id = "base_link";
        output.header.stamp = ros::Time::now();

        pub.publish(output);
        ROS_INFO("Published %s", pcd_files[index].c_str());

        ++index;
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
