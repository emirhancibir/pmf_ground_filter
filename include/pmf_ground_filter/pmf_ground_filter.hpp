#ifndef PMF_GROUND_FILTER_HPP
#define PMF_GROUND_FILTER_HPP

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <unordered_set>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

namespace sick_pcl
{
struct PointXYZIR
{
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
} EIGEN_ALIGN16;

};

// reg pcl
POINT_CLOUD_REGISTER_POINT_STRUCT (sick_pcl::PointXYZIR,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (std::uint16_t, ring, ring)
)

class PMFGroundFilter
{
public:
    PMFGroundFilter(ros::NodeHandle& nh);

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);

    ros::NodeHandle nh_;
    ros::Subscriber sub_point_cloud_;
    ros::Publisher pub_non_ground_;
    ros::Publisher pub_ground_;


    int max_window_size_;
    float slope_, initial_distance_, max_distance_;
};

#endif // PMF_GROUND_FILTER_HPP
