#include "pmf_ground_filter/pmf_ground_filter.hpp"

PMFGroundFilter::PMFGroundFilter(ros::NodeHandle& nh)
{
  // get param 
  nh.param("max_window_size", max_window_size_, 2);
  nh.param("slope", slope_, 1.0f);
  nh.param("initial_distance", initial_distance_, 0.4f);
  nh.param("max_distance", max_distance_, 1.5f);

  std::string input_topic;
  nh.param("input_topic", input_topic, std::string("/cloud_all_fields_fullframe"));
  std::string non_ground_topic;
  nh.param("non_ground_topic", non_ground_topic, std::string("/non_ground_points"));
  std::string ground_topic;
  nh.param("ground_topic", ground_topic, std::string("/ground_points"));

  sub_point_cloud_ = nh_.subscribe(input_topic, 1, &PMFGroundFilter::pointCloudCallback, this);
  pub_non_ground_ = nh_.advertise<sensor_msgs::PointCloud2>(non_ground_topic, 10);
  pub_ground_ = nh_.advertise<sensor_msgs::PointCloud2>(ground_topic, 100);

  // Print params
  ROS_INFO("Parameters loaded:");
  ROS_INFO(" - max_window_size: %d", max_window_size_);
  ROS_INFO(" - slope: %f", slope_);
  ROS_INFO(" - initial_distance: %f", initial_distance_);
  ROS_INFO(" - max_distance: %f", max_distance_);
  ROS_INFO(" - input_topic: %s", input_topic.c_str());
  ROS_INFO(" - non_ground_topic: %s", non_ground_topic.c_str());
  ROS_INFO(" - ground_topic: %s", ground_topic.c_str());
}

void PMFGroundFilter::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{

  ROS_INFO("Input timestamp: %f", input->header.stamp.toSec());

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<sick_pcl::PointXYZIR>::Ptr cloud_r(new pcl::PointCloud<sick_pcl::PointXYZIR>);

  pcl::fromROSMsg(*input, *cloud_r); // Read into custom point cloud
  pcl::fromROSMsg(*input, *cloud); // Read into standard point cloud

  if (cloud->empty()) {
    ROS_WARN("Received an empty point cloud.");
    return;
  }

  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInputCloud(cloud);
  pmf.setMaxWindowSize(max_window_size_);
  pmf.setSlope(slope_);
  pmf.setInitialDistance(initial_distance_);
  pmf.setMaxDistance(max_distance_);

  pcl::PointIndicesPtr ground(new pcl::PointIndices);
  pmf.extract(ground->indices);

  // Create new point clouds for ground and non-ground points with custom point type
  pcl::PointCloud<sick_pcl::PointXYZIR>::Ptr ground_cloud(new pcl::PointCloud<sick_pcl::PointXYZIR>);
  pcl::PointCloud<sick_pcl::PointXYZIR>::Ptr non_ground_cloud(new pcl::PointCloud<sick_pcl::PointXYZIR>);
  
  // Set for quick access
  std::unordered_set<int> ground_indices(ground->indices.begin(), ground->indices.end());

  // Loop through indices and populate new point clouds
  for (size_t i = 0; i < cloud_r->points.size(); ++i) {
    if (ground_indices.find(static_cast<int>(i)) != ground_indices.end()) {
      ground_cloud->points.push_back(cloud_r->points[i]);
    } else {
      non_ground_cloud->points.push_back(cloud_r->points[i]);
    }
  }

  // Publish non-ground points
  sensor_msgs::PointCloud2 non_ground_msg;
  pcl::toROSMsg(*non_ground_cloud, non_ground_msg);
  non_ground_msg.header = input->header;
  pub_non_ground_.publish(non_ground_msg);

  // Publish ground points
  sensor_msgs::PointCloud2 ground_msg;
  pcl::toROSMsg(*ground_cloud, ground_msg);
  ground_msg.header = input->header;
  pub_ground_.publish(ground_msg);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pmf_ground_filter");
  ros::NodeHandle nh("~");
  PMFGroundFilter filter(nh); 

  ros::Rate loop_rate(20); // 10 Hz frekans

  while (ros::ok())
  {
      ros::spinOnce(); // Callback'leri çağırır
      loop_rate.sleep(); // Hedef frekansta bekler
  }

  return 0;
}
