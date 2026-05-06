#ifndef PLANE_SEGMENTATION_CLASS_H
#define PLANE_SEGMENTATION_CLASS_H

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <deque>

/**
 * @brief PlaneSegmentation class - segments door surface and detects handle
 * via hole detection. Publishes door cloud and synthetic handle centroid.
 */
class PlaneSegmentation
{
public:
  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef PointCloud::Ptr CloudPtr;

public:
  PlaneSegmentation(const std::string &pointcloud_topic = "",
                    const std::string &base_frame = "base_link");
  ~PlaneSegmentation();

  bool initalize(ros::NodeHandle &nh);
  void update(const ros::Time &time);

private:
  bool preProcessCloud(CloudPtr& input, CloudPtr& output);
  bool segmentCloud(CloudPtr& input, CloudPtr& hole_cloud, geometry_msgs::Point& handle_midpoint, bool& has_handle);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

  // Centroid filtering - returns smoothed centroid, applies jump threshold
  geometry_msgs::Point filterCentroid(const geometry_msgs::Point& raw_centroid);

private:
  bool is_cloud_updated_;
  std::string base_frame_;
  std::string pointcloud_topic_;

  ros::Subscriber point_cloud_sub_;

  // Publishers
  ros::Publisher hole_cloud_pub_;
  ros::Publisher debug_cloud_pub_;
  ros::Publisher handle_synthetic_centroid_pub_;

  // Internal pointclouds
  CloudPtr raw_cloud_;
  CloudPtr preprocessed_cloud_;
  CloudPtr hole_cloud_; 

  ros::Time last_msg_stamp_;
  tf::TransformListener tfListener_;

  // Centroid smoothing buffer
  std::deque<geometry_msgs::Point> centroid_buffer_;
  static const size_t CENTROID_BUFFER_SIZE = 10;
  static constexpr double CENTROID_JUMP_THRESHOLD = 0.10; // 10cm jump detection
  static const int MAX_CONSECUTIVE_JUMPS = 3; // Block update after 3 jumps in a row
  geometry_msgs::Point last_valid_centroid_;
  bool has_valid_centroid_;
  int consecutive_jump_count_;

  // Timer for continuous publishing
  ros::Timer publish_timer_;
  void publishTimerCallback(const ros::TimerEvent& ev);
  geometry_msgs::Point current_smoothed_centroid_;
  bool centroid_ready_;
};

#endif
