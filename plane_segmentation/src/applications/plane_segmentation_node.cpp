#include <plane_segmentation/plane_segmentation.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plane_segmentation_node");
  
  ros::NodeHandle global_nh;
  ros::NodeHandle private_nh("~");

  std::string pointcloud_topic_name;
  std::string base_fame_name;

  // Get parameters using the *private* handle
  if (!private_nh.getParam("point_cloud_topic", pointcloud_topic_name))
  {
    ROS_ERROR("Failed to get 'point_cloud_topic' parameter from server.");
    return -1;
  }

  if (!private_nh.getParam("base_frame", base_fame_name))
  {
    ROS_ERROR("Failed to get 'base_frame' parameter from server.");
    return -1;
  }

  ROS_INFO("Parameters loaded:");
  ROS_INFO("point_cloud_topic: %s", pointcloud_topic_name.c_str());
  ROS_INFO("base_frame: %s", base_fame_name.c_str());

  // Construct the object with the *loaded* parameters
  PlaneSegmentation segmentation(
    pointcloud_topic_name, 
    base_fame_name);
  

  if(!segmentation.initalize(global_nh))
  {
    ROS_ERROR_STREAM("Error init PlaneSegmentation");
    return -1;
  }

  // Run
  ros::Rate rate(10); // 10 Hz
  while(ros::ok())
  {
    segmentation.update(ros::Time::now());
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}