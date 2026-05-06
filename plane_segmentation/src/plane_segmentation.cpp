#include <plane_segmentation/plane_segmentation.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/centroid.h>

#include <cstring>
#include <queue>
#include <cmath>

PlaneSegmentation::PlaneSegmentation(
    const std::string& pointcloud_topic, 
    const std::string& base_frame) :
  pointcloud_topic_(pointcloud_topic),
  base_frame_(base_frame),
  is_cloud_updated_(false),
  has_valid_centroid_(false),
  centroid_ready_(false),
  consecutive_jump_count_(0),
  tfListener_(ros::Duration(10.0))
{
}

PlaneSegmentation::~PlaneSegmentation()
{
}

bool PlaneSegmentation::initalize(ros::NodeHandle& nh)
{
  if (pointcloud_topic_.empty())
  {
    nh.param<std::string>("pointcloud_topic", pointcloud_topic_, "/throttle_filtering_points/filtered_points");
    if (pointcloud_topic_.empty())
      pointcloud_topic_ = "/throttle_filtering_points/filtered_points";
  }

  point_cloud_sub_ = nh.subscribe(pointcloud_topic_, 1, &PlaneSegmentation::cloudCallback, this);
  ROS_INFO("Subscribed to: %s", pointcloud_topic_.c_str());

  hole_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("hole_cloud", 1);
  debug_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("debug_cloud", 1);
  handle_synthetic_centroid_pub_ = nh.advertise<geometry_msgs::PointStamped>("handle_synthetic_centroid", 1);

  publish_timer_ = nh.createTimer(ros::Duration(0.1), &PlaneSegmentation::publishTimerCallback, this);

  raw_cloud_.reset(new PointCloud());
  preprocessed_cloud_.reset(new PointCloud());
  hole_cloud_.reset(new PointCloud());

  ROS_INFO("PlaneSegmentation initialized");
  return true;
}

void PlaneSegmentation::publishTimerCallback(const ros::TimerEvent& ev)
{
  if (centroid_ready_)
  {
    geometry_msgs::PointStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = base_frame_;

    // Kopiere die Werte und erhöhe Z um 1.0 (Meter)
    msg.point.x = current_smoothed_centroid_.x;
    msg.point.y = current_smoothed_centroid_.y + 0.02;
    msg.point.z = current_smoothed_centroid_.z + 0.01; // Hier passiert der Versatz

    handle_synthetic_centroid_pub_.publish(msg);
  }
}

geometry_msgs::Point PlaneSegmentation::filterCentroid(const geometry_msgs::Point& raw)
{
  // Debug: always print raw input
  ROS_INFO("RAW centroid: [%.3f, %.3f, %.3f]", raw.x, raw.y, raw.z);
  
  // Add to buffer (always, regardless of jumps)
  centroid_buffer_.push_back(raw);
  if (centroid_buffer_.size() > CENTROID_BUFFER_SIZE)
    centroid_buffer_.pop_front();

  // Compute moving average of buffer
  geometry_msgs::Point smoothed;
  smoothed.x = 0; smoothed.y = 0; smoothed.z = 0;
  
  for (const auto& pt : centroid_buffer_)
  {
    smoothed.x += pt.x;
    smoothed.y += pt.y;
    smoothed.z += pt.z;
  }
  
  double n = static_cast<double>(centroid_buffer_.size());
  smoothed.x /= n;
  smoothed.y /= n;
  smoothed.z /= n;

  ROS_INFO("SMOOTHED centroid (buffer size %zu): [%.3f, %.3f, %.3f]", 
           centroid_buffer_.size(), smoothed.x, smoothed.y, smoothed.z);

  // Check if smoothed value jumped too much from last published value
  if (has_valid_centroid_)
  {
    double dx = smoothed.x - last_valid_centroid_.x;
    double dy = smoothed.y - last_valid_centroid_.y;
    double dz = smoothed.z - last_valid_centroid_.z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    if (dist > CENTROID_JUMP_THRESHOLD)
    {
      consecutive_jump_count_++;
      
      // After enough consistent "jumps", accept the new position
      if (consecutive_jump_count_ >= MAX_CONSECUTIVE_JUMPS)
      {
        ROS_WARN("Position changed! Accepting new centroid after %d consistent readings (dist=%.3fm)", 
                 consecutive_jump_count_, dist);
        consecutive_jump_count_ = 0;
        // Fall through to accept new position
      }
      else
      {
        ROS_INFO("Jump %d/%d detected (%.3fm), waiting for consistency...", 
                 consecutive_jump_count_, MAX_CONSECUTIVE_JUMPS, dist);
        ROS_INFO("PUBLISHED centroid (keeping old): [%.3f, %.3f, %.3f]", 
                 last_valid_centroid_.x, last_valid_centroid_.y, last_valid_centroid_.z);
        return last_valid_centroid_;
      }
    }
    else
    {
      // No jump - reset counter
      consecutive_jump_count_ = 0;
    }
  }

  last_valid_centroid_ = smoothed;
  has_valid_centroid_ = true;
  
  ROS_INFO("PUBLISHED centroid: [%.3f, %.3f, %.3f]", smoothed.x, smoothed.y, smoothed.z);
  return smoothed;
}

void PlaneSegmentation::update(const ros::Time& time)
{
  if (!is_cloud_updated_)
    return;
    
  is_cloud_updated_ = false;

  if (raw_cloud_->points.empty())
    return;

  if (!preProcessCloud(raw_cloud_, preprocessed_cloud_))
    return;

  geometry_msgs::Point handle_midpoint;
  bool has_handle = false;

  if (!segmentCloud(preprocessed_cloud_, hole_cloud_, handle_midpoint, has_handle))
    return;

  // If handle found, publish hole cloud and centroid
  if (has_handle)
  {
    sensor_msgs::PointCloud2 hole_msg;
    pcl::toROSMsg(*hole_cloud_, hole_msg);
    hole_msg.header.stamp = last_msg_stamp_;
    hole_msg.header.frame_id = base_frame_;
    hole_cloud_pub_.publish(hole_msg);

    // Apply filtering
    current_smoothed_centroid_ = filterCentroid(handle_midpoint);
    centroid_ready_ = true;
  }
}

bool PlaneSegmentation::preProcessCloud(CloudPtr& input, CloudPtr& output)
{
  CloudPtr ds_cloud(new PointCloud());
  
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(input);
  float leaf = 0.01f;  
  sor.setLeafSize(leaf, leaf, leaf);
  sor.filter(*ds_cloud);
  
  if (ds_cloud->points.empty())
    return false;

  // Transform to base_frame
  CloudPtr transf_cloud(new PointCloud());
  if (!tfListener_.waitForTransform(base_frame_, ds_cloud->header.frame_id, ros::Time(0), ros::Duration(1.0)))
  {
    ROS_ERROR("Transform failed: %s -> %s", base_frame_.c_str(), ds_cloud->header.frame_id.c_str());
    return false;
  }
  pcl_ros::transformPointCloud(base_frame_, ros::Time(0), *ds_cloud, ds_cloud->header.frame_id, *transf_cloud, tfListener_);
  transf_cloud->header.frame_id = base_frame_;

  // Z-filter (handle height range)
  CloudPtr z_filtered(new PointCloud());
  pcl::PassThrough<PointT> pass_z;
  pass_z.setInputCloud(transf_cloud);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(0.5, 1.5);
  pass_z.filter(*z_filtered);

  // Y-filter
  pcl::PassThrough<PointT> pass_y;
  pass_y.setInputCloud(z_filtered);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(-2.0, 2.0);
  pass_y.filter(*output);

  // Publish debug cloud
  sensor_msgs::PointCloud2 debug_msg;
  pcl::toROSMsg(*output, debug_msg);
  debug_msg.header.frame_id = base_frame_;
  debug_msg.header.stamp = last_msg_stamp_;
  debug_cloud_pub_.publish(debug_msg);

  return !output->points.empty();
}

bool PlaneSegmentation::segmentCloud(CloudPtr& input, CloudPtr& hole_cloud, geometry_msgs::Point& handle_midpoint, bool& has_handle)
{
  
  has_handle = false;
  hole_cloud->points.clear();
  if (input->points.empty())
    return false;
  
  const float HANDLE_Z_MIN = 1.02f;
  const float HANDLE_Z_MAX = 1.07f;
  const float RESOLUTION = 0.01f;
  
  CloudPtr remaining_cloud(new PointCloud(*input));
  
  const int MAX_PLANES = 5;
  const int MIN_PLANE_POINTS = 500;
  
  for (int plane_iter = 0; plane_iter < MAX_PLANES && remaining_cloud->points.size() > MIN_PLANE_POINTS; ++plane_iter)
  {
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);
    seg.setInputCloud(remaining_cloud);
    seg.segment(*inliers, *coefficients);
    
    if (inliers->indices.size() < MIN_PLANE_POINTS)
      break;
    

    if (coefficients->values.size() >= 4)
    {
      float nx = coefficients->values[0];
      float ny = coefficients->values[1];
      float nz = coefficients->values[2];
      float d  = coefficients->values[3]; // d = -normal . point_on_plane
      
      if (std::abs(nx) < 0.7f) // Skip non-frontal planes
      {
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(remaining_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        CloudPtr temp(new PointCloud());
        extract.filter(*temp);
        remaining_cloud = temp;
        continue;
      }


      
      CloudPtr floating_points(new PointCloud());
      PointT min_pt, max_pt;
      pcl::getMinMax3D(*remaining_cloud, min_pt, max_pt); // Get bounds of the scene

      for (const auto& pt : input->points)
      {
        float dist = nx * pt.x + ny * pt.y + nz * pt.z + d;
        

        
        if (std::abs(dist) > 0.05 && std::abs(dist) < 0.15)
        {
          if (pt.z >= 0.5 && pt.z <= 1.5) 
          {
            floating_points->points.push_back(pt);
          }
        }
      }
      
      if (floating_points->points.empty()) 
      {
         // Next plane
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(remaining_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        CloudPtr temp(new PointCloud());
        extract.filter(*temp);
        remaining_cloud = temp;
        continue;
      }
      
      pcl::getMinMax3D(*floating_points, min_pt, max_pt);
      
      int grid_y = static_cast<int>((max_pt.y - min_pt.y) / RESOLUTION) + 1;
      int grid_z = static_cast<int>((max_pt.z - min_pt.z) / RESOLUTION) + 1;
      
      if (grid_y <= 0 || grid_z <= 0 || grid_y > 500 || grid_z > 500) continue;
      
      // Grid stores INDICES of points falling into that cell
      std::vector<std::vector<std::vector<int>>> grid(grid_y, std::vector<std::vector<int>>(grid_z));
      std::vector<std::vector<bool>> occupied(grid_y, std::vector<bool>(grid_z, false));
      
      for (size_t i = 0; i < floating_points->points.size(); ++i)
      {
        const auto& pt = floating_points->points[i];
        int iy = static_cast<int>((pt.y - min_pt.y) / RESOLUTION);
        int iz = static_cast<int>((pt.z - min_pt.z) / RESOLUTION);
        if (iy >= 0 && iy < grid_y && iz >= 0 && iz < grid_z)
        {
          grid[iy][iz].push_back(i);
          occupied[iy][iz] = true;
        }
      }
      
      // Despeckle: Remove small isolated noise (pixels with < 2 neighbors)
      std::vector<std::vector<bool>> clean_occupied = occupied;
      for (int iy = 1; iy < grid_y - 1; ++iy)
      {
        for (int iz = 1; iz < grid_z - 1; ++iz)
        {
          if (occupied[iy][iz])
          {
            int neighbors = 0;
            for (int dy = -1; dy <= 1; ++dy)
            {
              for (int dz = -1; dz <= 1; ++dz)
              {
                if (dy == 0 && dz == 0) continue;
                if (occupied[iy + dy][iz + dz]) neighbors++;
              }
            }
            if (neighbors < 2) // Less aggressive than hole filling, just remove floaters
              clean_occupied[iy][iz] = false;
          }
        }
      }
      occupied = clean_occupied;
      
      // Cluster (Flood Fill) - looking for OCCUPIED regions (the handle itself)
      std::vector<std::vector<bool>> visited(grid_y, std::vector<bool>(grid_z, false));
      
      for (int iy = 0; iy < grid_y; ++iy)
      {
        for (int iz = 0; iz < grid_z; ++iz)
        {
          if (occupied[iy][iz] && !visited[iy][iz])
          {
            std::vector<std::pair<int, int>> cluster_cells;
            std::queue<std::pair<int, int>> q;
            q.push({iy, iz});
            visited[iy][iz] = true;
            
            float sum_x = 0;
            float sum_y = 0;
            float sum_z = 0;
            int point_count = 0;
            
            while(!q.empty())
            {
              std::pair<int, int> curr = q.front();
              q.pop();
              int cy = curr.first;
              int cz = curr.second;
              cluster_cells.push_back({cy, cz});
              
              // Accumulate real coordinates for centroid
              for (int idx : grid[cy][cz])
              {
                const auto& pt = floating_points->points[idx];
                sum_x += pt.x;
                sum_y += pt.y;
                sum_z += pt.z;
                point_count++;
              }
              
              int dy[] = {-1, 1, 0, 0};
              int dz[] = {0, 0, -1, 1};
              for (int d = 0; d < 4; ++d)
              {
                int ny = cy + dy[d];
                int nz = cz + dz[d];
                if (ny >= 0 && ny < grid_y && nz >= 0 && nz < grid_z &&
                    occupied[ny][nz] && !visited[ny][nz])
                {
                  visited[ny][nz] = true;
                  q.push({ny, nz});
                }
              }
            }
            
            // Check if cluster is a valid handle candidate
            // Criteria: 
            // 1. Point count (e.g. > 20 points, < 2000)
            // 2. Height (Z) should be in handle range
            
            if (point_count > 20 && point_count < 2000)
            {
              float avg_z = sum_z / point_count;
              
              if (avg_z >= HANDLE_Z_MIN && avg_z <= HANDLE_Z_MAX)
              {
                has_handle = true;
                handle_midpoint.x = sum_x / point_count;
                handle_midpoint.y = sum_y / point_count;
                handle_midpoint.z = avg_z; // Use average Z
                
                // Populate hole_cloud (now handle_cloud) with the cluster points
                hole_cloud->header = input->header;
                hole_cloud->height = 1;
                hole_cloud->is_dense = true;
                
                uint8_t r = 0, g = 255, b = 0; // Green for handle
                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                float rgb_f;
                std::memcpy(&rgb_f, &rgb, sizeof(float));
                
                for (auto& cell : cluster_cells)
                {
                   for (int idx : grid[cell.first][cell.second])
                   {
                     PointT pt = floating_points->points[idx];
                     pt.rgb = rgb_f;
                     hole_cloud->points.push_back(pt);
                   }
                }
                hole_cloud->width = hole_cloud->points.size();
                
                ROS_INFO_THROTTLE(2.0, "Found floating handle! Pts: %d, Pos: [%.3f, %.3f, %.3f]", 
                                  point_count, handle_midpoint.x, handle_midpoint.y, handle_midpoint.z);
                return true; // Found it
              }
            }
          }
        }
      }
    }
    
    // Remove this plane and continue
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(remaining_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    CloudPtr temp(new PointCloud());
    extract.filter(*temp);
    remaining_cloud = temp;
  }
  
  return false;
}

void PlaneSegmentation::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  last_msg_stamp_ = msg->header.stamp;

  bool has_rgb = false;
  for (const auto &f : msg->fields)
  {
    if (f.name == "rgb" || f.name == "rgba")
    {
      has_rgb = true;
      break;
    }
  }

  try
  {
    if (has_rgb)
    {
      pcl::fromROSMsg(*msg, *raw_cloud_);
    }
    else
    {
      pcl::PointCloud<pcl::PointXYZ> tmp_xyz;
      pcl::fromROSMsg(*msg, tmp_xyz);
      raw_cloud_->header = tmp_xyz.header;
      raw_cloud_->is_dense = tmp_xyz.is_dense;
      raw_cloud_->points.resize(tmp_xyz.points.size());
      uint32_t rgb_int = 0u;
      float rgb_float;
      std::memcpy(&rgb_float, &rgb_int, sizeof(float));
      for (size_t i = 0; i < tmp_xyz.points.size(); ++i)
      {
        raw_cloud_->points[i].x = tmp_xyz.points[i].x;
        raw_cloud_->points[i].y = tmp_xyz.points[i].y;
        raw_cloud_->points[i].z = tmp_xyz.points[i].z;
        raw_cloud_->points[i].rgb = rgb_float;
      }
    }
    is_cloud_updated_ = true;
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("PointCloud conversion failed: %s", e.what());
    raw_cloud_->points.clear();
    is_cloud_updated_ = false;
  }
}


