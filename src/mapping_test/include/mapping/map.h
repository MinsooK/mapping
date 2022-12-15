#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#define _USE_MATH_DEFINES

#include <cmath>
#include <math.h>
#include <mutex>
#include <queue>
#include <vector>
#include <algorithm>
#include<iostream>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

int line_num = 17; // Lidar line 개수, lidar data 회전에 대한 보상을 위한 정보


//std::vector<std::vector<double>> map_buf; 
std::vector<double> temp_map_x; 
std::vector<double> temp_map_y; 
std::vector<double> temp_map_z; 
std::vector<double> temp_map_speed;
std::vector<double> temp_map_time;

std::mutex mutex_lock;

ros::Publisher pub_odom;
ros::Publisher pub_points;
ros::Publisher pub_map;


struct IMU_DATA
{
  double time;
  geometry_msgs::Quaternion imu_quat;
  std::array<double,3> imu_eul;
  double DCM[3][3];
  //std::array<double,4> local_pose;
};

struct LIDAR_DATA
{
  double time;
  //std::array<double,3> prev_imu_eul;
  std::array<double,4> prev_local_pose;
  double prev_timestamp;
  double prev_world_pose[3] = {0};
};

std::queue<IMU_DATA> imuBuf;
double prev_imu_eul_yaw = 0.0;


// Publish odometry & map
sensor_msgs::PointCloud2 pc_map;

