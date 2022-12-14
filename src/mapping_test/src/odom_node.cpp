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

ros::Publisher odom;
ros::Publisher pub_points;
ros::Publisher pub_map;



int seq = 0;

struct IMU_DATA
{
  double time;
  std::array<double,3> imu_eul;
  double DCM[3][3];
  std::array<double,4> local_pose;
};
std::queue<IMU_DATA> imuBuf;

struct LIDAR_DATA
{
  double time;
  std::array<double,3> prev_imu_eul;
  std::array<double,4> prev_local_pose;
  double prev_timestamp;
  double prev_world_pose[3] = {0};
};

IMU_DATA IMU;
LIDAR_DATA LIDAR;

std::array<float,3> cart2sph(float x, float y, float z)
{
  float hxy = hypot(x, y);
  float r = hypot(hxy, z);
  float el = atan2(z, hxy);
  float az = atan2(y, x);

  std::array<float,3> result;
  result[0] = r;
  result[1] = el;
  result[2] = az;

  return result;
}

// Median Filter
double getMedian(double* array, size_t arraySize) {
  size_t center = arraySize / 2; // 요소 개수의 절반값 구하기
  if (arraySize % 2 == 1) { // 요소 개수가 홀수면
    return array[center]; // 홀수 개수인 배열에서는 중간 요소를 그대로 반환
  } else {
    return (array[center - 1] + array[center]) / 2.0; // 짝수 개 요소는, 중간 두 수의 평균 반환
  }
}


// qsort 내부에서 사용되는 비교 함수
int comparisonFunctionDouble(const void *a, const void *b) {
  if (*(double*)a <  *(double*)b) return -1;
  if (*(double*)a == *(double*)b) return  0;

  return 1;
}


/*
void publish_odom(std::string frame_id, uint32_t seq, ros::Time timestamp, std::string  child_frame_id)
{
  nav_msgs::Odometry msg;

  msg.header.frame_id = frame_id;
  msg.header.seq = seq;
  msg.header.stamp = timestamp;
  msg.child_frame_id = child_frame_id;

  
  tf::poseKindrToMsg(transform_, &msg.pose.pose);
  //msg.pose.covariance = pose_covariance_;

  tf::vectorKindrToMsg(linear_velocity_, &msg.twist.twist.linear);
  tf::vectorKindrToMsg(angular_velocity_, &msg.twist.twist.angular);
  //msg.twist.covariance = twist_covariance_;

  odom.publish(msg);
}
*/

std::array<double,3> quat2eul(tf::Quaternion q)
{
  
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  
  std::array<double,3> imu_eul;
  imu_eul[0] = roll;
  imu_eul[1] = pitch;
  imu_eul[2] = yaw;

  //printf("euler : %f, %f, %f\n", imu_eul[0], imu_eul[1], imu_eul[2]);
  return imu_eul;
}

//std::array<std::array<double, 3>, 3> quat2DCM(double q1, double q2, double q3, double q4)
void quat2DCM(double q1, double q2, double q3, double q4)
{
	IMU.DCM[0][0] = q4 * q4 + q1 * q1 - q2 * q2 - q3 * q3;
  IMU.DCM[0][1] = 2 * (q1 * q2 - q3 * q4);
  IMU.DCM[0][2] = 2 * (q1 * q3 + q2 * q4);
  IMU.DCM[1][0] = 2 * (q1 * q2 + q3 * q4);
  IMU.DCM[1][1] = q4 * q4 - q1 * q1 + q2 * q2 - q3 * q3;
  IMU.DCM[1][2] = 2 * (q2 * q3 - q1 * q4);
  IMU.DCM[2][0] = 2 * (q1 * q3 - q2 * q4);
  IMU.DCM[2][1] = 2 * (q2 * q3 + q1 * q4);
  IMU.DCM[2][2] = q4 * q4 - q1 * q1 - q2 * q2 + q3 * q3;
}


void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{

  IMU.time = msg->header.stamp.toSec();
  double x_org = msg->orientation.x;
  double y_org = msg->orientation.y;
  double z_org = msg->orientation.z;
  double w_org = msg->orientation.w;

  tf::Quaternion q(x_org, y_org, z_org, w_org);
  //eular angler 변환 
  IMU.imu_eul = quat2eul(q);

  // quaternion -> 회전변환 matrix 생성

  quat2DCM(x_org, y_org, z_org, w_org);

  imuBuf.push(IMU);

  return;
}

void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  if(seq % 2 != 0)
    seq++;
  else
  {
    double begin = ros::Time::now().toSec();

    LIDAR.time = msg->header.stamp.toSec();

    // Msg to pointcloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg,*ptcloud); // ros msg 에서 pcl cloud 데이터로 변환
        
    // Get rotation data from IMU
    double current_DCM[3][3];
    IMU_DATA data;
    while(true)
    {
      /*
      if(imuBuf.front().time - msg->header.stamp.toSec() < 0.5)
      {
        data = imuBuf.front();
        if(!imuBuf.empty()) imuBuf.pop();
        break;
      }
      */
      if(imuBuf.empty()) break;

      if(msg->header.stamp.toSec() - imuBuf.front().time < 0.5)
      {
        data = imuBuf.front();
        if(!imuBuf.empty()) imuBuf.pop();
        break;
      }

      if(!imuBuf.empty()) imuBuf.pop();
    }

    memcpy(current_DCM, data.DCM, sizeof(data.DCM));
    printf("IMU Time   : %f\nLidar Time : %f\n", data.time, LIDAR.time);
  //  memcpy(current_DCM, IMU.DCM, sizeof(IMU.DCM));

    for (int i = 0; i < 3; i++)
        printf("DCM : %f, %f, %f\n", current_DCM[i][0], current_DCM[i][1], current_DCM[i][2]);
        printf("Eul : %f, %f, %f\n", RAD2DEG(data.imu_eul[0]), RAD2DEG(data.imu_eul[1]), RAD2DEG(data.imu_eul[2])); 

    // Rotate pointcloud
    size_t num_points = ptcloud->size(); 
    
    double local_points[num_points][3];
    double speed[num_points];

    for (int cnt = 0; cnt < num_points; cnt++)
    {
      local_points[cnt][0] = double(ptcloud->points[cnt].x);
      local_points[cnt][1] = double(ptcloud->points[cnt].y);
      local_points[cnt][2] = double(ptcloud->points[cnt].z);
      speed[cnt] = double(ptcloud->points[cnt].intensity);
    }
  
    double distv[num_points] = {0};
    double vx[num_points] = {0};
    double vy[num_points] = {0};
    double vz[num_points] = {0};

    for (int cnt = 0; cnt < num_points; cnt++)
    {
      distv[cnt] = sqrt(local_points[cnt][0] * local_points[cnt][0] + local_points[cnt][1] * local_points[cnt][1] + local_points[cnt][2] * local_points[cnt][2] );
      vx[cnt] = local_points[cnt][0] / distv[cnt] * speed[cnt];
      vy[cnt] = local_points[cnt][1] / distv[cnt] * speed[cnt];
      vz[cnt] = local_points[cnt][2] / distv[cnt] * speed[cnt];
      //printf("vel : %f, %f, %f \n", vx[cnt], vy[cnt], vz[cnt]);
    }
    
    double median_vx;
    double median_vy;
    double median_vz;
    double median_vel[] = {0.0, 0.0, 0.0};
    for (int cnt = 0; cnt < num_points; cnt++)
    {
      qsort((void *)vx, sizeof(vx) / sizeof(vx[0]), sizeof(vx[0]), comparisonFunctionDouble);
      qsort((void *)vy, sizeof(vy) / sizeof(vy[0]), sizeof(vy[0]), comparisonFunctionDouble);
      qsort((void *)vz, sizeof(vz) / sizeof(vz[0]), sizeof(vz[0]), comparisonFunctionDouble);

      median_vx = getMedian(vx, sizeof(vx) / sizeof(vx[0]));
      median_vy = getMedian(vy, sizeof(vy) / sizeof(vy[0]));
      median_vz = getMedian(vz, sizeof(vz) / sizeof(vz[0]));   
    }
    median_vel[0] = median_vx; median_vel[1] = median_vy; median_vel[2] = median_vz;

    //printf("median vel : %f, %f, %f \n", median_vx, median_vy, median_vz);

    //// rotation (body to navigation)
    // DCM * xyz pose
    double world_points[num_points][3] = {0};  // world
    double temp_local[3] = {0};
    
    for (int cnt = 0; cnt < num_points; cnt++)
    {
      for (int l = 0; l < 3; l++)
      {
        temp_local[l] = local_points[cnt][l];
      }

      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 3; j++)
          world_points[cnt][i] += current_DCM[i][j] * temp_local[j];
          //printf("%f ", pose[cnt][i]);
      }
  /*    
      for (int i = 0; i < 3; i++)
      {
          world_points[cnt][i] = temp_local[i];
          //printf("%f ", pose[cnt][i]);
      }
  */
      //printf("\n");
    }

    // rotation
    double v_rot[3] = {0};

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        v_rot[i] += current_DCM[i][j] * median_vel[j];

    // navigation 좌표계에서 이동한 양
    double dt;
    double world_pose[3] = {0};

    if(seq != 0)
    {
      dt = LIDAR.time - LIDAR.prev_timestamp;
      world_pose[0] = LIDAR.prev_world_pose[0] + v_rot[0] * dt;
      world_pose[1] = LIDAR.prev_world_pose[1] + v_rot[1] * dt;
      world_pose[2] = LIDAR.prev_world_pose[2] + v_rot[2] * dt;
    }

    for (int cnt = 0; cnt < num_points; cnt++)
    {
      world_points[cnt][0] = world_points[cnt][0] + world_pose[0];
      world_points[cnt][1] = world_points[cnt][1] + world_pose[1];
      world_points[cnt][2] = world_points[cnt][2] + world_pose[2];
    }


  //    printf("local: %f, %f, %f\n", local[cnt][0], local[cnt][1], local[cnt][2]);
  //    printf("world: %f, %f, %f\n", pose[cnt][0], pose[cnt][1], pose[cnt][2]);
  //  }
  //  printf("dt: %f - %f = %f \n", LIDAR.time, LIDAR.prev_timestamp, dt);


    // Convert to pointcloud2 and publish
    sensor_msgs::PointCloud2 pc_msg;
    sensor_msgs::PointCloud2Modifier mod(pc_msg);
            mod.setPointCloud2Fields(5, "timestamp", 1, sensor_msgs::PointField::UINT32,
                                  "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32,
                                  "intensity", 1, sensor_msgs::PointField::FLOAT32);

    pc_msg.header.seq = seq;
    pc_msg.header.stamp = ros::Time::now();
    pc_msg.header.frame_id = "map";

    // Fill in the size of the cloud
    pc_msg.height = 1;
    pc_msg.width = num_points; // <-- this is point number

    // Set up memory for our points
    mod.resize(pc_msg.height * pc_msg.width);

    // Now create iterators for fields
    sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(pc_msg, "intensity");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_s(pc_msg, "timestamp");

    uint32_t nsectime = ros::Time::now().nsec;
    uint32_t arrtime[num_points] = {0};
    for (int cnt = 0; cnt < num_points; cnt++)
    {
      *iter_x = world_points[cnt][0];
      *iter_y = world_points[cnt][1];
      *iter_z = world_points[cnt][2];
      *iter_i = speed[cnt];
      *iter_s = nsectime; 
      arrtime[cnt] = nsectime;
      ++iter_x; ++iter_y; ++iter_z, ++iter_i, ++iter_s;
    }

    LIDAR.prev_world_pose[0] = world_pose[0];
    LIDAR.prev_world_pose[1] = world_pose[1];
    LIDAR.prev_world_pose[2] = world_pose[2];

    pub_points.publish (pc_msg);

    // Publish odometry & map
    sensor_msgs::PointCloud2 pc_map;

    if (seq % 5 == 0)
    {
      for (int cnt = 0; cnt < num_points; cnt++)
      {
        temp_map_x.push_back(world_points[cnt][0]);
        temp_map_y.push_back(world_points[cnt][1]);
        temp_map_z.push_back(world_points[cnt][2]);
        temp_map_speed.push_back(speed[cnt]);
        temp_map_time.push_back(arrtime[cnt]);
      }

      int buf_size = temp_map_x.size();
    //  printf("map buffer size : %d\n", size);
      sensor_msgs::PointCloud2Modifier mod_map(pc_map);
      mod_map.setPointCloud2Fields(5, "timestamp", 1, sensor_msgs::PointField::UINT32,
                              "x", 1, sensor_msgs::PointField::FLOAT32,
                              "y", 1, sensor_msgs::PointField::FLOAT32,
                              "z", 1, sensor_msgs::PointField::FLOAT32,
                              "intensity", 1, sensor_msgs::PointField::FLOAT32);

      pc_map.header.seq = seq;
      pc_map.header.stamp = ros::Time::now();
      pc_map.header.frame_id = "map";

      // Fill in the size of the cloud
      pc_map.height = 1;
      pc_map.width = buf_size; // <-- this is point number

      // Set up memory for our points
      mod_map.resize(pc_map.height * pc_map.width);

      // Now create iterators for fields
      sensor_msgs::PointCloud2Iterator<float> iter_x_map(pc_map, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y_map(pc_map, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z_map(pc_map, "z");
      sensor_msgs::PointCloud2Iterator<float> iter_i_map(pc_map, "intensity");
      sensor_msgs::PointCloud2Iterator<uint32_t> iter_s_map(pc_map, "timestamp");

      for (int cnt = 0; cnt < buf_size; cnt++)
      {
        *iter_x_map = temp_map_x[cnt];
        *iter_y_map = temp_map_y[cnt];
        *iter_z_map = temp_map_z[cnt];
        *iter_i_map = temp_map_speed[cnt];
        *iter_s_map = temp_map_time[cnt];
  ;
        ++iter_x_map; ++iter_y_map; ++iter_z_map, ++iter_i_map, ++iter_s_map;
      }
      pub_map.publish (pc_map);
    }
    
    ROS_INFO("Published sequence: %d", seq);
    //  printf("====================================================================\n\n");



/*  -------------------------------------------------------------------------------------------------- */
/*
  std::array<double,3> diff_imu_eul;
  for (int i = 0; i < 3; i++)
  {
    diff_imu_eul[i] = IMU.imu_eul[i] - LIDAR.prev_imu_eul[i];
  }

  // -pi ~ pi 로 갑자기 변하는 부분 해결하기 위해
  for (int j = 0; j < 3; j++)
  {
    if (diff_imu_eul[j] > M_PI)
      diff_imu_eul[j] = diff_imu_eul[j] - 2 * M_PI;
    else if (diff_imu_eul[j] < -M_PI)
      diff_imu_eul[j] = diff_imu_eul[j] + 2 * M_PI;
  }  

  // sync 문제로 차이 데이터 delay 추후 현재값에서 이전 값을 빼면 됨
  std::array<double,3> delta_imu_eul;
  for (int k = 0; k < 3; k++)
  {
    delta_imu_eul[k] = diff_imu_eul[k] / line_num;
  }


  size_t num_points = ptcloud->size();  
  printf("pt size : %zu\n", num_points);
  
  float x[num_points];
  float y[num_points];
  float z[num_points];
  
  float r[num_points];
  float el[num_points];
  float az[num_points];

  std::array<float,3> sph_coord;
  std::vector<float> arr_el;

  for (int cnt = 0; cnt < num_points; cnt++)
  {
    x[cnt] = ptcloud->points[cnt].x;
    y[cnt] = ptcloud->points[cnt].y;
    z[cnt] = ptcloud->points[cnt].z;

    sph_coord = cart2sph(x[cnt], y[cnt], z[cnt]);
    r[cnt] = sph_coord[0];
    el[cnt] = sph_coord[1];
    az[cnt] = sph_coord[2];
  }
  
  float diff_el[num_points-1];
  for (int cnt = 0; cnt < num_points-1; cnt++)
  {
    diff_el[cnt] = el[cnt + 1] - el[cnt];
    if (abs(diff_el[cnt]) > 0.0001)
      arr_el.push_back(abs(diff_el[cnt]));
  }

  for (int l = 0; l < arr_el.size(); l ++)
  {
   // if 
  }

  */
/*  -------------------------------------------------------------------------------------------------- */



  //printf("Lidar time : %f, IMU time : %f\n", LIDAR.time, IMU.time);
  LIDAR.prev_timestamp = LIDAR.time;
  seq++;

  double end = ros::Time::now().toSec();
  double tact = end - begin;

  ROS_INFO("tact: %f", tact);
  printf("====================================================================\n\n");

  } 

  return;
}

int main(int argc, char **argv)
{
  ROS_INFO("odom_node start!");
  ros::init(argc, argv, "odom_node");

  ros::NodeHandle nh;
  //map_buf.push_back(temp_map_x);
  //map_buf.push_back(temp_map_y);
  //map_buf.push_back(temp_map_z);
  
  ros::Subscriber sub_lidar = nh.subscribe("/ifwslidar_pcl", 1000, lidar_callback);
  ros::Subscriber sub_imu = nh.subscribe("/imu/data", 1000, imu_callback);

  pub_points = nh.advertise<sensor_msgs::PointCloud2> ("/map/current", 1000);
  pub_map = nh.advertise<sensor_msgs::PointCloud2> ("/map/map", 1000);
  ros::spin();

  return 0;
}