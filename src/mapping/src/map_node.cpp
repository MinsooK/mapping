#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

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

#include "mapping/map.h"


int seq = 0;
int pub_seq = 0;
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

std::array<double,3> quat2eul(tf::Quaternion q)
{
  
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    //IMU.imu_eul[0] = roll;
    //IMU.imu_eul[1] = pitch;
    //IMU.imu_eul[2] = yaw;
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
    IMU.imu_quat.x = x_org;
    IMU.imu_quat.y = y_org;
    IMU.imu_quat.z = z_org;
    IMU.imu_quat.w = w_org;

    tf::Quaternion q(x_org, y_org, z_org, w_org);
    //eular angler 변환 
    IMU.imu_eul = quat2eul(q);
    //quat2eul(q);

    // quaternion -> 회전변환 matrix 생성

    quat2DCM(x_org, y_org, z_org, w_org);

    imuBuf.push(IMU);

    return;
}

void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if(seq % 5 != 0)
        seq++;
    else
    {
        pub_seq++;
        double begin = ros::Time::now().toSec();

        LIDAR.time = msg->header.stamp.toSec();

        // Msg to pointcloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg,*ptcloud); // ros msg 에서 pcl cloud 데이터로 변환
            
        // Get rotation data from IMU
        double current_DCM[3][3];
        IMU_DATA CUR_IMU;
        while(true)
        {
            /*
            if(imuBuf.front().time - msg->header.stamp.toSec() < 0.5)
            {
                CUR_IMU = imuBuf.front();
                if(!imuBuf.empty()) imuBuf.pop();
                break;
            }
            */
            if(imuBuf.empty()) break;

            if(msg->header.stamp.toSec() - imuBuf.front().time < 0.3)
            {
                CUR_IMU = imuBuf.front();
                if(!imuBuf.empty()) imuBuf.pop();
                break;
            }

            if(!imuBuf.empty()) imuBuf.pop();
        }

        memcpy(current_DCM, CUR_IMU.DCM, sizeof(CUR_IMU.DCM));
    //    printf("IMU Time   : %f\nLidar Time : %f\n", CUR_IMU.time, LIDAR.time);

    //    for (int i = 0; i < 3; i++)
    //        printf("DCM : %f, %f, %f\n", current_DCM[i][0], current_DCM[i][1], current_DCM[i][2]);
    //        printf("Eul : %f, %f, %f\n", RAD2DEG(CUR_IMU.imu_eul[0]), RAD2DEG(CUR_IMU.imu_eul[1]), RAD2DEG(CUR_IMU.imu_eul[2])); 

        // Rotate pointcloud
        size_t num_points = ptcloud->size(); 
        int total_points = static_cast<int>(num_points);
        ROS_INFO("%d points received!!", total_points);
        double local_points[num_points][3];
        double speed[num_points];

        double distv[num_points] = {0};
        double vx[num_points] = {0};
        double vy[num_points] = {0};
        double vz[num_points] = {0};

        for (int cnt = 0; cnt < total_points; cnt++)
        {
            local_points[cnt][0] = double(ptcloud->points[cnt].x);
            local_points[cnt][1] = double(ptcloud->points[cnt].y);
            local_points[cnt][2] = double(ptcloud->points[cnt].z);
            speed[cnt] = double(ptcloud->points[cnt].intensity);

            distv[cnt] = sqrt(local_points[cnt][0] * local_points[cnt][0] + local_points[cnt][1] * local_points[cnt][1] + local_points[cnt][2] * local_points[cnt][2] );
            vx[cnt] = local_points[cnt][0] / distv[cnt] * speed[cnt];
            vy[cnt] = local_points[cnt][1] / distv[cnt] * speed[cnt];
            vz[cnt] = local_points[cnt][2] / distv[cnt] * speed[cnt];
        }
   
        double median_vx = 0.0;
        double median_vy = 0.0;
        double median_vz = 0.0;
        double median_vel[] = {0.0, 0.0, 0.0};

        qsort((void *)vx, sizeof(vx) / sizeof(vx[0]), sizeof(vx[0]), comparisonFunctionDouble);
        qsort((void *)vy, sizeof(vy) / sizeof(vy[0]), sizeof(vy[0]), comparisonFunctionDouble);
        qsort((void *)vz, sizeof(vz) / sizeof(vz[0]), sizeof(vz[0]), comparisonFunctionDouble);

        median_vx = getMedian(vx, sizeof(vx) / sizeof(vx[0]));
        median_vy = getMedian(vy, sizeof(vy) / sizeof(vy[0]));
        median_vz = getMedian(vz, sizeof(vz) / sizeof(vz[0]));   
        
        median_vel[0] = median_vx; median_vel[1] = median_vy; median_vel[2] = median_vz;

        //// rotation (body to navigation)
        // DCM * xyz pose
        double world_points[total_points][3] = {0};  // world
        double temp_local[3] = {0};
        
        for (int cnt = 0; cnt < total_points; cnt++)
        {
            for (int l = 0; l < 3; l++)
            {
                temp_local[l] = local_points[cnt][l];
            }

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                world_points[cnt][i] += current_DCM[i][j] * temp_local[j];
            }
        }

        // rotation
        double v_rot[3] = {0};

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                v_rot[i] += current_DCM[i][j] * median_vel[j];

        // navigation 좌표계에서 이동한 양
        double dt = {0};
        double world_pose[3] = {0};

        if(seq != 0)
        {
            dt = LIDAR.time - LIDAR.prev_timestamp;
            world_pose[0] = LIDAR.prev_world_pose[0] + v_rot[0] * dt;
            world_pose[1] = LIDAR.prev_world_pose[1] + v_rot[1] * dt;
            world_pose[2] = LIDAR.prev_world_pose[2] + v_rot[2] * dt;
        }

        for (int cnt = 0; cnt < total_points; cnt++)
        {
            world_points[cnt][0] = world_points[cnt][0] + world_pose[0];
            world_points[cnt][1] = world_points[cnt][1] + world_pose[1];
            world_points[cnt][2] = world_points[cnt][2] + world_pose[2];
        }

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
        pc_msg.width = total_points; // <-- this is point number

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
        for (int cnt = 0; cnt < total_points; cnt++)
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

        ////////// Odom /////////
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "map";

        odom.pose.pose.position.x = world_pose[0];
        odom.pose.pose.position.y = world_pose[1];
        odom.pose.pose.position.z = world_pose[2];

        odom.pose.pose.orientation.x = CUR_IMU.imu_quat.x;
        odom.pose.pose.orientation.y = CUR_IMU.imu_quat.y;
        odom.pose.pose.orientation.z = CUR_IMU.imu_quat.z;
        odom.pose.pose.orientation.w = CUR_IMU.imu_quat.w;

        odom.child_frame_id = "odom";
        odom.twist.twist.linear.x = median_vx;
        odom.twist.twist.linear.y = median_vy;
        odom.twist.twist.linear.z = median_vz;

        double vel_yaw = (CUR_IMU.imu_eul[2] - prev_imu_eul_yaw) / dt;
        odom.twist.twist.angular.z = vel_yaw;

        //printf("(%f - %f)/%f = %f\n", RAD2DEG(CUR_IMU.imu_eul[2]), RAD2DEG(prev_imu_eul_yaw), dt, RAD2DEG(vel_yaw));
        pub_odom.publish(odom);
        //////////////////////////
    
        /////// Publish TF //////
        static tf2_ros::StaticTransformBroadcaster static_broadcaster;
        geometry_msgs::TransformStamped static_transformStamped;

        static_transformStamped.header.stamp = ros::Time::now();
        static_transformStamped.header.frame_id = "map";
        static_transformStamped.child_frame_id = "robot";
        static_transformStamped.transform.translation.x = world_pose[0];
        static_transformStamped.transform.translation.y = world_pose[1];
        static_transformStamped.transform.translation.z = world_pose[2];
//        tf2::Quaternion quat;
//        quat.setRPY(CUR_IMU.imu_eul[0], CUR_IMU.imu_eul[1], CUR_IMU.imu_eul[2]);
        static_transformStamped.transform.rotation.x = CUR_IMU.imu_quat.x;
        static_transformStamped.transform.rotation.y = CUR_IMU.imu_quat.y;
        static_transformStamped.transform.rotation.z = CUR_IMU.imu_quat.z;
        static_transformStamped.transform.rotation.w = CUR_IMU.imu_quat.w;
        static_broadcaster.sendTransform(static_transformStamped);
       
        ///////////////////////////

       if (seq % 10 == 0)
        {
            for (int cnt = 0; cnt < total_points; cnt++)
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
                ++iter_x_map; ++iter_y_map; ++iter_z_map, ++iter_i_map, ++iter_s_map;
            }
            ROS_INFO("Map updated!");
            pub_map.publish (pc_map);
        }
        
        ROS_INFO("Published sequence: %d", pub_seq);
        
        //printf("Lidar time : %f, IMU time : %f\n", LIDAR.time, IMU.time);
        LIDAR.prev_timestamp = LIDAR.time;
        prev_imu_eul_yaw = CUR_IMU.imu_eul[2];
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
    ROS_INFO("map_node start!");
    ros::init(argc, argv, "map_node");

    ros::NodeHandle nh;

    ros::Subscriber sub_lidar = nh.subscribe("/ifwslidar_pcl", 1000, lidar_callback);
    ros::Subscriber sub_imu = nh.subscribe("/imu/data", 1000, imu_callback);

    pub_points = nh.advertise<sensor_msgs::PointCloud2> ("/map/current", 1000);
    pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 50);
    pub_map = nh.advertise<sensor_msgs::PointCloud2> ("/map/map", 1000);

    ros::spin();

    return 0;
}