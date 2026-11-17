#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/geometry.h>

#include <omp.h> // Ensure your compilation environment supports OpenMP


using namespace std;

const double PI = 3.1415926;

string stateEstimationTopic = "/integrated_to_init";
string registeredScanTopic = "/velodyne_cloud_registered";
bool flipStateEstimation = true;
bool flipRegisteredScan = true;
bool sendTF = true;
bool reverseTF = false;

double radius_;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());

nav_msgs::Odometry odomData;
tf::StampedTransform odomTrans;
ros::Publisher *pubOdometryPointer = NULL;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;
ros::Publisher *pubLaserCloudPointer = NULL;

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  odomData = *odom;

  if (flipStateEstimation) {
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

    pitch = -pitch;
    yaw = -yaw;
    // roll = roll + PI;

    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = odom->pose.pose.position.z;
    odomData.pose.pose.position.y = odom->pose.pose.position.x;
    odomData.pose.pose.position.z = odom->pose.pose.position.y;
  }

  // publish odometry messages
  odomData.header.frame_id = "map";
  odomData.child_frame_id = "sensor";
  pubOdometryPointer->publish(odomData);

  // publish tf messages
  odomTrans.stamp_ = odom->header.stamp;
  odomTrans.frame_id_ = "map";
  odomTrans.child_frame_id_ = "sensor";
  odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
  odomTrans.setOrigin(tf::Vector3(odomData.pose.pose.position.x, odomData.pose.pose.position.y, odomData.pose.pose.position.z));

  if (sendTF) {
    if (!reverseTF) {
      tfBroadcasterPointer->sendTransform(odomTrans);
    } else {
      tfBroadcasterPointer->sendTransform(tf::StampedTransform(odomTrans.inverse(), odom->header.stamp, "sensor", "map"));
    }
  }
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*laserCloudIn, *laserCloud);

    if (flipRegisteredScan) {
        int laserCloudSize = laserCloud->points.size();
        for (int i = 0; i < laserCloudSize; i++) {
            laserCloud->points[i].x = laserCloud->points[i].x;
            laserCloud->points[i].y = -laserCloud->points[i].y;
            laserCloud->points[i].z = -laserCloud->points[i].z;
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 使用odomData更新中心点位置
    pcl::PointXYZ center;
    center.x = odomData.pose.pose.position.x;
    center.y = odomData.pose.pose.position.y;
    center.z = odomData.pose.pose.position.z;

    #pragma omp parallel
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_private(new pcl::PointCloud<pcl::PointXYZ>);

        #pragma omp for nowait
        for (size_t i = 0; i < laserCloud->size(); ++i) {
            if (pcl::geometry::distance(laserCloud->points[i], center) >= radius_) {
                cloud_filtered_private->push_back(laserCloud->points[i]);
            }
        }

        #pragma omp critical
        *cloud_filtered += *cloud_filtered_private;
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    output.header.frame_id = "sensor"; // 确保这是正确的坐标系
    output.header.stamp = laserCloudIn->header.stamp;

    pubLaserCloudPointer->publish(output);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "loamInterface");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("stateEstimationTopic", stateEstimationTopic);
  nhPrivate.getParam("registeredScanTopic", registeredScanTopic);
  nhPrivate.getParam("flipStateEstimation", flipStateEstimation);
  nhPrivate.getParam("flipRegisteredScan", flipRegisteredScan);
  nhPrivate.getParam("sendTF", sendTF);
  nhPrivate.getParam("reverseTF", reverseTF);
  nhPrivate.getParam("/ma_cloud_filter/radius", radius_);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry> (stateEstimationTopic, 5, odometryHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> (registeredScanTopic, 5, laserCloudHandler);

  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry> ("/state_estimation", 5);
  pubOdometryPointer = &pubOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/registered_scan", 5);
  pubLaserCloudPointer = &pubLaserCloud;

  ros::spin();

  return 0;
}
