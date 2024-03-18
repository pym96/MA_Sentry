#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <omp.h> // C++17 execution policies
#include <vector>
#include <mutex>
#include <iostream>

class CloudFilter {
public:
    CloudFilter(const double& radius) {
        // Initialize ROS node
        ros::NodeHandle nh;

        this->radius_ = radius;
        // Subscribe to point cloud data
        sub = nh.subscribe("/cloud_registered", 1, &CloudFilter::cloudCallback, this);

        // Publish filtered point cloud
        pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered", 1);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        // Convert ROS message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // Create a new point cloud for the filtered results
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        // Define the center point for filtering
        pcl::PointXYZ center; // Set the appropriate values for the center
        center.x = 0.0;
        center.y = 0.0;
        center.z = 0.0;

        #pragma omp parallel
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_private(new pcl::PointCloud<pcl::PointXYZ>);

            #pragma omp for nowait
            for (size_t i = 0; i < cloud->size(); ++i) {
                if (pcl::geometry::distance(cloud->points[i], center) >= this->radius_) {
                    cloud_filtered_private->push_back(cloud->points[i]);
                }
            }

            #pragma omp critical
            *cloud_filtered += *cloud_filtered_private;
        }

        // Convert the filtered point cloud back to a ROS message
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header.frame_id = cloud_msg->header.frame_id;

        // Publish the filtered point cloud
        pub.publish(output);
    }




private:
    ros::Subscriber sub;
    ros::Publisher pub;
    double radius_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_filter_node");

    ros::NodeHandle nh;

    double radius_;

    if (nh.getParam("/ma_cloud_filter/radius", radius_)) {
        ROS_INFO("Current port name is %s, baud rate is %f.", radius_);
    } else {
        ROS_ERROR("Failed to retrieve the port parameter.");
        return 1;
    }

    CloudFilter cloudFilter(radius_);
    ros::spin();
    return 0;
}
