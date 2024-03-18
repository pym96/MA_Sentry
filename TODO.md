#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <execution> // C++17 execution policies
#include <vector>
#include <mutex>

class CloudFilter {
public:
    CloudFilter() {
        // Initialize ROS node
        ros::NodeHandle nh;

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

        // Define the center point and radius for filtering
        pcl::PointXYZ center; // Set the appropriate values for the center
        center.x = 0.0;
        center.y = 0.0;
        center.z = 0.0;
        double radius = 0.35;

        // Mutex for thread-safe push_back operation
        std::mutex mutex;

        // Iterate through each point in parallel and add it to the filtered cloud if it's outside the radius
        std::for_each(std::execution::par, cloud->begin(), cloud->end(), [&](const pcl::PointXYZ& point) {
            if (pcl::geometry::distance(point, center) >= radius) {
                std::lock_guard<std::mutex> lock(mutex);
                cloud_filtered->push_back(point);
            }
        });

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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_filter_node");
    CloudFilter cloudFilter;
    ros::spin();
    return 0;
}
