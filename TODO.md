#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>

class CloudFilter {
public:
    CloudFilter() {
        // 初始化ROS节点
        ros::NodeHandle nh;

        // 订阅点云数据
        sub = nh.subscribe("/cloud_registered", 1, &CloudFilter::cloudCallback, this);

        // 发布过滤后的点云
        pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered", 1);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        // 将ROS消息转换为PCL点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // 创建过滤器并应用
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(0.35);
        outrem.setMinNeighborsInRadius(2); // 设置在0.35半径内最小邻居数，根据需要调整
        outrem.filter(*cloud_filtered);

        // 将过滤后的点云转换回ROS消息
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header.frame_id = cloud_msg->header.frame_id; // 保持帧ID

        // 发布过滤后的点云
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
