```
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sstream>

void pathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    ROS_INFO("Received a path message");
    ROS_INFO("Frame ID: %s", path_msg->header.frame_id.c_str());
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

    // 打印每个路径点的详细信息
    int count = 0;
    for(const auto& pose_stamped : path_msg->poses) {
        std::stringstream ss;
        ss << "Path point " << count << ": ";
        ss << "Position => x: " << pose_stamped.pose.position.x << ", ";
        ss << "y: " << pose_stamped.pose.position.y << ", ";
        ss << "z: " << pose_stamped.pose.position.z << "; ";
        ss << "Orientation => x: " << pose_stamped.pose.orientation.x << ", ";
        ss << "y: " << pose_stamped.pose.orientation.y << ", ";
        ss << "z: " << pose_stamped.pose.orientation.z << ", ";
        ss << "w: " << pose_stamped.pose.orientation.w;
        ROS_INFO("%s", ss.str().c_str());
        count++;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_listener");
    ros::NodeHandle nh;

    ros::Subscriber path_sub = nh.subscribe("/path", 1000, pathCallback);

    ros::spin();

    return 0;
}
```
