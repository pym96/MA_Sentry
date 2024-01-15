int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_to_goal_rotation");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        // 假设odom的frame_id是 "odom"，机器人的frame_id是 "base_link"
        listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return -1;
    }

    // 假设目标点的坐标是 (goal_x, goal_y)
    double goal_x = 10.0;
    double goal_y = 10.0;

    // 获取odom的位置和方向
    double odom_x = transform.getOrigin().x();
    double odom_y = transform.getOrigin().y();
    tf::Quaternion q = transform.getRotation();
    Eigen::Quaterniond quaternion(q.w(), q.x(), q.y(), q.z());
    Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();

    // 计算从odom到目标点的方向向量
    Eigen::Vector3d direction_to_goal(goal_x -
