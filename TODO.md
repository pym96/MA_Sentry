process[velocity_smoother_ema-1]: started with pid [12814]
[ERROR] [1711206976.444113546]: Client [/rostopic_11537_1711206883903] wants topic /cmd_vel to have datatype/md5sum [geometry_msgs/TwistStamped/98d34b0043a2093cf9d9345ab6eef12e], but our version has [geometry_msgs/Twist/9f195f881246fdfa2798d1d3eebca84a]. Dropping connection.
[ERROR] [1711206976.640208152]: Client [/vehicleSimulator] wants topic /cmd_vel to have datatype/md5sum [geometry_msgs/TwistStamped/98d34b0043a2093cf9d9345ab6eef12e], but our version has [geometry_msgs/Twist/9f195f881246fdfa2798d1d3eebca84a]. Dropping connection.

```

#include <tf/tf.h>

template <typename T>
void LaserMapping::SetPosestamp(T &out) {
    out.pose.position.x = state_point_.pos(0);
    out.pose.position.y = state_point_.pos(1);
    out.pose.position.z = state_point_.pos(2);

    // 获得当前四元数
    tf::Quaternion q(
        state_point_.rot.coeffs()[0],
        state_point_.rot.coeffs()[1],
        state_point_.rot.coeffs()[2],
        state_point_.rot.coeffs()[3]);

    // 四元数转换为欧拉角
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // 取pitch和yaw的反值
    pitch = -pitch;
    yaw = -yaw;

    // 将修改后的欧拉角转换回四元数
    tf::Quaternion q_new = tf::createQuaternionFromRPY(roll, pitch, yaw);

    // 更新out对象的方向
    out.pose.orientation.x = q_new.x();
    out.pose.orientation.y = q_new.y();
    out.pose.orientation.z = q_new.z();
    out.pose.orientation.w = q_new.w();
}


```


```
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

// 全局ROS发布器
ros::Publisher vel_pub;

// MoveToPosition 实现
class MoveToPosition : public BT::SyncActionNode
{
public:
    MoveToPosition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
        // 实现移动逻辑，这里作为示例
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 1.0; // 示例值
        vel_pub.publish(cmd_vel);
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts() {
        return { BT::InputPort<geometry_msgs::Point>("target_position") };
    }
};

// CheckHP 实现
int current_hp = 1000; // 假设的血量值

class CheckHP : public BT::ConditionNode
{
public:
    CheckHP(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}

    BT::NodeStatus tick() override {
        return current_hp < 400 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

// Wait 实现
class Wait : public BT::SyncActionNode
{
public:
    Wait(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
        ros::Duration(5).sleep(); // 等待5秒
        return BT::NodeStatus::SUCCESS;
    }
};


```

```
int main(int argc, char** argv) {
    ros::init(argc, argv, "bt_navigator");
    ros::NodeHandle nh;

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 定义行为树
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<MoveToPosition>("MoveToPosition");
    factory.registerNodeType<CheckHP>("CheckHP");
    factory.registerNodeType<Wait>("Wait");

    // 构建行为树
    auto tree = factory.createTreeFromFile("/path/to/your_tree.xml");

    // 运行行为树
    BT::NodeStatus status;
    do {
        status = tree.tickRoot();
        ros::spinOnce();
    } while (status == BT::NodeStatus::RUNNING);

    return 0;
}



```
