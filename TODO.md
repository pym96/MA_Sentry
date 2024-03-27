```
class MoveToPosition : public BT::SyncActionNode
{
public:
    MoveToPosition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    BT::NodeStatus tick() override {
        geometry_msgs::Point target_position;
        if (!getInput<geometry_msgs::Point>("target_position", target_position)) {
            ROS_ERROR("Missing target_position input");
            return BT::NodeStatus::FAILURE;
        }
        // 基于target_position计算并发布速度命令，这里省略计算过程
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 1.0; // 示例值，你需要根据实际目标位置计算
        // 发布速度命令，确保有一个全局或成员变量的ros::Publisher来发布
        // vel_pub.publish(cmd_vel);
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts() {
        return { BT::InputPort<geometry_msgs::Point>("target_position") };
    }
};


// 假设的全局血量变量，或者更好的做法是通过某种机制传递到CheckHP节点
int global_current_hp = 1000;

void hpCallback(const std_msgs::Int32::ConstPtr& msg) {
    global_current_hp = msg->data;
}

class CheckHP : public BT::ConditionNode
{
public:
    CheckHP(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}

    BT::NodeStatus tick() override {
        // 使用全局或通过某种机制传递的血量变量
        return global_current_hp < 400 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "behavior_tree_robot");
    ros::NodeHandle nh;

    // 订阅 /current_hp 以更新血量
    ros::Subscriber hp_sub = nh.subscribe("/current_hp", 10, hpCallback);

    BT::BehaviorTreeFactory factory;

    // 注册自定义节点
    factory.registerNodeType<MoveToPosition>("MoveToPosition");
    factory.registerNodeType<CheckHP>("CheckHP");
    factory.registerNodeType<Wait>("Wait");

    // 创建行为树
    auto tree = factory.createTreeFromFile("/path/to/your_tree.xml");

    ros::Rate rate(10);
    while (ros::ok()) {
        // 运行行为树
        tree.tickRoot();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

```
