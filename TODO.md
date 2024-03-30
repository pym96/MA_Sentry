class CheckHP : public BT::ConditionNode
{
public:
    CheckHP(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config), global_current_hp(600) // 初始化血量
    {
        // 在构造函数中订阅，确保只订阅一次
        hp_sub_ = nh_.subscribe("/sentry_hp", 5, &CheckHP::hp_callback, this);
    }

    BT::NodeStatus tick() override {
        uint16_t current_hp_copy;
        {
            std::lock_guard<std::mutex> lock(hp_mutex_);
            current_hp_copy = global_current_hp;
        }
        return current_hp_copy < 400 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts() {
        return {};
    }

private:
    uint16_t global_current_hp;
    ros::NodeHandle nh_;
    ros::Subscriber hp_sub_;
    std::mutex hp_mutex_; // 用于保护 global_current_hp 的互斥锁

    void hp_callback(const std_msgs::UInt16::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(hp_mutex_);
        global_current_hp = msg->data;
    }
};
