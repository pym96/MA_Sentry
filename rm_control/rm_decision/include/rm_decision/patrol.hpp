#ifndef __MA_PATROL_HPP__
#define __MA_PATROL_HPP__

#include <behaviortree_cpp_v3/action_node.h>
#include <string>

class PatrolAction: public BT::SyncActionNode
{
    public:
        PatrolAction(const std::string& name, const BT::NodeConfiguration& config);
        BT::NodeStatus tick() override;
};


#endif