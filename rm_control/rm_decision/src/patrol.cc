#include "rm_decision/patrol.hpp"


PatrolAction::PatrolAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

BT::NodeStatus PatrolAction::tick(){
    // Patrol logic
    return BT::NodeStatus::RUNNING;
}

BT::PortsList PatrolAction::providedPorts(){
    return {};
}