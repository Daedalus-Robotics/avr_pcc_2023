#include "node.hpp"

#include "system.hpp"

Node::Node(const char name[], const char ns[]) : name(name),
                                                 ns(ns),
                                                 node()
{
}

void Node::setup(rclc_support_t *support, rclc_executor_t *executor)
{
    HANDLE_ROS_ERROR(rclc_node_init_default(&node, name, ns, support), true);
}

void Node::cleanup(rclc_executor_t *executor)
{
    HANDLE_ROS_ERROR(rcl_node_fini(&node), false);
}
