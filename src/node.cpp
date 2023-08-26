#include "node.hpp"

Node::Node(rclc_support_t *support, const char name[], const char ns[]) : node()
{
    HANDLE_ERROR(rclc_node_init_default(&node, name, ns, support), true);
    CLEANUP_ACTION(this, [](Node *context) { return rcl_node_fini(&context->node); });
}
