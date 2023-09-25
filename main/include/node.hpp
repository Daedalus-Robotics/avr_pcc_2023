#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifndef AVR_PCC_2023_NODE_HPP
#define AVR_PCC_2023_NODE_HPP

/**
 * Represents a micro ros node
 */
class Node
{
public:
    Node(const char name[], const char ns[]);

    virtual void setup(rclc_support_t *support, rclc_executor_t *executor);

    virtual void cleanup(rclc_executor_t *executor);

protected:
    const char *name;
    const char *ns;

    rcl_node_t node;
};

#endif //AVR_PCC_2023_NODE_HPP
