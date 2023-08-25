#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include "utils.hpp"

#ifndef AVR_PCC_2023_NODE_HPP
#define AVR_PCC_2023_NODE_HPP


class Node
{
public:
    Node(rclc_support_t *support, const char name[], const char ns[]);

protected:
    rcl_node_t node;
};


#endif //AVR_PCC_2023_NODE_HPP
