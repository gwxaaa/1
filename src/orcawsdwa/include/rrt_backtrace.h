// #ifndef RRT_BACKTRACE_H
// #define RRT_BACKTRACE_H

// #include <vector>
// namespace RVO
// {
//     class RRT;
//     struct Node1
//     {
//         int id_;
//         double x_;
//         double y_;
//         Node1() : id_(-1), x_(0.0), y_(0.0) {}
//         Node1(int id, double x, double y) : id_(id), x_(x), y_(y) {}
//         // Node1( const Node& node):id_(node.id_),x_(node.x_),y_(node.y_){}
//     };
//     struct NodeWithParent
//     {
//         Node1 node;
//         Node1 parent; // 存储父节点的完整信息，包括坐标等其他属性
//         NodeWithParent(const Node1 &node_, const Node1 &parent_ = Node1()) : node(node_), parent(parent_) {}
//     };

//     std::vector<NodeWithParent> addParentInfoToNodes(std::vector<Node1> &returned_path);
//     std::vector<Node1> backtracePath(std::vector<NodeWithParent> &nodes_with_parent, int goal_id_, std::vector<Node1> &returned_path);
//     Node1 moveAlongLine(const Node1 &start, const Node1 &end, double ratio);
//     std::vector<Node1> processNodes(std::vector<RVO::Node1> path);

// }
// #endif // RRT_BACKTRACE_H
#ifndef RRT_BACKTRACE_H
#define RRT_BACKTRACE_H

#include <vector>
#include <algorithm>
#include <iostream>

namespace RVO
{
    class RRTBacktrace
    {
    public:
        struct Node1
        {
            int id_;
            double x_;
            double y_;
            Node1() : id_(-1), x_(0.0), y_(0.0) {}
            Node1(int id, double x, double y) : id_(id), x_(x), y_(y) {}
        };

        struct NodeWithParent
        {
            Node1 node;
            Node1 parent;
            NodeWithParent(const Node1 &node_, const Node1 &parent_ = Node1()) : node(node_), parent(parent_) {}
        };

        static std::vector<NodeWithParent> addParentInfoToNodes(std::vector<Node1> &returned_path);
        static std::vector<Node1> backtracePath(std::vector<NodeWithParent> &nodes_with_parent, int goal_id_, std::vector<Node1> &returned_path);
        static Node1 moveAlongLine(const Node1 &start, const Node1 &end, double ratio);
        static std::vector<Node1> processNodes(std::vector<Node1> path);
    };
}

#endif // RRT_BACKTRACE_H
