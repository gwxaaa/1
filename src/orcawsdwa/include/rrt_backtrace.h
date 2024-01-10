
#ifndef RRT_BACKTRACE_H
#define RRT_BACKTRACE_H

#include <vector>
#include <unordered_map>
namespace RVO
{
struct Node
{
    int id_;
    double x_;
    double y_;
    Node() : id_(-1), x_(0.0), y_(0.0) {}
    Node(int id, double x, double y) : id_(id), x_(x), y_(y) {}
};
std::vector<Node> backtracePath(const std::vector<Node>& returned_path, const std::unordered_map<int, int>& parent_map, int goal_id);
}
#endif // RRT_BACKTRACE_H