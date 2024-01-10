#include "rrt_backtrace.h"
#include <algorithm>
namespace RVO
{
std::vector<NodeWithParent> addParentInfoToNodes( std::vector<Node1>& returned_path) {
    std::vector<NodeWithParent> nodes_with_parent;
    //在设置的节点中，两两一组，也就是0和1，2和3，1和2之间没有关系
    nodes_with_parent.emplace_back(returned_path[0], Node1()); // 初始节点没有父节点
    for (size_t i = 1; i < returned_path.size()-1; i += 2) {
        nodes_with_parent.emplace_back(returned_path[i], returned_path[i - 1]);
      //  nodes_with_parent.emplace_back(returned_path[i + 1], returned_path[i]);
    }
    return nodes_with_parent;
}
//从末端开始回溯，将点和点之间进行连接----根据父节点的关系
std::vector<Node1> backtracePath( std::vector<NodeWithParent>& nodes_with_parent, int goal_id_) {

    std::vector<Node1> path;
    int current_id_ = goal_id_;
    //因为在前面的设置过程中，将-1设置为了节点会碰撞，所以现在的信息就是如果父节点为-1，那么就是空的
    //将起点设置为这个值，作为找到起点，循环结束
    while (current_id_ != -1) {
       // 找寻当前节点是否存在，然后找到后就找寻父节点
        const Node1& current_node = std::find_if(nodes_with_parent.begin(), nodes_with_parent.end(),
                                                [current_id_](const NodeWithParent& node) { return node.node.id_ == current_id_; })->node;

        path.push_back(current_node);

        auto it = std::find_if(nodes_with_parent.begin(), nodes_with_parent.end(),
                               [current_id_](const NodeWithParent& node) { return node.node.id_ == current_id_; });

        if (it != nodes_with_parent.end()) {
            current_id_ = it->parent.id_;
        } else {
            break;
        }
    }

    std::reverse(path.begin(), path.end());
    return path;
}

}