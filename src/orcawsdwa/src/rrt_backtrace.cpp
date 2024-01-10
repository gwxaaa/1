
#include "rrt_backtrace.h"
#include <algorithm>
namespace RVO
{


std::vector<Node> backtracePath(const std::vector<Node>& returned_path, const std::unordered_map<int, int>& parent_map, int goal_id) {
    std::vector<Node> path;
    int current_id = goal_id; // 从目标节点开始回溯

    while (current_id != -1) {
        const Node& current_node = returned_path[current_id]; // 获取当前节点信息
        path.push_back(current_node); // 将当前节点添加到路径中
        auto it = parent_map.find(current_id); // 查找当前节点的父节点索引
        if (it != parent_map.end()) {
            current_id = it->second; // 更新为父节点索引，继续向前回溯
        } else {
            break; // 没有找到父节点索引，到达起始节点，结束回溯
        }
    }
    std::reverse(path.begin(), path.end()); // 反转路径，使其从起始节点到目标节点
    return path; // 返回回溯得到的路径
}
}