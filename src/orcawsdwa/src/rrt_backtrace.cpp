

#include "rrt_backtrace.h"
#include <algorithm>
#include <iostream>
namespace RVO
{
    std::vector<RVO::RRTBacktrace::NodeWithParent> RVO::RRTBacktrace::addParentInfoToNodes(std::vector<Node1> &returned_path)
    {
        std::vector<NodeWithParent> nodes_with_parent;
        // 在设置的节点中，两两一组，也就是0和1，2和3，1和2之间没有关系
        nodes_with_parent.emplace_back(returned_path[0], Node1()); // 初始节点没有父节点
        for (size_t i = 1; i < returned_path.size(); i += 2)
        {
            nodes_with_parent.emplace_back(returned_path[i], returned_path[i - 1]);
            // 判断是否找到父节点
            std::cout << "son Node ID: " << returned_path[i].id_ << ", Parent ID: " << returned_path[i - 1].id_ << std::endl;
            //  nodes_with_parent.emplace_back(returned_path[i + 1], returned_path[i]);
        }
        return nodes_with_parent;
    }

    std::vector<RVO::RRTBacktrace::Node1> RVO::RRTBacktrace::backtracePath(std::vector<NodeWithParent> &nodes_with_parent, int goal_id_, std::vector<Node1> &returned_path)
    {
        std::vector<Node1> path;
        int current_id_ = goal_id_;
        // 因为在前面的设置过程中，将-1设置为了节点会碰撞，所以现在的信息就是如果父节点为-1，那么就是空的
        // 将起点设置为这个值，作为找到起点，循环结束
        while (current_id_ != -1)
        {
            // 找寻当前节点是否存在，然后找到后就找寻父节点
            const Node1 &current_node = std::find_if(nodes_with_parent.begin(), nodes_with_parent.end(),
                                                     [current_id_](const NodeWithParent &node)
                                                     { return node.node.id_ == current_id_; })
                                            ->node;
            path.push_back(current_node);

            auto it = std::find_if(nodes_with_parent.begin(), nodes_with_parent.end(),
                                   [current_id_](const NodeWithParent &node)
                                   { return node.node.id_ == current_id_; });
            if (it != nodes_with_parent.end())
            {
                current_id_ = it->parent.id_;
            }
            else
            {
                break;
            }
        }
        if (current_id_ == -1)
        {
            path.push_back(returned_path[0]);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    RVO::RRTBacktrace::Node1 RVO::RRTBacktrace::moveAlongLine(const Node1 &start, const Node1 &end, double ratio)
    {
      // 计算方向向量
      double deltaX = end.x_ - start.x_;
      double deltaY = end.y_ - start.y_;
      // 计算移动后的坐标
      double newX = start.x_ + ratio * deltaX;
      double newY = start.y_ + ratio * deltaY;
      return Node1(1,newX, newY);
    }

    std::vector<RVO::RRTBacktrace::Node1> RVO::RRTBacktrace::processNodes(std::vector<Node1> path)
    {
      std::vector<Node1> processedNodes;
      // 处理第一个节点
      processedNodes.push_back(path[0]);
      // 遍历处理每两个相邻的节点
      for (size_t i = 0; i < path.size() - 1; ++i)
      {
        // 移动第二个节点
        Node1 adjustedNode = moveAlongLine(path[i], path[i + 1], 0.1);
        processedNodes.push_back(adjustedNode); // 调整后的节点
      }
      // 添加最后一个节点
      processedNodes.push_back(path.back());
      return processedNodes;
    }
}


// #include "rrt_backtrace.h"
// #include <algorithm>
// #include <iostream>
// namespace RVO
// {
//     std::vector<NodeWithParent> addParentInfoToNodes(std::vector<Node1> &returned_path)
//     {
//         std::vector<NodeWithParent> nodes_with_parent;
//         // 在设置的节点中，两两一组，也就是0和1，2和3，1和2之间没有关系
//         nodes_with_parent.emplace_back(returned_path[0], Node1()); // 初始节点没有父节点
//         for (size_t i = 1; i < returned_path.size(); i += 2)
//         {
//             nodes_with_parent.emplace_back(returned_path[i], returned_path[i - 1]);
//             // 判断是否找到父节点
//             std::cout << "son Node ID: " << returned_path[i].id_ << ", Parent ID: " << returned_path[i - 1].id_ << std::endl;
//             //  nodes_with_parent.emplace_back(returned_path[i + 1], returned_path[i]);
//         }
//         return nodes_with_parent;
//     }
//     // 从末端开始回溯，将点和点之间进行连接----根据父节点的关系
//     std::vector<Node1> backtracePath(std::vector<NodeWithParent> &nodes_with_parent, int goal_id_, std::vector<Node1> &returned_path)
//     {
//         std::vector<Node1> path;
//         int current_id_ = goal_id_;
//         // 因为在前面的设置过程中，将-1设置为了节点会碰撞，所以现在的信息就是如果父节点为-1，那么就是空的
//         // 将起点设置为这个值，作为找到起点，循环结束
//         while (current_id_ != -1)
//         {
//             // 找寻当前节点是否存在，然后找到后就找寻父节点
//             const Node1 &current_node = std::find_if(nodes_with_parent.begin(), nodes_with_parent.end(),
//                                                      [current_id_](const NodeWithParent &node)
//                                                      { return node.node.id_ == current_id_; })
//                                             ->node;
//             path.push_back(current_node);

//             auto it = std::find_if(nodes_with_parent.begin(), nodes_with_parent.end(),
//                                    [current_id_](const NodeWithParent &node)
//                                    { return node.node.id_ == current_id_; });
//             if (it != nodes_with_parent.end())
//             {
//                 current_id_ = it->parent.id_;
//             }
//             else
//             {
//                 break;
//             }
//         }
//         if (current_id_ == -1)
//         {
//             path.push_back(returned_path[0]);
//         }
//         std::reverse(path.begin(), path.end());
//         return path;
//     }
//     Node1 moveAlongLine(const Node1 &start, const Node1 &end, double ratio)
//     {
//       // 计算方向向量
//       double deltaX = end.x_ - start.x_;
//       double deltaY = end.y_ - start.y_;
//       // 计算移动后的坐标
//       double newX = start.x_ + ratio * deltaX;
//       double newY = start.y_ + ratio * deltaY;
//       return Node1(1,newX, newY);
//     }
//     // 对节点进行处理的函数
//     std::vector<Node1> processNodes(std::vector<RVO::Node1> path)
//     {
//       std::vector<Node1> processedNodes;
//       // 处理第一个节点
//       processedNodes.push_back(path[0]);
//       // 遍历处理每两个相邻的节点
//       for (size_t i = 0; i < path.size() - 1; ++i)
//       {
//         // 移动第二个节点
//         Node1 adjustedNode = moveAlongLine(path[i], path[i + 1], 0.1);
//         processedNodes.push_back(adjustedNode); // 调整后的节点
//       }
//       // 添加最后一个节点
//       processedNodes.push_back(path.back());
//       return processedNodes;
//     }
// }
