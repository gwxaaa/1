#ifndef RRT_H
#define RRT_H

#include <vector>
#include <unordered_map>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>

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

  struct MyObstacle1
  {
    std::string name;
    geometry_msgs::Pose pose;
  };

  class RRT
  {
  public:
    RRT(const std::vector<gazebo_msgs::ModelState> &other_models_states, const geometry_msgs::Pose &current_pose,
        geometry_msgs::Pose goal_pose, int sample_num, double step, double size);

    Node plan();

  private:
    // 其他私有成员函数和变量
    std::unordered_map<int, Node> sample_list_;
    geometry_msgs::Pose goal_pose_;
    geometry_msgs::Pose current_pose_;
    std::vector<MyObstacle1> obstacles;
    double sample_num_;
    double step_;
    double size_;
    Node new_node;
    Node goal_;
    Node generateRandomNode(double size);
    Node findNearestPoint(std::unordered_map<int, Node> &list, const Node &random_node);
    bool _isAnyObstacleInPath(const Node &n1, const Node &n2);
    bool checkGoal(const Node &new_node);
    std::vector<MyObstacle1> convertToObstacles(const std::vector<gazebo_msgs::ModelState> &other_models_states);
    double calculateAngle(const Node &point1, const Node &point2);
    double calculateDistance(const Node &p1, const Node &p2);
    double distanceToSegment(double x, double y, double x1, double y1, double x2, double y2);
  };

} // namespace RVO

#endif // RRT_H

// #ifndef RRT_H
// #define RRT_H

// #include <vector>
// #include <unordered_map>
// // #include "math_helper.h"
// //

// struct Node
// {
//   int x_; // 节点 x 坐标
//   int y_; // 节点 y 坐标
//   int g_;
//   int pid_;
//   int id_;   // 节点标识符
//   int cost_; // 节点代价值
//    // Node(int x, int y, int g, int pid, int id, int cost) : x_(x), y_(y), g_(g), pid_(pid), id_(id), cost_(cost) {}
// };

// struct Obstacle
// {
//   int x_, y_;

// };

// class RRT
// {
// public:
//   // 构造函数
//   RRT(int sample_num, double max_dist);

//   // 规划函数
//   bool plan(const std::vector<Node> &obstacles, const Node &start, const Node &goal,
//             std::vector<Node> &path, std::vector<Node> &expand);

// private:
//   // 寻找最近节点
//   Node _findNearestPoint(std::unordered_map<int, Node> &list, const Node &node);

//   // 检查路径中是否有障碍物
//   bool _isAnyObstacleInPath(const Node &n1, const Node &n2);

//   // 生成随机节点
//   Node _generateRandomNode();

//   // 检查目标是否可达
//   bool _checkGoal(const Node &new_node);
//   // 其他需要的函数或变量可在此添加

// private:
//   Node start_, goal_;                         // 起点和目标节点的副本
//   std::unordered_map<int, Node> sample_list_; // 样本节点集合
//   int sample_num_;                            // 最大样本数量
//   double max_dist_;
//   std::vector<Obstacle *> getObstacleNeighbors() const; // 最大距离阈值
//   double MAX_X;
//   double MAX_Y;
//   int nx_;
// };

// #endif // RRT_H