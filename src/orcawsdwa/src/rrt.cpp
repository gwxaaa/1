
#include <cmath>
#include <random>
#include "rrt.h"
#include <math.h>

namespace RVO
{

  RRT::RRT(const std::vector<gazebo_msgs::ModelState> &other_models_states, geometry_msgs::Pose &current_pose,
           geometry_msgs::Pose goal_pose, double sample_num, double step, double size_)
      : obstacles(convertToObstacles(other_models_states)),
        current_pose_(current_pose), goal_pose_(goal_pose),
        sample_num(sample_num), step_(step), size_(size_)
  {
  }
  std::vector<Node> RRT::plan()
  {
    std::vector<Node> path;
    path.clear();
    sample_list_.clear();
    double x = current_pose_.position.x;
    double y = current_pose_.position.y;
    Node start_node(-1, x, y); //
    double g_x = goal_pose_.position.x;
    double g_y = goal_pose_.position.y;
    goal_ = Node(-2, g_x, g_y);
    sample_list_.insert(std::make_pair(start_node.id_, start_node));
    // path.push_back(start_node);
    int iteration = 0;
    while (iteration < sample_num)
    {
      std::cout << "S iteration: (" << iteration << std::endl;
      Node sample_node = generateRandomNode(size_);
      bool isSampleNodeInObstacles = false;
      for (const auto &obstacle : obstacles)
      {
        if (obstacle.pose.position.x == sample_node.x_ && obstacle.pose.position.y == sample_node.y_)
        {
          isSampleNodeInObstacles = true;
          break;
        }
      }
      if (isSampleNodeInObstacles)
        continue;
      NodePair result = findNearestPoint(sample_list_, sample_node);
      if (result.random_node.id_ == -1)
        continue;
      else
      {
        sample_list_.insert(std::make_pair(result.random_node.id_, result.random_node));
        // 如果不放置这个，那么就会出现节点和这个不同
        path.push_back(result.nearest_node);
        path.push_back(result.random_node);
        // 对于是否到达终点，且这个时候的节点和终点之间连线是否会碰撞，都进行设置，如果到达终点且这两点不会碰撞，
        // 那么就会将这个点和终点连接，认为到达终点，如果不满足条件就会将这个舍弃找下一个节点
        if (checkGoal(result.random_node))
        {
          path.push_back(result.random_node); // Add the goal node to the path
          path.push_back(goal_);              // 将终点方进来，保证最后可以到达终点位置
          return path;
        }
        else
        {
          // 将靠的近一点的节点放进去
          // if (calculateDistance(result.random_node, goal_) < calculateDistance(path.back(), goal_))
          // {
          //   path.pop_back();
          // sample_list_.insert(std::make_pair(new_node.id_, new_node));
          // path.push_back(new_node);
          // return path;
          iteration++;
          continue;
          // }
        }
      }
      // iteration++;
    }
    return path;
  }

  Node RRT::generateRandomNode(double size_)
  {
    Node random_node;
    double min_val = -std::abs(size_);
    double max_val = std::abs(size_);
    std::random_device rd;
    std::mt19937 gen(rd());
    double center_x = (current_pose_.position.x + goal_pose_.position.x) / 2.0;
    double center_y = (current_pose_.position.y + goal_pose_.position.y) / 2.0;
    std::normal_distribution<double> gauss_x(center_x, 1);
    std::normal_distribution<double> gauss_y(center_y, 1);
    random_node.x_ = gauss_x(gen);
    random_node.y_ = gauss_y(gen);
    // std::uniform_real_distribution<double> x_dist(min_val, max_val);
    // std::uniform_real_distribution<double> y_dist(min_val, max_val);
    // random_node.x_ = x_dist(gen);
    // random_node.y_ = y_dist(gen);

    std::uniform_int_distribution<int> distr(1, std::numeric_limits<int>::max()); // 1到INT_MAX之间的随机整数
    random_node.id_ = distr(gen);

    return random_node;
  }

  NodePair RRT::findNearestPoint(std::unordered_map<int, Node> &sample_list_, Node &random_node)
  {
    Node nearest_node;
    double min_dist = std::numeric_limits<double>::max();
    // Node RRT::findNearestPoint(std::unordered_map<int, Node> &sample_list_, Node &random_node)
    // {
    //   Node nearest_node;
    //   double min_dist = std::numeric_limits<double>::max();
    //   bool isNearestInvalid = false;
    // std::cout << "Sample Node Coordinate: (" << random_node.x_ << ", " << random_node.y_ << ")" << std::endl;
    for (const auto &point : sample_list_)
    {
      double new_dist = calculateDistance(point.second, random_node);
      if (new_dist < min_dist)
      {
        min_dist = new_dist;
        nearest_node = point.second;
        // nearest_node.id_ = 1;
      }
    }
    // 找到最近的节点，如果节点在步长以内，就不需要修改，如果在步长以外，那么就需要修改最近点
    if (min_dist > step_)
    {
      // if (_isAnyObstacleInPath(random_node, nearest_node))
      random_node.id_ = -1;
      // else
      // {
      //   double theta = calculateAngle(random_node, nearest_node);
      //   random_node.x_ = nearest_node.x_ + (step_ * cos(theta));
      //   random_node.y_ = nearest_node.y_ + (step_ * sin(theta));
      //   random_node.id_ = 1;
      // }
    }
    if (_isAnyObstacleInPath(random_node, nearest_node))
      random_node.id_ = -1;

    NodePair result;
    result.random_node = random_node;
    result.nearest_node = nearest_node;
    return result;
  }

  bool RRT::_isAnyObstacleInPath(const Node &n1, const Node &n2)
  {
    double dist_threshold = 0.5;
    double x1 = n1.x_;
    double y1 = n1.y_;
    double x2 = n2.x_;
    double y2 = n2.y_;
    for (const auto &obstacle : obstacles)
    {
      double obstacle_dist = distanceToSegment(obstacle.pose.position.x, obstacle.pose.position.y, x1, y1, x2, y2);
      if (obstacle_dist < dist_threshold)
      {
        return true;
      }
    }
    return false;
  }

  bool RRT::checkGoal(const Node &new_node)
  {
    auto dist_to_goal = calculateDistance(new_node, goal_);
    if (dist_to_goal <= step_)
    {
      if (!_isAnyObstacleInPath(new_node, goal_))
      {
        // Node goal(1, goal_.x_, goal_.y_);
        // sample_list_.insert(std::make_pair(goal.id_, goal));

        return true;
      }
    }
    return false;
  }

  std::vector<MyObstacle1> RRT::convertToObstacles(const std::vector<gazebo_msgs::ModelState> &other_models_states)
  {
    std::vector<MyObstacle1> obstacles;
    for (const auto &model_state : other_models_states)
    {
      // 将非model的障碍物转化为obstacles
      if (model_state.model_name.find("model") == std::string::npos)
      {
        MyObstacle1 obstacle;
        obstacle.name = model_state.model_name;
        obstacle.pose = model_state.pose;
        obstacles.push_back(obstacle);
      }
    }
    return obstacles;
  }
  double RRT::calculateAngle(const Node &point1, const Node &point2)
  {
    double dx = point2.x_ - point1.x_;
    double dy = point2.y_ - point1.y_;

    double angle = atan2(dy, dx);

    return angle;
  }

  double RRT::calculateDistance(const Node &p1, const Node &p2)
  {
    double dx = p1.x_ - p2.x_;
    double dy = p1.y_ - p2.y_;
    return std::sqrt(dx * dx + dy * dy);
  }

  double RRT::distanceToSegment(double x, double y, double x1, double y1, double x2, double y2)
  {
    double A = x - x1;
    double B = y - y1;
    double C = x2 - x1;
    double D = y2 - y1;

    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    double param = -1;

    if (len_sq != 0)
      param = dot / len_sq;

    double xx, yy;

    if (param < 0)
    {
      xx = x1;
      yy = y1;
    }
    else if (param > 1)
    {
      xx = x2;
      yy = y2;
    }
    else
    {
      xx = x1 + param * C;
      yy = y1 + param * D;
    }

    double dx = x - xx;
    double dy = y - yy;

    return sqrt(dx * dx + dy * dy);
  }

} // namespace RVO
