
#include <cmath>
#include <random>
#include "rrt.h"
#include <math.h>


namespace RVO
{

  RRT::RRT(const std::vector<gazebo_msgs::ModelState> &other_models_states, const geometry_msgs::Pose &current_pose,
           geometry_msgs::Pose goal_pose, int sample_num, double step, double size)
      : obstacles(convertToObstacles(other_models_states)),
        current_pose_(current_pose), goal_pose_(goal_pose),
        sample_num_(sample_num), step_(step), size_(size)
  {
  }

  Node RRT::plan()
  {
    // path.clear();
    // expand.clear();
    sample_list_.clear();
    double x = current_pose_.position.x;
    double y = current_pose_.position.y;
    Node start_node(0, x, y); //
    double g_x = goal_pose_.position.x;
    double g_y = goal_pose_.position.y;
    goal_ = Node(1, g_x, g_y);
    sample_list_.insert(std::make_pair(start_node.id_, start_node));
    // expand.push_back(start_node);
    int iteration = 0;
    while (iteration < sample_num_)
    {
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

      if (sample_list_.find(sample_node.id_) != sample_list_.end())
        continue;

      Node new_node = findNearestPoint(sample_list_, sample_node);
      if (new_node.id_ == -1)
        continue;
      else
      {
        sample_list_.insert(std::make_pair(new_node.id_, new_node));
        // expand.push_back(new_node);
      }

      if (checkGoal(new_node))
      {
        // path = convertClosedListToPath(new_node);
        return new_node;
      }
      iteration++;
    }
    return Node();
  }

  Node RRT::generateRandomNode(double size)
  {
    Node random_node;
    double min_val = -std::abs(size);
    double max_val = std::abs(size);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> x_dist(min_val, max_val);
    std::uniform_real_distribution<double> y_dist(min_val, max_val);

    random_node.x_ = x_dist(gen);
    random_node.y_ = y_dist(gen);

    return random_node;
  }

  Node RRT::findNearestPoint(std::unordered_map<int, Node> &list, const Node &random_node)
  {
    Node nearest_node;
    double min_dist = std::numeric_limits<double>::max();

    for (const auto &point : list)
    {
      double new_dist = calculateDistance(point.second, random_node);

      if (new_dist < min_dist)
      {
        min_dist = new_dist;
        nearest_node = point.second;
      }
    }
    if (min_dist > step_)
    {

      double theta = calculateAngle(random_node, nearest_node);
      nearest_node.x_ = nearest_node.x_ + (step_ * cos(theta));
      nearest_node.y_ = nearest_node.y_ + (step_ * sin(theta));
    }
    if (_isAnyObstacleInPath(new_node, nearest_node))
      nearest_node.id_ = -1;
    return nearest_node;
  }

  bool RRT::_isAnyObstacleInPath(const Node &n1, const Node &n2)
  {
    double dist_threshold = 0.25;
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
        Node goal(1, goal_.x_, goal_.y_);
        sample_list_.insert(std::make_pair(goal.id_, goal));
        return true;
      }
    }
    return true;
  }

  // std::vector<MyObstacle> RRT::convertToObstacles(const std::vector<gazebo_msgs::ModelState>& other_models_states) {
  //     std::vector<MyObstacle> obstacles;
  //     for (const auto& model_state : other_models_states) {
  //         MyObstacle obstacle;
  //         obstacle.name = model_state.model_name;
  //         obstacle.pose = model_state.pose;
  //         obstacles.push_back(obstacle);
  //     }
  //     return obstacles;
  // }

  std::vector<MyObstacle1> RRT::convertToObstacles(const std::vector<gazebo_msgs::ModelState> &other_models_states)
  {
    std::vector<MyObstacle1> obstacles;
    for (const auto &model_state : other_models_states)
    {
      if (model_state.model_name.find("model") != std::string::npos)
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

// #include <cmath>
// #include <random>
// #include "rrt.h"
// #include <math.h>
// //#include <math_helper.h>
// RRT::RRT(int sample_num, double max_dist)
//     : sample_num_(sample_num), max_dist_(max_dist)
// {}

// bool RRT::plan(const std::vector<Node> &obstacles, const Node &start, const Node &goal,
//                std::vector<Node> &path, std::vector<Node> &expand)
// {
//   path.clear();
//   expand.clear();
//   sample_list_.clear();
//   start_ = start;
//   goal_ = goal;

//   sample_list_.insert(std::make_pair(start.id_, start));
//   expand.push_back(start);

//   int iteration = 0;
//   while (iteration < sample_num_)
//   {
//     Node sample_node = _generateRandomNode();

//     bool isSampleNodeInObstacles = false;
//     // 检查障碍物
//     for (const auto &obstacle : obstacles)
//     {
//       if (obstacle.x_ == sample_node.x_ && obstacle.y_ == sample_node.y_)
//       {
//         isSampleNodeInObstacles = true;
//         break;
//       }
//     }

//     if (isSampleNodeInObstacles)
//       continue;

//     if (sample_list_.find(sample_node.id_) != sample_list_.end())
//       continue;

//     Node new_node = _findNearestPoint(sample_list_, sample_node);
//     if (new_node.id_ == -1)
//       continue;
//     else
//     {
//       sample_list_.insert(std::make_pair(new_node.id_, new_node));
//       expand.push_back(new_node);
//     }

//     if (_checkGoal(new_node))
//     {
//       // path = _convertClosedListToPath(new_node);
//       return true;
//     }
//     iteration++;
//   }
//   return false;
// }

// Node RRT::_generateRandomNode()
// {
//   // obtain a random number from hardware
//   std::random_device rd;
//   // seed the generator
//   std::mt19937 eng(rd());
//   // define the range
//   std::uniform_real_distribution<float> p(0, 1);

//   if (p(eng) > 0.05)
//   {
//     // Generate node within a certain range (modify as needed)
//     std::uniform_int_distribution<int> x_distribution(0, MAX_X);
//     std::uniform_int_distribution<int> y_distribution(0, MAX_Y);

//     // Generate random x and y coordinates
//     int x = x_distribution(eng);
//     int y = y_distribution(eng);

//      return Node(x, y, 0, 0, -1,0);
//   }
//   else
//      return Node(goal_.x_, goal_.y_, 0, 0, goal_.id_, 0);

// }
// Node RRT::_findNearestPoint(std::unordered_map<int, Node> &list, const Node &node)
// {
//   Node nearest_node, new_node(node);
//   double min_dist = std::numeric_limits<double>::max();

//   // 遍历节点列表，找到最近的节点
//   for (const auto &p : list)
//   {
//     // 计算距离
//     double new_dist = helper::dist(p.second, new_node);

//     // 更新最近节点
//     if (new_dist < min_dist)
//     {
//       nearest_node = p.second;
//       new_node.pid_ = nearest_node.id_;
//       new_node.g_ = new_dist + p.second.g_;
//       min_dist = new_dist;
//     }
//   }

//   // 距离大于阈值
//   if (min_dist > max_dist_)
//   {
//     // 连接采样节点和最近节点，然后将最近节点向采样节点移动，移动距离为 `max_distance`
//     double theta = helper::angle(nearest_node, new_node);
//     new_node.x_ = nearest_node.x_ + (int)(max_dist_ * cos(theta));
//     new_node.y_ = nearest_node.y_ + (int)(max_dist_ * sin(theta));
//     // 直接进行坐标到索引的转换
//     new_node.id_ = new_node.x_ + nx_ * new_node.y_; // 假设 nx_ 是用于计算索引的一个重要参数
//     new_node.g_ = max_dist_ + nearest_node.g_;
//   }

//   // 障碍物检查
//   if (_isAnyObstacleInPath(new_node, nearest_node))
//     new_node.id_ = -1;

//   return new_node;
// }
// bool RRT::_isAnyObstacleInPath(const Node &n1, const Node &n2)
// {
//   double theta = helper::angle(n1, n2);
//   double dist_ = helper::dist(n1, n2);

//   if (dist_ > max_dist_)
//     return true;
//   double resolution_ = 1;
//   int n_step = (int)(dist_ / resolution_);
//   for (int i = 0; i < n_step; i++)
//   {
//     float line_x = (float)n1.x_ + (float)(i * resolution_ * cos(theta));
//     float line_y = (float)n1.y_ + (float)(i * resolution_ * sin(theta));

//     // 直接在这里将坐标转换为索引
//     int index = (int)line_x + nx_ * (int)line_y; // 假设 nx_ 是用于计算索引的一个重要参数

//     // if (costs_[index] >= lethal_cost_ * factor_)
//     //   return true;
//   }
//   return false;
// }

// bool RRT::_checkGoal(const Node &new_node)
// {
//   auto dist_ = helper::dist(new_node, goal_);
//   if (dist_ > max_dist_)
//     return false;

//   if (!_isAnyObstacleInPath(new_node, goal_))
//   {
//     Node goal(goal_.x_, goal_.y_, dist_ + new_node.g_, 0, grid2Index(goal_.x_, goal_.y_), new_node.id_);
//     sample_list_.insert(std::make_pair(goal.id_, goal));
//     return true;
//   }
//   return false;
// }
