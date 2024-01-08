
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
    random_node.id_ = 0;
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
    double dist_threshold = 0.2;
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
      //将非model的障碍物转化为obstacles
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
