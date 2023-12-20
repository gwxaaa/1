#include "DWA.h"
#include <cmath>
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <KinematicModel.h>
namespace RVO
{
  DWAPlanner::DWAPlanner(const geometry_msgs::Pose &new_pose, const std::vector<geometry_msgs::Pose> &obstacles,
                         double max_linear_speed, double max_angular_speed, double time, double num,
                         const geometry_msgs::Pose &current_pose, double theta_)
      : new_pose(new_pose), obstacles(obstacles), max_distance(1.0), some_threshold(some_threshold),
        max_linear_speed(max_linear_speed), max_angular_speed(max_angular_speed), time(time), num(num),
        theta(theta)
  {
  }
  DWAPlanner::~DWAPlanner()
  {
  }
  const geometry_msgs::Twist &DWAPlanner::FindBestTwist(const geometry_msgs::Pose &current_pose)
  {
    // 初始化最佳速度和最佳分数
    best_twist = current_twist;
    double best_score = -std::numeric_limits<double>::infinity();
    std::vector<geometry_msgs::Twist> twist_combinations = GenerateTwists();
    double max_distance = FindMaxDistance(twist_combinations, current_pose);
    // std::cout << "Max Distance: " << max_distance << std::endl;
    // 创建一个临时变量来存储最佳速度
    geometry_msgs::Twist best_temp_twist = best_twist;
    geometry_msgs::Pose best_final_pose = final_pose;
    double best_final_score = -0.1;
    // 循环评估速度组合
    for (const auto &current_twist : twist_combinations)
    {
      // 预测最终姿态
      final_pose = PredictPose(current_pose, current_twist, time);
      // 检查是否接近目标点
      double distance_to_target = CalculateDistance(final_pose);
      // 检查碰撞
      double collision = CalculateCollision(final_pose);
      double yawdistance = CalculateyawDistance(twist, final_pose, theta);
      // 如果当前组合的分数更高，更新最佳分数和速度角速度
      double score = CalculateScore(current_twist);
      // std::cout << "Score1: " << best_final_score << std::endl;
      // std::cout << " - current_twist: x = " << current_twist.linear.x << ", y = " << current_twist.angular.z << std::endl;
      // std::cout << "Score: " << score << std::endl;
      // std::cout << "2rfinal_yaw: x=" << final_yaw << std::endl;
      if (score > best_final_score)
      {
        best_final_score = score;
        best_temp_twist = current_twist;
        best_final_pose = final_pose;
      }
    }
    // 更新最佳速度
    best_twist = best_temp_twist;
    final_pose = best_final_pose;
    return best_twist;
  }

  std::vector<geometry_msgs::Twist> DWAPlanner::GenerateTwists()
  {
    std::vector<geometry_msgs::Twist> twist_combinations; // 存储多组速度角速度组合
    double linear_speed_increment = (2 * max_linear_speed) / static_cast<double>(num - 1);
    double angular_speed_increment = (2 * max_angular_speed) / static_cast<double>(num - 1);
    for (int i = 0; i < num; i++)
    {
      for (int j = 0; j < num; j++)
      {
        // 创建速度组合
        geometry_msgs::Twist current_twist;
        current_twist.linear.x = -max_linear_speed + i * linear_speed_increment;
        current_twist.angular.z = -max_angular_speed + j * angular_speed_increment;
        twist_combinations.push_back(current_twist); // 存储当前组合
      }
    }

    return twist_combinations;
  }

  geometry_msgs::Pose DWAPlanner::PredictPose(const geometry_msgs::Pose &current_pose, const geometry_msgs::Twist &twist,
                                              double time)
  {
    KinematicModel kinematic_model(current_pose, twist);
    geometry_msgs::Pose final_pose = kinematic_model.calculateNewPosition(time);
    return final_pose;
  }

  double DWAPlanner::CalculateCollision(const geometry_msgs::Pose &final_pose)
  {
    double min_collision_distance = 0.4;       // 最小碰撞距离
    double avoidance_distance = 1.5;           // 避障范围起始距离
    double collision_distance_threshold = 1.5; // 碰撞阈值

    double distance_to_obstacle = std::numeric_limits<double>::max();

    for (const geometry_msgs::Pose &obstacle : obstacles)
    {
      double dx = final_pose.position.x - obstacle.position.x;
      double dy = final_pose.position.y - obstacle.position.y;
      double distance = std::sqrt(dx * dx + dy * dy);

      if (distance < distance_to_obstacle)
      {
        distance_to_obstacle = distance;
      }
    }

    if (distance_to_obstacle < min_collision_distance)
    {
      // 如果距离障碍物小于最小碰撞距离，直接返回0分表示碰撞
      return 0.0;
    }
    else if (distance_to_obstacle <= avoidance_distance)
    {
      // 如果距离障碍物在避障范围内，根据距离插值计算分数
      double score = 1.0 - (distance_to_obstacle - min_collision_distance) /
                               (avoidance_distance - min_collision_distance);
      return score;
    }
    else if (distance_to_obstacle <= collision_distance_threshold)
    {
      // 如果距离障碍物在碰撞阈值内，给予一定的分数
      double score = 0.5; // 你可以根据需要调整这个分数
      return score;
    }
    else
    {
      // 如果没有遇到障碍物或者距离障碍物超出碰撞阈值，返回最高分数
      return 1.0;
    }
  }

  double DWAPlanner::FindMaxDistance(const std::vector<geometry_msgs::Twist> &twist_vector,
                                     const geometry_msgs::Pose &current_pose)
  {
    double max_distance = 1;
    // 遍历每组速度角速度，计算对应的距离
    for (const auto &twist : twist_vector)
    {
      geometry_msgs::Pose final_pose = PredictPose(current_pose, twist, time);
      double dx = final_pose.position.x - new_pose.position.x;
      double dy = final_pose.position.y - new_pose.position.y;
      double distance = std::sqrt(dx * dx + dy * dy);
      // 更新最大距离
      if (distance > max_distance)
      {
        max_distance = distance;
      }
    }
    return max_distance;
  }
  // 在这里需要计算ORCA速度矢量到达的位置点

  double DWAPlanner::CalculateDistance(const geometry_msgs::Pose &final_pose)
  {
    // 计算位置之间的距离
    double dx = final_pose.position.x - new_pose.position.x;
    double dy = final_pose.position.y - new_pose.position.y;
    double distance_position = std::sqrt(dx * dx + dy * dy);
    // 将距离归一化为【0，1】
    // 最大距离引入
    double normalized_distance_distance = distance_position / (max_distance * 1);
    double distance = normalized_distance_distance;
    return distance;
  }
  double DWAPlanner::CalculateyawDistance(const geometry_msgs::Twist twist, const geometry_msgs::Pose &current_pose, double theta)
  {
    // 计算朝向之间的角度差异
    // double target_yaw = atan2(2.0 * (new_pose.orientation.z * new_pose.orientation.w +
    //                                  new_pose.orientation.x * new_pose.orientation.y),
    //                           1.0 - 2.0 * (new_pose.orientation.y * new_pose.orientation.y +
    //                                        new_pose.orientation.z * new_pose.orientation.z));
    double target_yaw = theta;
    // 对于ORCA算法，所得到的角度是与坐标系的结果，那么需要与整体的朝向进行对比。
    // fanal_yaw就是最后的朝向，也就是初始模型朝向加上转换角度得到的结果。
    double final_yaw = twist.angular.z * time;
    // double final_yaw = atan2(
    //     2.0 * (final_pose.orientation.z * final_pose.orientation.w + final_pose.orientation.x * final_pose.orientation.y),
    //     1.0 - 2.0 * (final_pose.orientation.y * final_pose.orientation.y +
    //                  final_pose.orientation.z * final_pose.orientation.z));
    // geometry_msgs::Pose modelPose;
    // modelPose = current_pose;
    // double initial_yaw = atan2(2.0 * (modelPose.orientation.w * modelPose.orientation.z + modelPose.orientation.x * modelPose.orientation.y),
    //                            1.0 - 2.0 * (modelPose.orientation.y * modelPose.orientation.y + modelPose.orientation.z * modelPose.orientation.z));

    // double final_yaw= initial_yaw+final_yaw1;
    double yaw_difference1 = std::abs(target_yaw - final_yaw);
    // double yaw_difference2 = ((yaw_difference1 > 0 && final_yaw > 0) || (yaw_difference1 < 0 && final_yaw < 0)) ? -0.5 : 0.1;
    // 将角度差归一化为 [0, 1]
    double yaw_difference2 = 0;
    double normalized_angle_distance = yaw_difference1 / 2 * M_PI + yaw_difference2;
    double angle = normalized_angle_distance;
    return angle;
  }

  double DWAPlanner::CalculateScore(const geometry_msgs::Twist &twist)
  {
    // 计算距离目标的距离
    double distance = CalculateDistance(final_pose);
    // 计算碰撞惩罚
    double collision = CalculateCollision(final_pose);
    double yaw = CalculateyawDistance(twist, final_pose, theta);
    // if (collision<1)
    // {
    //   distance=-distance;
    // }
    // 设置权重
    double distance_weight = -5;
    double yaw_weight = -4;
    double collision_weight = 1000;
    // 计算距离分数和碰撞分数
    double distance_score = distance_weight * distance;
    double yaw_score = yaw_weight * yaw;
    double collision_score = collision_weight * collision;
    // 计算总分数
    double score = collision_weight + distance_score + yaw_score;
    // 如果当前分数比最佳分数高，更新最佳分数和对应的 twist 值
    if (score > best_final_score)
    {
      best_final_score = score;
      best_twist = twist;
    }
    return score;
  }
}