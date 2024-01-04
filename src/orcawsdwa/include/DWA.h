
#pragma once
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
namespace RVO
{
    struct MyObstacle
    {
      std::string name;
      geometry_msgs::Pose pose;
    };
  class DWAPlanner
  {
  private:
    geometry_msgs::Pose current_pose;
    geometry_msgs::Pose new_pose;
    geometry_msgs::Pose final_pose;
    geometry_msgs::Twist current_twist;
    geometry_msgs::Twist twist;
    geometry_msgs::Twist best_twist;
    double best_score;
    double max_linear_speed;
    double max_angular_speed;
    double min_linear_speed;
    double min_angular_speed;
    double best_final_score;
    double num;
    double time;
    double theta;
    double score;
    double some_threshold;
    float max_distance;
    // std::vector<geometry_msgs::Pose> obstacles;

    std::vector<MyObstacle> obstacles;

  public:
    DWAPlanner(const geometry_msgs::Pose &new_pose, const std::vector<gazebo_msgs::ModelState> &other_models_states,
               double max_linear_speed, double max_angular_speed,
               double time, double num, const geometry_msgs::Pose &current_pose, double theta);
    ~DWAPlanner();

    // 主要函数，用于计算并返回最佳速度
    const geometry_msgs::Twist &FindBestTwist(const geometry_msgs::Pose &current_pose);
    geometry_msgs::Pose GetFinalPose() const
    {
      return final_pose;
    }
    double GetBestScore() const
    {
      return best_score;
    }

  private:
    // 采样，速度组合
    // 设置参数
    std::vector<geometry_msgs::Twist> GenerateTwists();
    // 生成轨迹，
    geometry_msgs::Pose PredictPose(const geometry_msgs::Pose &current_pose, const geometry_msgs::Twist &twist, double time);
    // 计算碰撞惩罚
    double CalculateCollision(const geometry_msgs::Pose &final_pose);
    // 计算距离
    double CalculateDistance(const geometry_msgs::Pose &final_pose);
    // 计算计算角度，也就是朝向的区别---借助模型此刻的四元数信息
    double CalculateyawDistance(const geometry_msgs::Twist twist, const geometry_msgs::Pose &final_pose, double theta);
    double FindMaxDistance(const std::vector<geometry_msgs::Twist> &twist_vector,
                           const geometry_msgs::Pose &current_pose);
    // 评估速度和角速度组合的得分----根据得分得到最佳速度
    double CalculateScore(const geometry_msgs::Twist &twist);
    std::vector<MyObstacle> convertToObstacles(const std::vector<gazebo_msgs::ModelState> &other_models_states);
    // 在评价的参数内设置、相关的归一化，然后所得到的输出，就是【0，1】
    // 寻找最大的距离是为了将距离设置
    // 距离包含位置距离和角度距离，角度距离就可以粗略的设置处以2M——PI;但位置距离就不好设置，人为设置误差较大
  };
}
