#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
#include "std_msgs/String.h"
#include "ModelSubPub.h"
#include "Agent.h"
#include "Neighbor.h"
#include <cmath>
#include <KinematicModel.h>
#include "DWA.h"
#include "Vector2.h"
#include "Line.h"
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Path.h"
#include "rrt.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "rrt_backtrace.h"
#include <RRTmain.h>
namespace RVO
{
  ModelSubPub::ModelSubPub(const std::string &modelName, double time, gazebo_msgs::ModelState target_model_state,
                           geometry_msgs::Pose goal_pose, double maxSpeed_, double neighborDistance_, double timeHorizon_, double radius_, double num,
                           double max_linear_speed, double max_angular_speed, double sample_num, double step, double size_, double ratio)
      : modelName_(modelName),
        time(time),
        maxSpeed_(maxSpeed_),
        neighborDistance_(neighborDistance_),
        timeHorizon_(timeHorizon_),
        radius_(radius_),
        goal_pose(goal_pose),
        target_model_state(target_model_state),
        lastStoredNewVelocity(agentVelocity),
        num(num),
        sample_num(sample_num),
        step(step),
        size_(size_),
        ratio(ratio),
        max_linear_speed(max_linear_speed),
        max_angular_speed(max_angular_speed),
        newVelocities(1, Vector2(0, 0)),
        polygonVertices(polygonVertices)
  {
    // 初始化ROS节点
    ros::NodeHandle nh;
    target_model_ = modelName;
    model_states_sub_ = nh.subscribe("/gazebo/model_states", 1, &ModelSubPub::modelStatesCallback, this);
    model_states_pub_ = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
    pose_stamped_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/pose_stamped_topic", 10); // 新增的发布器
    path_pub_ = nh.advertise<nav_msgs::Path>("/path_topic", 10);
    node_publisher = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
  }
  // 回调函数，处理模型状态信息
  void ModelSubPub::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
  {

    other_models_states.clear();
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      if (msg->name[i] == target_model_)
      {
        // 存储特定目标模型的信
        // gazebo_msgs::ModelState target_model_;
        target_model_state.model_name = msg->name[i];
        target_model_state.pose = msg->pose[i];
        target_model_state.twist = msg->twist[i];
      }
      else if (msg->name[i] != "ground_plane")
      {
        // 存储其他模型的信息
        gazebo_msgs::ModelState other_model_state;
        other_model_state.model_name = msg->name[i];
        other_model_state.pose = msg->pose[i];
        other_model_state.twist = msg->twist[i];
        other_models_states.push_back(other_model_state);
      }
    }
    // 目标模型信息格式的转换，终点信息的转换
    std::string agentname = target_model_;
    agentpose = target_model_state.pose;
    agenttwist = target_model_state.twist;
    // 格式转化
    Vector2 agentPosition(agentpose.position.x, agentpose.position.y);
    double deltaTheta = agenttwist.angular.z * time;
    // 这个是这一次的转换角度
    double velocityX = agenttwist.linear.x * cos(deltaTheta);
    double velocityY = agenttwist.linear.x * sin(deltaTheta);
    Vector2 agentVelocity(velocityX, velocityY);
    // 这个是一直循环的
    // 将RRT算法的结果进行输出

    // 传入的是一个路径，判断机器人此刻的位置点和下一个目标点的距离，判断距离，然后将点的信息进行修改
    RVO::RRTmain myRRTinstance(modelName_, time, target_model_state, goal_pose,
                               sample_num, step, size_, ratio);
    std::vector<geometry_msgs::Pose> pathPoses = myRRTinstance.getFinalPathPoses();
    size_t current_target_index = 0;
    // 如果还有未达到的目标点
    if (current_target_index < pathPoses.size())
    {
      // 获取当前目标点的坐标
      Vector2 goalPosition(pathPoses[current_target_index + 1].position.x, pathPoses[current_target_index + 1].position.y);

      // 计算当前位置与目标点的距离
      double distance_to_target = std::sqrt(std::pow(agentpose.position.x - goalPosition.x(), 2) +
                                            std::pow(agentpose.position.y - goalPosition.y(), 2));

      // 如果距离小于某个阈值，切换到下一个目标点
      if (distance_to_target < 0.1) // 这里的0.1是阈值
      {
        ++current_target_index;
      }
    }
    else
    {
      ROS_INFO("Reached the final target!");
    }

    //  开始计算ORCA算法
    RVO::Neighbor neighborobject(*this);
    // // 获取计算后的邻居信息
    std::vector<RVO::Agent *> agentNeighbors_ = neighborobject.getAgentNeighbors();
    std::vector<RVO::Obstacle *> obstacleNeighbors_ = neighborobject.getObstacleNeighbors();
    RVO::Agent agent(agentPosition, agentVelocity, goalPosition, time, maxSpeed_,
                     neighborDistance_, timeHorizon_, other_models_states, radius_);
    RVO::Vector2 newVelocity = agent.computeNewVelocity(agentPosition, agentVelocity,
                                                        goalPosition, agentNeighbors_, obstacleNeighbors_, time);

    // 知道速度矢量，将位置计算出来，对位置进行对比。----
    if (std::isnan(newVelocity.x()) || std::isnan(newVelocity.y()))
    {
      new_pose.position.x = agentPosition.x();
      new_pose.position.y = agentPosition.y();
      std::cout << "New velocity contains NaN. Keeping original position." << std::endl;
    }
    else
    {
      new_pose.position.x = agentPosition.x() + newVelocity.x() * time;
      new_pose.position.y = agentPosition.y() + newVelocity.y() * time;
      std::cout << "Moved to new position: x=" << new_pose.position.x << ", y=" << new_pose.position.y << std::endl;
    }
    // 这里需要注意，最后朝向和ORCA结果不同，是机器人的朝向。
    final_pose = agentpose;
    double final_yaw = atan2(
        2.0 * (final_pose.orientation.z * final_pose.orientation.w + final_pose.orientation.x * final_pose.orientation.y),
        1.0 - 2.0 * (final_pose.orientation.y * final_pose.orientation.y +
                     final_pose.orientation.z * final_pose.orientation.z));
    double theta1 = atan2(newVelocity.y(), newVelocity.x());
    double theta = theta1 - final_yaw;
    // double theta = atan2(newVelocity.y(), newVelocity.x());

    RVO::DWAPlanner planner(new_pose, other_models_states, max_linear_speed, max_angular_speed, time, num, agentpose, theta);
    // 查找最佳速度，final_pose，最佳分数
    const geometry_msgs::Twist &best_twist = planner.FindBestTwist(agentpose);
    std::cout << " best_twist.linear.xx=" << best_twist.linear.x << ", y=" << best_twist.angular.z << std::endl;
    geometry_msgs::Pose final_pose;
    KinematicModel kinematic_model(agentpose, best_twist);
    final_pose = kinematic_model.calculateNewPosition(time);
    // std::cout << "reMoved to new position: x=" << final_pose.position.x << ", y=" << final_pose.position.y << std::endl;
    //   std::cout << "rfinal_yaw: x=" << final_yaw<< std::endl;
    // 发布信息
    gazebo_msgs::ModelState amodel_state;
    amodel_state.model_name = agentname;
    amodel_state.pose = final_pose;
    model_states_pub_.publish(amodel_state);
    newpose = final_pose;
    newposes.push_back(newpose);
    std::size_t size = newposes.size();
    // 发布pose信息
    geometry_msgs::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header.stamp = ros::Time::now(); // 使用当前时间作为时间戳
    pose_stamped_msg.header.frame_id = "map";
    pose_stamped_msg.pose.position.x = newpose.position.x;
    pose_stamped_msg.pose.position.y = newpose.position.y;
    pose_stamped_msg.pose.orientation = newpose.orientation;
    // 发布 geometry_msgs::PoseStamped 类型的消息
    pose_stamped_pub_.publish(pose_stamped_msg);
    // 发布path信息
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map"; // 设置路径消息的坐标系
    // 需要设置较多的信息
    for (int i = 0; i < newposes.size(); ++i)
    {
      // 添加路径点到路径消息中
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "map"; // 设置路径点的坐标系
      pose.pose = newposes[i];
      path_msg.poses.push_back(pose); // 将路径点添加到路径消息中
    }
    path_pub_.publish(path_msg); // 发布路径消息
    // ros::shutdown();
  }

  std::vector<gazebo_msgs::ModelState> ModelSubPub::getothermodels() const
  {
    return other_models_states;
  };

}
