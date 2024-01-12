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
#include "Vector2.h"
namespace RVO
{
  // RRTmain* myRRTinstance=nullptr;
  RRTmain::RRTmain(const std::string &modelName, double time, gazebo_msgs::ModelState target_model_state,
                   geometry_msgs::Pose goal_pose,
                   double sample_num, double step, double size_, double ratio)
      : modelName_(modelName),
        time(time),
        goal_pose(goal_pose),
        target_model_state(target_model_state),
        sample_num(sample_num),
        step(step),
        size_(size_),
        ratio(ratio),
        isFirstCalculation(true),
        shouldRecompute(true)

  {
    // 初始化ROS节点
    ros::NodeHandle nh;
    target_model_ = modelName;
    model_states_sub_ = nh.subscribe("/gazebo/model_states", 1, &RRTmain::modelStatesCallback1, this);
    model_states_pub_ = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
    pose_stamped_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/pose_stamped_topic", 10); // 新增的发布器
    path_pub_ = nh.advertise<nav_msgs::Path>("/path_topic", 10);
    node_publisher = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
  }
  // 回调函数，处理模型状态信息
  void RRTmain::modelStatesCallback1(const gazebo_msgs::ModelStates::ConstPtr &msg)
  {

    std::cout << "isFirstCalculation: " << isFirstCalculation << std::endl;
    std::cout << "shouldRecompute: " << shouldRecompute << std::endl;
    // if (!isFirstCalculation && !shouldRecompute)
    // {
    //   return;
    // }
    // 如果是第一次计算或者远离计算点，则将 shouldRecompute 设置为 true
    if (isFirstCalculation)
    {
      shouldRecompute = true;
      isFirstCalculation = false; // 第一次计算后更新标志
    }
    // 检查重新计算标志是否为 false，如果为 true，则跳过计算
    if (!shouldRecompute)
    {
      return;
    }
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

    std::string agentname = target_model_;
    agentpose = target_model_state.pose;
    agenttwist = target_model_state.twist;
    // 格式转化
    Vector2 agentPosition(agentpose.position.x, agentpose.position.y);
    // Vector2 goalPosition(goal_pose.position.x, goal_pose.position.y);
    //  创建 RRT 对象
    // 这个信息在前面的话就可以出现，但是在这个rrt后面就不
    RVO::RRT rrt(other_models_states, target_model_state.pose, goal_pose, sample_num, step, size_);
    // 运行 RRT 算法得到下一个可行的节点
    std::vector<Node> returned_path = rrt.plan();
    double number_of_points = returned_path.size();
    std::vector<RVO::RRTBacktrace::Node1> returned_path1;
    for (const auto &node : returned_path)
    {
      RVO::RRTBacktrace::Node1 node1(node.id_, node.x_, node.y_);
      returned_path1.push_back(node1);
    }
    RVO::RRTBacktrace rrtBacktrace;
    std::vector<RVO::RRTBacktrace::NodeWithParent> nodes_with_parent = rrtBacktrace.addParentInfoToNodes(returned_path1);
    int goal_id_ = -2;
    // 调用 backtracePath 函数
    std::vector<RVO::RRTBacktrace::Node1> path = rrtBacktrace.backtracePath(nodes_with_parent, goal_id_, returned_path1);
    //  double ratio = 0.1;
    adjusted_path = RVO::RRTBacktrace::processNodes(path, ratio);
    flag = false;
    //isFirstCalculation = false;
    shouldRecompute = false;
    std::cout << "1isFirstCalculation: " << isFirstCalculation << std::endl;
    std::cout << "1shouldRecompute: " << shouldRecompute << std::endl;
    double number_of_points1 = path.size();
    // for (const auto &node : path)
    // {
    //   std::cout << "111Node ID: " << node.id_ << ", x: " << node.x_ << ", y: " << node.y_ << std::endl;
    // }
    double number_of_points2 = adjusted_path.size();
    // for (const auto &node : adjusted_path)
    // {
    //   std::cout << "222Node ID: " << node.id_ << ", x: " << node.x_ << ", y: " << node.y_ << std::endl;
    // }
    ros::Rate rate(30);
    visualization_msgs::MarkerArray marker_array_msg;
    for (int i = 0; i < number_of_points - 1; i += 2)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "";
      marker.id = i;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = returned_path[i].x_;
      marker.pose.position.y = returned_path[i].y_;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.07; // Size of the sphere
      marker.scale.y = 0.07;
      marker.scale.z = 0.07;
      marker.color.r = 1.0; // Color: red
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0; // Alpha
      marker_array_msg.markers.push_back(marker);
      marker_array_pub.publish(marker_array_msg);
      rate.sleep(); // 暂停，等待指定的频率
      if (i < number_of_points - 1)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "";
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.03; // Line width
        marker.color.r = 1.0;  // Color: red
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0; // Alpha

        geometry_msgs::Point start_point, end_point;
        start_point.x = returned_path[i].x_;
        start_point.y = returned_path[i].y_;
        start_point.z = 0;
        end_point.x = returned_path[i + 1].x_;
        end_point.y = returned_path[i + 1].y_;
        end_point.z = 0;
        marker.points.push_back(start_point);
        marker.points.push_back(end_point);
        marker_array_msg.markers.push_back(marker);
        marker_array_pub.publish(marker_array_msg);
        rate.sleep(); // 暂停，等待指定的频率
      }
    }
    // 将起点和终点展示
    visualization_msgs::Marker first_marker;
    first_marker.header.frame_id = "map";
    first_marker.header.stamp = ros::Time::now();
    first_marker.ns = "point";
    first_marker.id = 100; // 为第一个点设置一个不同的ID，以便与其他点区分开来
    first_marker.type = visualization_msgs::Marker::SPHERE;
    first_marker.action = visualization_msgs::Marker::ADD;
    first_marker.pose.position.x = returned_path[0].x_; // 第一个点的 x 坐标
    first_marker.pose.position.y = returned_path[0].y_; // 第一个点的 y 坐标
    first_marker.pose.position.z = 0;
    first_marker.pose.orientation.x = 0.0;
    first_marker.pose.orientation.y = 0.0;
    first_marker.pose.orientation.z = 0.0;
    first_marker.pose.orientation.w = 1.0;
    first_marker.scale.x = 0.15; // 放大的尺寸
    first_marker.scale.y = 0.15;
    first_marker.scale.z = 0.15;
    first_marker.color.r = 1.0;
    first_marker.color.g = 0.0;
    first_marker.color.b = 0.0;
    first_marker.color.a = 1.0;
    marker_array_msg.markers.push_back(first_marker);
    visualization_msgs::Marker last_marker;
    last_marker.header.frame_id = "map";
    last_marker.header.stamp = ros::Time::now();
    last_marker.ns = "line";
    last_marker.id = 1001; // 为最后一个点设置一个不同的ID，以便与其他点区分开来
    last_marker.type = visualization_msgs::Marker::SPHERE;
    last_marker.action = visualization_msgs::Marker::ADD;
    last_marker.pose.position.x = returned_path[number_of_points - 1].x_; // 最后一个点的 x 坐标
    last_marker.pose.position.y = returned_path[number_of_points - 1].y_; // 最后一个点的 y 坐标
    last_marker.pose.position.z = 0;
    last_marker.pose.orientation.x = 0.0;
    last_marker.pose.orientation.y = 0.0;
    last_marker.pose.orientation.z = 0.0;
    last_marker.pose.orientation.w = 1.0;
    last_marker.scale.x = 0.15; // 放大的尺寸
    last_marker.scale.y = 0.15;
    last_marker.scale.z = 0.15;
    last_marker.color.r = 1.0;
    last_marker.color.g = 0.0;
    last_marker.color.b = 1.0;
    last_marker.color.a = 1.0;
    marker_array_msg.markers.push_back(last_marker);
    marker_array_pub.publish(marker_array_msg);

    visualization_msgs::Marker pathmarker;
    pathmarker.header.frame_id = "map";
    pathmarker.header.stamp = ros::Time::now();
    pathmarker.ns = "path";
    pathmarker.id = 0;
    pathmarker.type = visualization_msgs::Marker::LINE_STRIP;
    pathmarker.action = visualization_msgs::Marker::ADD;
    pathmarker.pose.orientation.w = 1.0;
    pathmarker.scale.x = 0.08; // Line width
    pathmarker.color.r = 0.0;  // Color: red
    pathmarker.color.g = 1.0;
    pathmarker.color.b = 1.0;
    pathmarker.color.a = 1.0; // Alpha
    for (int i = 0; i < number_of_points1; ++i)
    {
      geometry_msgs::Point point;
      point.x = path[i].x_;
      point.y = path[i].y_;
      point.z = 0;
      pathmarker.points.push_back(point);
    }
    marker_array_msg.markers.push_back(pathmarker);
    marker_array_pub.publish(marker_array_msg);
    rate.sleep(); // 暂停，等待指定的频率
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "1";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.04; // Line width
    marker.color.r = 0.0;  // Color: red
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0; // Alpha
    for (int i = 0; i < number_of_points2; ++i)
    {
      geometry_msgs::Point point;
      point.x = adjusted_path[i].x_;
      point.y = adjusted_path[i].y_;
      point.z = 0;
      marker.points.push_back(point);
    }
    marker_array_msg.markers.push_back(marker);
    marker_array_pub.publish(marker_array_msg);
    rate.sleep(); // 暂停，等待指定的频率
    // Vector2 goalPosition(adjusted_path[1].x_, adjusted_path[1].y_);
    // std::cout << "s goalPosition " << goalPosition.x() << ",  goalPosition " << goalPosition.y() << std::endl;
    // ros::shutdown();
  }

  std::vector<gazebo_msgs::ModelState> RRTmain::getothermodels1() const
  {
    return other_models_states;
  };
  std::vector<geometry_msgs::Pose> RRTmain::getFinalPathPoses() const
  {
    std::vector<geometry_msgs::Pose> finalPathPoses;

    for (const auto &node : adjusted_path)
    {
      geometry_msgs::Pose pose;
      pose.position.x = node.x_;
      pose.position.y = node.y_;
      pose.position.z = 0.0; // Assuming z-coordinate is always zero
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;

      finalPathPoses.push_back(pose);
    }
    return finalPathPoses;
  }
  // 示例函数，用于判断是否远离计算点
  // bool isFarFromCalculationPoint()
  // {

  //   return true;
  // }
}
