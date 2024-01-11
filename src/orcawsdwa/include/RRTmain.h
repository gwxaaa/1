#ifndef RRT_MAIN_H
#define RRT_MAIN_H
#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelState.h"
#include "std_msgs/String.h"
#include "Agent.h"
#include "Obstacle.h"
#include "Neighbor.h"
#include <KinematicModel.h>
#include "Line.h"
#include "Vector2.h"
#include <visualization_msgs/Marker.h>
#include "rrt_backtrace.h"
namespace RVO
{
  class RRT;
  class RRTmain
  {
  public:
    RRTmain(const std::string &modelName, double time, gazebo_msgs::ModelState target_model_state, geometry_msgs::Pose goal_pose,
            double sample_num, double step, double size_, double ratio);
    // 回调函数，处理模型状态信息
    void modelStatesCallback1(const gazebo_msgs::ModelStates::ConstPtr &msg);
    std::vector<gazebo_msgs::ModelState> getothermodels1() const;
    std::vector<geometry_msgs::Pose> getFinalPathPoses() const;
    double radius_; // 避障半径
    std::vector<RVO::RRTBacktrace::Node1> adjusted_path;


  private:
    ros::NodeHandle nh;
    ros::Subscriber model_states_sub_;
    ros::Publisher model_states_pub_;
    ros::Publisher node_publisher;
    ros::Publisher marker_array_pub;
    std::string target_model_; // 用于存储目标模型的名称
    double time;
    double num;
    double sample_num;
    double step;
    double size_;
    double ratio;
    geometry_msgs::Pose target_model_pose;
    double new_velocity;
    geometry_msgs::Pose agentpose;
    geometry_msgs::Twist agenttwist;
    geometry_msgs::Pose goal_pose;
    std::vector<gazebo_msgs::ModelState> other_models_states;
    std::string modelName_;
    double time_;
    gazebo_msgs::ModelState target_model_state;
    Vector2 agentPosition;
    Vector2 agentVelocity;
    Vector2 goalPosition;
    geometry_msgs::Pose new_pose;
    geometry_msgs::Twist new_twist;
    geometry_msgs::Pose final_pose;
    std::vector<geometry_msgs::Pose> obstacle_poses;
    Vector2 currentPosition;
    ros::Publisher pose_stamped_pub_;
    ros::Publisher path_pub_;
    std::vector<geometry_msgs::Pose> newposes;
    geometry_msgs::Pose newpose;
    geometry_msgs::Point p;
    // Node new_goal_node;
    //  std::vector<Obstacle> obstacles;
    bool flag;
  };
} // namespace RVO
#endif // RRT_MAIN_H
