#pragma once

#include <kinova_msgs/JointVelocity.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <memory>
#include <mutex>

#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_comm.h"

// Forward declaration(s)
class KinovaAnglesState;
//---------------------------------------------------------------------------------------------------------------------
class PidController
{
public:
  PidController(ros::NodeHandle nh, kinova::KinovaComm* const comm,
                const trajectory_msgs::JointTrajectoryConstPtr& msg);
  ~PidController();

  // Should be triggered every 1/100 s
  void controlCallback(const ros::TimerEvent&);

  bool isTrajectoryFinished() const;
private:
  /* Methods */
  // TODO: Reset control parameters per waypoint? --> probably not
  void updateCurrentWaypoint();
  void updateCurrentJointStates();

  inline void updateUnalteredVelocityMessage();

  // TODO: Bottleneck?
  void getGoalJointPositions(kinova::KinovaAngles& goal,
                             const ros::Time& timestamp) const;

  void finishTrajectory();

  /* Attributes */
  ros::NodeHandle nh_;
  kinova::KinovaComm* const comm_;
  ros::Publisher pub_joint_velocity_;
  ros::Timer control_callback_timer_;
  ros::Time control_loop_start_;

  ros::Time trajectory_starting_time_;
  std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_waypoints_;

  // Processing state of current trajectory corresponding to current goal
  // waypoint
  unsigned int current_waypoint_index_ = 0u;
  ros::Time current_time_limit_;
  kinova_msgs::JointVelocity unaltered_velocity_msg_;
  std::vector<double> vector_unaltered_velocity_msg_;

  // Read arm state
  std::unique_ptr<KinovaAnglesState> current_state_;
  std::unique_ptr<KinovaAnglesState> previous_state_;

  // TODO: Set fixed coefficients via ros params for testing and determining how
  // to best set them
  const int num_joints_ = 6;
  std::vector<double> Kp = std::vector<double>(num_joints_, 0.5);
  std::vector<double> Kd = std::vector<double>(num_joints_, 0.3);
  std::vector<double> Ki = std::vector<double>(num_joints_, 0.2);

  std::vector<double> error_deltas_ = std::vector<double>(num_joints_, 0.0);
  std::vector<double> error_sums_ = std::vector<double>(num_joints_, 0.0);

  // Sent control commands sent (TODO: Useful or not?)
  kinova_msgs::JointVelocity current_velocity_msg_;
  kinova_msgs::JointVelocity previous_velocity_msg_;

  bool finished_trajectory_ = false;
  mutable std::mutex finished_trajectory_mutex_;


  // DEBUG
  std::vector<ros::Time> control_callback_start_timestamps_;
  std::vector<ros::Duration> control_callback_durations_;
  std::vector<ros::Time> timestamps_message_sent_;
  std::vector<std::vector<double>> velocity_errors_;

};
//---------------------------------------------------------------------------------------------------------------------
