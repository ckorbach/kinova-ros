#pragma once

#include <ros/ros.h>
#include <functional>
#include <vector>
#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_comm.h"
//---------------------------------------------------------------------------------------------------------------------
class KinovaAnglesState
{
public:
  KinovaAnglesState(kinova::KinovaComm* const comm);

  const ros::Time& getTimestamp() const;
  // TODO: Read out velocity directly instead of deducing it from change in
  // position? Working on position still better to reach desired joint
  // configurations more exactly?
  // TODO: Used after having moved to working with velocity errors?
  const kinova::KinovaAngles& getJointPositions() const;

  // How to compute?
  //  - Divide errors in position by time passed between these two states
  //    --> erros in position have to be called before!
  //    + store errors in velocity internally
  void computeVelocityErrors(const kinova::KinovaAngles& goal_positions,
                             const KinovaAnglesState& previous_state);

  const std::vector<double>& getVelocityErrors() const;

  // TODO: Move to private? Only called while processing velocity errors
  void computeErrorsInPosition(const kinova::KinovaAngles& goal);
  const std::vector<double>& getErrorsInPosition() const;

private:
  // Time, when joint positions were requested
  ros::Time timestamp_;
  kinova::KinovaAngles joint_positions_;

  // Stored for later differentiation
  std::vector<double> position_errors_ = std::vector<double>(6, 0.0);

  std::vector<double> velocity_errors_ = std::vector<double>(6, 0.0);
};

//---------------------------------------------------------------------------------------------------------------------

