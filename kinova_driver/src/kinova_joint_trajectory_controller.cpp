#include <angles/angles.h>
#include <kinova_driver/kinova_joint_trajectory_controller.h>
#include <ros/console.h>
// Log trajectory data
#include <fstream>
#include <iostream>
// fmod
#include <cmath>

using namespace kinova;

JointTrajectoryController::JointTrajectoryController(
    kinova::KinovaComm& kinova_comm, ros::NodeHandle& n)
  : kinova_comm_(kinova_comm), nh_(n)
{
  // ROS_DEBUG_STREAM_ONCE("Get in: " << __PRETTY_FUNCTION__);
  ROS_INFO_STREAM("Starting kinova::JointTrajectoryController ...");

  ros::NodeHandle pn("~");
  std::string robot_type;
  nh_.param<std::string>("robot_name", prefix_, "j2n6s300");
  nh_.param<std::string>("robot_type", robot_type, "j2n6s300");
  number_joint_ = robot_type[3] - '0';

  // Display debug information in teminal
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  sub_command_ = nh_.subscribe("trajectory_controller/command", 1,
                               &JointTrajectoryController::commandCB, this);

  pub_joint_feedback_ =
      nh_.advertise<control_msgs::FollowJointTrajectoryFeedback>(
          "trajectory_controller/state", 1);
  pub_joint_velocity_ =
      pn.advertise<kinova_msgs::JointVelocity>("in/joint_velocity", 2);

  traj_frame_id_ = "root";
  joint_names_.resize(number_joint_);
  // std::cout << "joint names in feedback of trajectory state are: " <<
  // std::endl;
  for (uint i = 0; i < joint_names_.size(); i++)
  {
    joint_names_[i] =
        prefix_ + "_joint_" + boost::lexical_cast<std::string>(i + 1);
    std::cout << joint_names_[i] << " ";
  }
  std::cout << std::endl;

  timer_pub_joint_vel_ = nh_.createTimer(
      ros::Duration(0.01), &JointTrajectoryController::pub_joint_vel, this,
      false, false);
  terminate_thread_ = false;

  thread_update_state_ = new boost::thread(
      boost::bind(&JointTrajectoryController::update_state, this));

  traj_feedback_msg_.joint_names.resize(joint_names_.size());
  traj_feedback_msg_.desired.positions.resize(joint_names_.size());
  traj_feedback_msg_.desired.velocities.resize(joint_names_.size());
  traj_feedback_msg_.actual.positions.resize(joint_names_.size());
  traj_feedback_msg_.actual.velocities.resize(joint_names_.size());
  traj_feedback_msg_.error.positions.resize(joint_names_.size());
  traj_feedback_msg_.error.velocities.resize(joint_names_.size());
  traj_feedback_msg_.joint_names = joint_names_;

  // counter in the timer to publish joint velocity command: pub_joint_vel()
  traj_command_points_index_ = 0;

  // ROS_DEBUG_STREAM_ONCE("Get out: " << __PRETTY_FUNCTION__);
}

JointTrajectoryController::~JointTrajectoryController()
{
  // ROS_DEBUG_STREAM_ONCE("Get in: " << __PRETTY_FUNCTION__);
  ROS_WARN("destruction entered!");
  {
    boost::mutex::scoped_lock terminate_lock(terminate_thread_mutex_);
    terminate_thread_ = true;
  }

  sub_command_.shutdown();
  pub_joint_feedback_.shutdown();
  pub_joint_velocity_.shutdown();

  timer_pub_joint_vel_.stop();
  thread_update_state_->join();
  delete thread_update_state_;

  // ROS_DEBUG_STREAM_ONCE("Get out: " << __PRETTY_FUNCTION__);
}

void JointTrajectoryController::commandCB(
    const trajectory_msgs::JointTrajectoryConstPtr& traj_msg)
{
  ROS_INFO_STREAM("JointTrajectoryController received trajectory!");
  // ROS_DEBUG_STREAM_ONCE("Get in: " << __PRETTY_FUNCTION__);

  bool command_abort = false;

  //    // if receive new command, clear all trajectory and stop api
  //    kinova_comm_.stopAPI();
  //    if(!kinova_comm_.isStopped())
  //    {
  //        ros::Duration(0.01).sleep();
  //    }
  //    kinova_comm_.eraseAllTrajectories();

  //    kinova_comm_.startAPI();
  //    if(kinova_comm_.isStopped())
  //    {
  //        ros::Duration(0.01).sleep();
  //    }

  traj_command_points_ = traj_msg->points;
  ROS_INFO_STREAM(
      "Trajectory controller received trajectory with points number: "
      << traj_command_points_.size());

  // Map the index in joint_names and the msg
  std::vector<int> lookup(number_joint_, -1);

  for (size_t j = 0; j < number_joint_; j++)
  {
    for (size_t k = 0; k < traj_msg->joint_names.size(); k++)
      if (traj_msg->joint_names[k] == joint_names_[j])  // find joint_j in msg;
      {
        lookup[j] = k;
        break;
      }

    if (lookup[j] == -1)  // if joint_j not found in msg;
    {
      std::string error_msg = "Joint name : " + joint_names_[j] + " not found "
                                                                  "in the msg.";
      ROS_ERROR("%s", error_msg.c_str());
      command_abort = true;
      return;
    }
  }

  // check msg validation
  for (size_t j = 0; j < traj_command_points_.size(); j++)
  {
    // position should not be empty
    if (traj_command_points_[j].positions.empty())  // find joint_j in msg;
    {
      ROS_ERROR_STREAM(
          "Positions in trajectory command cannot be empty at point: " << j);
      command_abort = true;
      break;
    }
    // position size match
    if (traj_command_points_[j].positions.size() != number_joint_)
    {
      ROS_ERROR_STREAM(
          "Positions at point "
          << j << " has size " << traj_command_points_[j].positions.size()
          << " in trajectory command, which does not match joint number! ");
      command_abort = true;
      break;
    }

    // if velocity provided, size match
    if (!traj_command_points_[j].velocities.empty() &&
        traj_command_points_[j].velocities.size() != number_joint_)
    {
      ROS_ERROR_STREAM(
          "Velocities at point "
          << j << " has size " << traj_command_points_[j].velocities.size()
          << " in trajectory command, which does not match joint number! ");
      command_abort = true;
      break;
    }
  }

  if (command_abort)
    return;

  // store angle velocity command sent to robot
  //    std::vector<KinovaAngles> kinova_angle_command;
  kinova_angle_command_.resize(traj_command_points_.size());
  for (size_t i = 0; i < traj_command_points_.size(); i++)
  {
    kinova_angle_command_[i].InitStruct();  // initial joint velocity to zeros.

    kinova_angle_command_[i].Actuator1 =
        traj_command_points_[i].velocities[0] * 180 / M_PI;
    kinova_angle_command_[i].Actuator2 =
        traj_command_points_[i].velocities[1] * 180 / M_PI;
    kinova_angle_command_[i].Actuator3 =
        traj_command_points_[i].velocities[2] * 180 / M_PI;
    kinova_angle_command_[i].Actuator4 =
        traj_command_points_[i].velocities[3] * 180 / M_PI;
    if (number_joint_ >= 6)
    {
      kinova_angle_command_[i].Actuator5 =
          traj_command_points_[i].velocities[4] * 180 / M_PI;
      kinova_angle_command_[i].Actuator6 =
          traj_command_points_[i].velocities[5] * 180 / M_PI;
      if (number_joint_ == 7)
        kinova_angle_command_[i].Actuator7 =
            traj_command_points_[i].velocities[6] * 180 / M_PI;
    }
  }
  // replace last velocity command (which is zero) to previous non-zero value,
  // trying to drive robot moving a forward to get closer to the goal.
  kinova_angle_command_[traj_command_points_.size() - 1] =
      kinova_angle_command_[traj_command_points_.size() - 2];
  // TODO: This seems odd..?

  std::vector<double> durations(traj_command_points_.size(),
                                0.0);  // computed by time_from_start
  double trajectory_duration = traj_command_points_[0].time_from_start.toSec();

  durations[0] = trajectory_duration;
  //    ROS_DEBUG_STREAM("durationsn 0 is: " << durations[0]);

  for (int i = 1; i < traj_command_points_.size(); i++)
  {
    durations[i] = (traj_command_points_[i].time_from_start -
                    traj_command_points_[i - 1].time_from_start)
                       .toSec();
    trajectory_duration += durations[i];
    //        ROS_DEBUG_STREAM("durations " << i << " is: " << durations[i]);
  }

  // start timer thread to publish joint velocity command
  // time_pub_joint_vel_ = ros::Time::now();
  // timer_pub_joint_vel_.start();
  pid_controller_ = std::make_unique<PidController>(nh_, &kinova_comm_, traj_msg);

  // ROS_DEBUG_STREAM_ONCE("Get out: " << __PRETTY_FUNCTION__);

  // TODO: WIP
  joint_id_to_goal_distances_.clear();
  for (int i = 0; i < /*number_joint_*/ 6 * 2 + 1;
       i++)  // *2 goal & current state, +1 for timestamps
    joint_id_to_goal_distances_[i] = std::vector<double>();

  additional_time_needed_.clear();
}

void JointTrajectoryController::pub_joint_vel(const ros::TimerEvent&)
{
  const auto starting_time = ros::Time::now();
  // send out each velocity command with corresponding duration delay.

  kinova_msgs::JointVelocity joint_velocity_msg;

  if (traj_command_points_index_ < kinova_angle_command_.size() && ros::ok())
  {
    bool last_pass_for_this_point = false;
    const ros::Duration time_from_timer_start =
        (ros::Time::now() - time_pub_joint_vel_);

    // If we're overshooting
    const auto overshoot_time =
        time_from_timer_start -
        traj_command_points_[traj_command_points_index_].time_from_start;
    if ((time_from_timer_start >=
         traj_command_points_[traj_command_points_index_].time_from_start))
    {
      ROS_INFO_STREAM("Overshot by " << overshoot_time.toSec() * 1000
                                     << " milliseconds, next point.");
      // Go to next point
      traj_command_points_index_++;
      last_pass_for_this_point = true;
    }

    joint_velocity_msg.joint1 =
        kinova_angle_command_[traj_command_points_index_].Actuator1;
    joint_velocity_msg.joint2 =
        kinova_angle_command_[traj_command_points_index_].Actuator2;
    joint_velocity_msg.joint3 =
        kinova_angle_command_[traj_command_points_index_].Actuator3;
    joint_velocity_msg.joint4 =
        kinova_angle_command_[traj_command_points_index_].Actuator4;
    joint_velocity_msg.joint5 =
        kinova_angle_command_[traj_command_points_index_].Actuator5;
    joint_velocity_msg.joint6 =
        kinova_angle_command_[traj_command_points_index_].Actuator6;
    joint_velocity_msg.joint7 =
        kinova_angle_command_[traj_command_points_index_].Actuator7;

    // DEBUG
    KinovaAngles current_joint_angles;
    // KinovaAngles current_joint_velocity;
    // AngularPosition current_joint_command;

    // Get joints' current configurations
    {
      const auto start = ros::Time::now();
      // kinova_comm_.getAngularCommand(current_joint_command);
      kinova_comm_.getJointAngles(current_joint_angles);
      // kinova_comm_.getJointVelocities(current_joint_velocity);

      const auto delta_time = ros::Time::now() - start;
      ROS_INFO_STREAM("Reading out information took: " << delta_time.toSec()
                                                       << "s");
      // ROS_INFO_STREAM("---");
    }

    // Calculate current distance to goal position
    {
      const auto start = ros::Time::now();
      try
      {
        // first entry is timestamp
        joint_id_to_goal_distances_[0].emplace_back(
            (ros::Time::now() - time_pub_joint_vel_).toSec());
        // Joint states
        joint_id_to_goal_distances_[1].emplace_back(
            current_joint_angles.Actuator1 / 180.0 * M_PI);
        joint_id_to_goal_distances_[2].emplace_back(
            current_joint_angles.Actuator2 / 180.0 * M_PI);
        joint_id_to_goal_distances_[3].emplace_back(
            current_joint_angles.Actuator3 / 180.0 * M_PI);
        joint_id_to_goal_distances_[4].emplace_back(
            current_joint_angles.Actuator4 / 180.0 * M_PI);
        joint_id_to_goal_distances_[5].emplace_back(
            current_joint_angles.Actuator5 / 180.0 * M_PI);
        joint_id_to_goal_distances_[6].emplace_back(
            current_joint_angles.Actuator6 / 180.0 * M_PI);
        // joint_id_to_goal_distances_[7].emplace_back(current_joint_angles.Actuator7
        // / 180.0 * M_PI);

        // Current joint position goals
        joint_id_to_goal_distances_[7].emplace_back(
            traj_command_points_[traj_command_points_index_].positions[0]);
        joint_id_to_goal_distances_[8].emplace_back(
            traj_command_points_[traj_command_points_index_].positions[1]);
        joint_id_to_goal_distances_[9].emplace_back(
            traj_command_points_[traj_command_points_index_].positions[2]);
        joint_id_to_goal_distances_[10].emplace_back(
            traj_command_points_[traj_command_points_index_].positions[3]);
        joint_id_to_goal_distances_[11].emplace_back(
            traj_command_points_[traj_command_points_index_].positions[4]);
        joint_id_to_goal_distances_[12].emplace_back(
            traj_command_points_[traj_command_points_index_].positions[5]);
        joint_id_to_goal_distances_[13].emplace_back(
            traj_command_points_[traj_command_points_index_].positions[6]);

        // Map radiants onto each other
        for (int i = 1; i < joint_id_to_goal_distances_.size(); i++)
        {
          const auto val = joint_id_to_goal_distances_[i].back();
          if (val < 0.0)
            joint_id_to_goal_distances_[i].back() =
                2.0 * M_PI - std::fmod(-val, (2.0 * M_PI));
          else
            joint_id_to_goal_distances_[i].back() =
                std::fmod(val, (2.0 * M_PI));
        }
      }
      catch (const std::exception& e)
      {
        ROS_INFO_STREAM("Failed to calculate current position offset.");
        ROS_INFO_STREAM("Exception occured:\n" << e.what());
      }
      const auto delta_time = ros::Time::now() - start;
      ROS_INFO_STREAM("Calculations for log file took: " << delta_time.toSec()
                                                         << "s");
    }

    // If time left for this traj point is less than timer precision (10ms),
    // adjust speed to not overshoot the point
    const auto time_left =
        (traj_command_points_[traj_command_points_index_].time_from_start -
         time_from_timer_start)
            .toSec();
    if (time_left < 0.01)
    {
      // adjust_factor<1
      // const float adjust_factor = time_left / 0.01;
      const float adjust_factor = 0.01 / time_left;
      ROS_INFO_STREAM("Using adjust_factor: " << adjust_factor);

      const auto f = 0.1;

      joint_velocity_msg.joint1 *= (adjust_factor * f);
      joint_velocity_msg.joint2 *= (adjust_factor * f);
      joint_velocity_msg.joint3 *= (adjust_factor * f);
      joint_velocity_msg.joint4 *= adjust_factor;
      joint_velocity_msg.joint5 *= adjust_factor;
      joint_velocity_msg.joint6 *= adjust_factor;
      joint_velocity_msg.joint7 *= adjust_factor;
      traj_command_points_index_++;
      last_pass_for_this_point = true;
    }

    // Send trajectory point to driver
    pub_joint_velocity_.publish(joint_velocity_msg);

    // In debug: compare values with topic: follow_joint_trajectory/goal,
    // command
    //        ROS_DEBUG_STREAM_ONCE( std::endl <<" joint_velocity_msg.joint1: "
    //        << joint_velocity_msg.joint1 * M_PI/180 <<
    //                          std::endl <<" joint_velocity_msg.joint2: " <<
    //                          joint_velocity_msg.joint2 * M_PI/180 <<
    //                          std::endl <<" joint_velocity_msg.joint3: " <<
    //                          joint_velocity_msg.joint3 * M_PI/180 <<
    //                          std::endl <<" joint_velocity_msg.joint4: " <<
    //                          joint_velocity_msg.joint4 * M_PI/180 <<
    //                          std::endl <<" joint_velocity_msg.joint5: " <<
    //                          joint_velocity_msg.joint5 * M_PI/180 <<
    //                          std::endl <<" joint_velocity_msg.joint6: " <<
    //                          joint_velocity_msg.joint6 * M_PI/180 );

    if (last_pass_for_this_point)
    {
      ROS_INFO_STREAM("Moved to point " << traj_command_points_index_ - 1);
    }
  }
  else  // if come accross all the points, then stop timer.
  {
    // Print stats about time consumption
    ROS_INFO_STREAM("###");
    ROS_INFO_STREAM("Times to slow: " << additional_time_needed_.size());
    double sum_times = 0.0;
    for (const double val : additional_time_needed_)
      sum_times += val;
    ROS_INFO_STREAM("Sum of additional time needed: " << sum_times);
    ROS_INFO_STREAM(
        "Available time till end:       "
        << traj_command_points_[traj_command_points_index_].time_from_start);

    // TODO: Not necessary as they are not published?
    joint_velocity_msg.joint1 = 0;
    joint_velocity_msg.joint2 = 0;
    joint_velocity_msg.joint3 = 0;
    joint_velocity_msg.joint4 = 0;
    joint_velocity_msg.joint5 = 0;
    joint_velocity_msg.joint6 = 0;
    joint_velocity_msg.joint7 = 0;

    traj_command_points_.clear();

    traj_command_points_index_ = 0;
    timer_pub_joint_vel_.stop();

    // Write goal distances to file
    std::ofstream file;
    const std::string file_path = "/tmp/kinova_trajectory_log.txt";
    const auto separator = ",";

    if (!joint_id_to_goal_distances_.empty())
    {
      try
      {
        file.open(file_path);
        // Add header row
        const int num_keys =
            joint_id_to_goal_distances_.size();  // Already includes space for
        // timestamps
        int num_entries = joint_id_to_goal_distances_[0].size();
        file << separator;  // index column
        int count_keys = 0;

        // first column is timestamp
        file << "Time" << separator;
        for (int i = 0; i < number_joint_; i++)
        {
          file << "Actuator" << i + 1;
          if (count_keys++ != num_keys - 1)
            file << separator;
        }
        // Add goal states
        for (int i = 0; i < number_joint_; i++)
        {
          file << "Actuator" << i + 1 << "Goal";
          if (count_keys++ != num_keys - 1)
            file << separator;
        }

        file << std::endl;
        ROS_INFO_STREAM("Found " << num_entries << " log entries per joint.");

        for (int k = 0; k < num_entries; k++)
        {
          file << k << separator;  // index column
          for (int i = 0; i < num_keys; i++)
          {
            file << joint_id_to_goal_distances_[i][k];
            if (i != num_keys - 1)
              file << separator;
          }
          file << std::endl;
        }
        file.close();

        ROS_INFO_STREAM("Wrote trajectory log to: " << file_path);
      }
      catch (const std::exception& e)
      {
        ROS_INFO_STREAM("Could not write to file at " << file_path);
        ROS_INFO_STREAM("Caught exception: " << e.what());
      }
    }
  }
  const auto delta_time = ros::Time::now() - starting_time;
  ROS_INFO_STREAM("Processing timer callback took: " << delta_time.toSec()
                                                     << "s");
  const auto available_time = 0.01;  // Operates in 100Hz
  const auto perc_available =
      100.0 * (available_time - delta_time.toSec()) / 0.01;
  ROS_INFO_STREAM("Percentage available time remaining: " << perc_available
                                                          << "%");
  if (perc_available < 0.0)
  {
    additional_time_needed_.push_back(delta_time.toSec() - available_time);
    ROS_INFO_STREAM("# TOOK TOO LONG THIS TIME! #");
  }
  ROS_INFO_STREAM("---");
}

void JointTrajectoryController::update_state()
{
  // ROS_DEBUG_STREAM_ONCE("Get in: " << __PRETTY_FUNCTION__);

  ros::Rate update_rate(10);
  previous_pub_ = ros::Time::now();
  while (nh_.ok())
  {
    // check if terminate command is sent from main thread
    {
      boost::mutex::scoped_lock terminate_lock(terminate_thread_mutex_);
      if (terminate_thread_)
      {
        break;
      }
    }

    traj_feedback_msg_.header.frame_id = traj_frame_id_;
    traj_feedback_msg_.header.stamp = ros::Time::now();
    KinovaAngles current_joint_angles;
    KinovaAngles current_joint_velocity;
    AngularPosition current_joint_command;

    kinova_comm_.getAngularCommand(current_joint_command);
    kinova_comm_.getJointAngles(current_joint_angles);
    kinova_comm_.getJointVelocities(current_joint_velocity);

    traj_feedback_msg_.desired.velocities[0] =
        current_joint_command.Actuators.Actuator1 * M_PI / 180;
    traj_feedback_msg_.desired.velocities[1] =
        current_joint_command.Actuators.Actuator2 * M_PI / 180;
    traj_feedback_msg_.desired.velocities[2] =
        current_joint_command.Actuators.Actuator3 * M_PI / 180;
    traj_feedback_msg_.desired.velocities[3] =
        current_joint_command.Actuators.Actuator4 * M_PI / 180;
    if (number_joint_ >= 6)
    {
      traj_feedback_msg_.desired.velocities[4] =
          current_joint_command.Actuators.Actuator5 * M_PI / 180;
      traj_feedback_msg_.desired.velocities[5] =
          current_joint_command.Actuators.Actuator6 * M_PI / 180;
      if (number_joint_ == 7)
        traj_feedback_msg_.desired.velocities[5] =
            current_joint_command.Actuators.Actuator7 * M_PI / 180;
    }

    traj_feedback_msg_.actual.positions[0] =
        current_joint_angles.Actuator1 * M_PI / 180;
    traj_feedback_msg_.actual.positions[1] =
        current_joint_angles.Actuator2 * M_PI / 180;
    traj_feedback_msg_.actual.positions[2] =
        current_joint_angles.Actuator3 * M_PI / 180;
    traj_feedback_msg_.actual.positions[3] =
        current_joint_angles.Actuator4 * M_PI / 180;
    if (number_joint_ >= 6)
    {
      traj_feedback_msg_.actual.positions[4] =
          current_joint_angles.Actuator5 * M_PI / 180;
      traj_feedback_msg_.actual.positions[5] =
          current_joint_angles.Actuator6 * M_PI / 180;
      if (number_joint_ == 7)
        traj_feedback_msg_.actual.positions[5] =
            current_joint_angles.Actuator7 * M_PI / 180;
    }

    traj_feedback_msg_.actual.velocities[0] =
        current_joint_velocity.Actuator1 * M_PI / 180;
    traj_feedback_msg_.actual.velocities[1] =
        current_joint_velocity.Actuator2 * M_PI / 180;
    traj_feedback_msg_.actual.velocities[2] =
        current_joint_velocity.Actuator3 * M_PI / 180;
    traj_feedback_msg_.actual.velocities[3] =
        current_joint_velocity.Actuator4 * M_PI / 180;
    if (number_joint_ >= 6)
    {
      traj_feedback_msg_.actual.velocities[4] =
          current_joint_velocity.Actuator5 * M_PI / 180;
      traj_feedback_msg_.actual.velocities[5] =
          current_joint_velocity.Actuator6 * M_PI / 180;
      if (number_joint_ == 7)
        traj_feedback_msg_.actual.velocities[5] =
            current_joint_velocity.Actuator7 * M_PI / 180;
    }

    for (size_t j = 0; j < joint_names_.size(); j++)
    {
      traj_feedback_msg_.error.velocities[j] =
          traj_feedback_msg_.actual.velocities[j] -
          traj_feedback_msg_.desired.velocities[j];
      // ROS_INFO_STREAM(j << ": " << traj_feedback_msg_.error.velocities[j]);
    }

    //        ROS_WARN_STREAM("I'm publishing after second: " <<
    //        (ros::Time::now() - previous_pub_).toSec());
    pub_joint_feedback_.publish(traj_feedback_msg_);
    previous_pub_ = ros::Time::now();
    update_rate.sleep();
  }
  // ROS_DEBUG_STREAM_ONCE("Get out: " << __PRETTY_FUNCTION__);
}
