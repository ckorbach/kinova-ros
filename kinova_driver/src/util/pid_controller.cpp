#include "util/pid_controller.h"
#include "util/kinova_angles_state.h"
#include "util/kinova_angles_state_inline.h"
#include "util/time_helpers.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>
#include <thread>

#define PER_LOOP_OUTPUT 0 
#define ADDITIONAL_OUTPUT 0 

//---------------------------------------------------------------------------------------------------------------------
PidController::PidController(ros::NodeHandle nh, kinova::KinovaComm* comm,
    const trajectory_msgs::JointTrajectoryConstPtr& msg)
    : nh_(nh)
    , comm_(comm)
{
    ROS_INFO_STREAM("PidController constructor called!");
    pub_joint_velocity_ = nh_.advertise<kinova_msgs::JointVelocity>("in/joint_velocity", 2);

    // TODO: Initialize attributes: joint names, positions, velocities
    trajectory_waypoints_ = msg->points;

    updateUnalteredVelocityMessage();
    trajectory_starting_time_ = ros::Time::now();
    // TODO: Send first control message without any checking? Set trajectory
    // starting time shortly before.
    current_time_limit_ = trajectory_starting_time_
        + trajectory_waypoints_[current_waypoint_index_].time_from_start;

    // Start timer callbacks afterwards
    ROS_INFO_STREAM("Starting control callback timer!");
    control_callback_timer_
        = nh_.createTimer(ros::Duration(0.01), &PidController::controlCallback, this, false, false);
    control_callback_timer_.start();
}

//--------------------------------------------------------------------------------------------------------------------
void PidController::updateCurrentWaypoint()
{
#if ADDITIONAL_OUTPUT
    ROS_INFO_STREAM("Checking current waypoint...");
#endif


    control_loop_start_ = ros::Time::now();
    const auto time_left = current_time_limit_ - control_loop_start_;
    // Move to next waypoint, if no time left
    if (time_left.toSec() < 0.0)
    {
        current_waypoint_index_++;
        if (current_waypoint_index_ >= trajectory_waypoints_.size())
        {
            finishTrajectory();
            return;
        }

        current_time_limit_ = trajectory_starting_time_
            + trajectory_waypoints_[current_waypoint_index_].time_from_start;
        updateUnalteredVelocityMessage();

#if ADDITIONAL_OUTPUT
        ROS_INFO_STREAM("Updating current waypoint...");
#endif
    }


#if ADDITIONAL_OUTPUT
    ROS_INFO_STREAM("Current waypoint id: " << current_waypoint_index_);
#endif

}

void PidController::updateCurrentJointStates()
{
    // ROS_INFO_STREAM("Updating current joint states...");
    if (current_state_ != nullptr)
        previous_state_ = std::move(current_state_);

    // Fetch current joint states
    {
#if PER_LOOP_OUTPUT
        const auto start_reading = ros::Time::now();
#endif
        current_state_ = std::make_unique<KinovaAnglesState>(comm_);
#if PER_LOOP_OUPUT
        ROS_INFO_STREAM("Reading current joint states took:    " << nanoseconds_to_string(
                            (ros::Time::now() - start_reading).toNSec()));
#endif
    }


    kinova::KinovaAngles goal_for_current_state;
    // Compute positions which should have been reached at the moment current
    // joint positions were fetched
    {
#if PER_LOOP_OUTPUT
        const auto start_reading = ros::Time::now();
#endif
        getGoalJointPositions(goal_for_current_state, current_state_->getTimestamp());
#if PER_LOOP_OUTPUT
        ROS_INFO_STREAM("Interpolating goal joint states took: " << nanoseconds_to_string(
                            (ros::Time::now() - start_reading).toNSec()));
#endif
    }

    // Compute and store differences in positions compared to idealized joint
    // movement
    {
#if PER_LOOP_OUTPUT
        const auto start_reading = ros::Time::now();
#endif
        // TODO: Store goal in state and update position and velocity errors without providing an argument
        current_state_->computeErrorsInPosition(goal_for_current_state);
#if PER_LOOP_OUTPUT
        ROS_INFO_STREAM("Computing joint state errors took:    " << nanoseconds_to_string(
                            (ros::Time::now() - start_reading).toNSec()));
#endif
    }

    if(previous_state_ != nullptr)
    {
#if PER_LOOP_OUTPUT
        const auto start_reading = ros::Time::now();
#endif
        current_state_->computeVelocityErrors(goal_for_current_state, *previous_state_.get());
#if PER_LOOP_OUTPUT
        ROS_INFO_STREAM("Computing velocity errors took:       " << nanoseconds_to_string(
                    (ros::Time::now() - start_reading).toNSec()));
#endif
    }

    // ROS_INFO_STREAM("Done updating current joint states!");
};

inline void PidController::updateUnalteredVelocityMessage()
{
    unaltered_velocity_msg_.joint1
        = trajectory_waypoints_[current_waypoint_index_].velocities[0] * 180.0 / M_PI;
    unaltered_velocity_msg_.joint2
        = trajectory_waypoints_[current_waypoint_index_].velocities[1] * 180.0 / M_PI;
    unaltered_velocity_msg_.joint3
        = trajectory_waypoints_[current_waypoint_index_].velocities[2] * 180.0 / M_PI;
    unaltered_velocity_msg_.joint4
        = trajectory_waypoints_[current_waypoint_index_].velocities[3] * 180.0 / M_PI;
    unaltered_velocity_msg_.joint5
        = trajectory_waypoints_[current_waypoint_index_].velocities[4] * 180.0 / M_PI;
    unaltered_velocity_msg_.joint6
        = trajectory_waypoints_[current_waypoint_index_].velocities[5] * 180.0 / M_PI;

    // DEBUG
    vector_unaltered_velocity_msg_.clear();
    for(const auto velo : trajectory_waypoints_[current_waypoint_index_].velocities)
        vector_unaltered_velocity_msg_.emplace_back(velo * 180.0 / M_PI);
}

void PidController::getGoalJointPositions(
    kinova::KinovaAngles& goal, const ros::Time& timestamp) const
{
    // ROS_INFO_STREAM("Getting goal joint positions...");

    // TODO: Uses linear interpolation, thus assuming acceleration happens
    // instantaneously -> not as
    // it happens in reality
    
    const auto current_goal = trajectory_waypoints_[current_waypoint_index_];
    if (current_waypoint_index_ == 0u)
    {
        copy_vector_to_kinova_angles(current_goal.positions, goal);
        return;
    }
    const auto previous_goal = trajectory_waypoints_[current_waypoint_index_ - 1u];

    const auto t0 = trajectory_starting_time_ + previous_goal.time_from_start;
    const auto t1 = trajectory_starting_time_ + current_goal.time_from_start;

    const auto interpol_factor = (timestamp - t0).toSec() / (t1 - t0).toSec();
    const auto inv_interpol_factor = 1.0 - interpol_factor;
    // Linearly interpolate
    const auto lin_interpol = [interpol_factor, inv_interpol_factor, &current_goal, &previous_goal](
        unsigned int actuator_id) -> double {
        return inv_interpol_factor * previous_goal.positions[actuator_id]
            + interpol_factor * current_goal.positions[actuator_id];
    };

    generate_values_in_kinova_angles(lin_interpol, goal);
   
}

void PidController::finishTrajectory()
{
    ROS_INFO_STREAM("Finished trajectory! Stopping control callback timer...");
    control_callback_timer_.stop();


    const auto print_time_stats = [](std::vector<double>& values)
    {
        const auto num_values = static_cast<int>(values.size());
        std::sort(std::begin(values), std::end(values));
        ROS_INFO_STREAM("\tmin:    " << nanoseconds_to_string(values.front()));
        ROS_INFO_STREAM("\tmax:    " << nanoseconds_to_string(values.back()));
        
        const auto sum = std::accumulate(std::begin(values), std::end(values), 0.0);
        const auto mean = sum / static_cast<double>(num_values);
        ROS_INFO_STREAM("\tmean:   " << nanoseconds_to_string(mean));

        double variance = 0.0;
        for(const auto val : values)
            variance += (mean - val) * (mean - val);
        variance /= static_cast<double>(num_values);
        const auto std_dev = std::sqrt(variance);
        ROS_INFO_STREAM("\tstddev: " << nanoseconds_to_string(std_dev));


        for(int i = num_values - 1; i >= 0; i--)
        {
            if(values[i] <= 1e7)
            {
                const auto num_too_slow = num_values - 1 - i; 
                ROS_INFO_STREAM("Found " << num_too_slow << " of " << num_values << 
                        " elements > 10ms (" <<
                        100.0 * num_too_slow / static_cast<double>(num_values) << "%).");
                break;
            }
        }
        ROS_INFO_STREAM("---");
    };



    // Distance between control callback calls
    {
        std::vector<double> distances;
        for (int i = 1; i < control_callback_start_timestamps_.size(); i++)
            distances.emplace_back(
                    (control_callback_start_timestamps_[i] - control_callback_start_timestamps_[i - 1]).toNSec());
        ROS_INFO_STREAM("Duration between control callback calls:");
        print_time_stats(distances);
    }

    // Mean control callback durations
    {
        std::vector<double> durations;
        for(const auto& d : control_callback_durations_)
            durations.emplace_back(d.toNSec());
        ROS_INFO_STREAM("Control callback durations:");
        print_time_stats(durations);
    }


    // Distance between publishing velocity messages
    {
        std::vector<double> distances;
        for (int i = 1; i < timestamps_message_sent_.size(); i++)
            distances.emplace_back((timestamps_message_sent_[i] - timestamps_message_sent_[i - 1]).toNSec());
        ROS_INFO_STREAM("Distances between publishing velocity messages:");
        print_time_stats(distances);
   
    }
  

    

    // Set trajectory finished
    std::lock_guard<std::mutex> lock(finished_trajectory_mutex_);
    finished_trajectory_ = true;




    // TODO: Print distance to final goal state!
    previous_state_ = std::move(current_state_);
    current_state_ = std::make_unique<KinovaAnglesState>(comm_);
    kinova::KinovaAngles final_goal;
    copy_vector_to_kinova_angles(trajectory_waypoints_.back().positions,  final_goal);
    
    kinova::KinovaAngles diff;
    get_difference_in_angles(final_goal, current_state_->getJointPositions(), diff);
    std::vector<double> position_diffs;
    copy_kinova_angles_to_vector(diff, position_diffs);
    
    std::stringstream ss;
    for(const auto d : position_diffs)
        ss << d << " ";
    ROS_INFO_STREAM("Final position errors: " << ss.str());



    ROS_INFO_STREAM("Completely finished trajectory!");
    ROS_INFO_STREAM("---------------------");
    ROS_INFO_STREAM("---------------------");
}

bool PidController::isTrajectoryFinished() const
{
    std::lock_guard<std::mutex> lock(finished_trajectory_mutex_);
    return finished_trajectory_;
}

//--------------------------------------------------------------------------------------------------------------------
void PidController::controlCallback(const ros::TimerEvent&)
{
    const auto starting_time_control_callback = ros::Time::now();
#if ADDITIONAL_OUTPUT
    ROS_INFO_STREAM(nanoseconds_to_string((starting_time_control_callback - trajectory_starting_time_).toNSec()) << 
            ": Entering controlCallback...");
#endif
    control_callback_start_timestamps_.push_back(starting_time_control_callback);

    // Update current state
    updateCurrentWaypoint();
    if(finished_trajectory_)
        return;

    updateCurrentJointStates();

    
    const auto& current_velocities_errors = current_state_->getVelocityErrors();
    // First execution of control callback
    if (previous_state_ == nullptr)
    {
        // TODO: Verify this!

        ROS_INFO_STREAM("Initial control loop iteration!");
        // Initially, simply copy resulting errors
        error_deltas_ = current_velocities_errors;
        const auto delta_time
            = (current_state_->getTimestamp() - trajectory_starting_time_).toSec();
        for (auto& delta : error_deltas_)
            delta /= delta_time;
        error_sums_ = current_velocities_errors;
    }
    else
    {
        const auto& previous_velocities_errors = previous_state_->getVelocityErrors();
        const auto delta_time
            = (current_state_->getTimestamp() - previous_state_->getTimestamp()).toSec();

        // Compute changing rate of velocity errors in each joint
        std::transform(std::begin(current_velocities_errors), std::end(current_velocities_errors),
            std::begin(previous_velocities_errors), std::begin(error_deltas_),
            // TODO: Attention if angle a < 2 * Pi and b > 2 * Pi -> b <
            // a, but should be b - a >> 0
            [delta_time](const auto a, const auto b) { return (b - a) / delta_time; });


        // Add velocity errors in each joint to error sums
        std::transform(std::begin(current_velocities_errors), std::end(current_velocities_errors),
            std::begin(error_sums_), std::begin(error_sums_),
            [](const auto a, const auto b) { return a + b; }); // TODO: Integral part - multiply by delta_time?
    }


    // Compute control output based on error deltas and sums
    const auto delta_ns = (previous_state_ != nullptr) ? (current_state_->getTimestamp() - previous_state_->getTimestamp()).toNSec() : 1.0;
    auto control_output = std::vector<double>(num_joints_, 0.0);
    for (int i = 0; i < num_joints_; i++)
    {
        control_output[i] = Kp[i] * current_velocities_errors[i] + Kd[i] * error_deltas_[i]
            + Ki[i] * error_sums_[i];


        ROS_INFO_STREAM(i << ": " << 100 * current_velocities_errors[i]/control_output[i] << " " << 100 * error_deltas_[i]/control_output[i]
                << " " << 100 * error_sums_[i]/control_output[i] << " --> " << control_output[i] * delta_ns << " | "  << vector_unaltered_velocity_msg_[i]);
        
        control_output[i] *= delta_ns;

        //ROS_INFO_STREAM(i << ": " << current_velocities_errors[i] << " " << error_deltas_[i] << " " << error_sums_[i] <<
        //        " --> " << control_output[i] << " | "  << vector_unaltered_velocity_msg_[i]);
    }
    //ROS_INFO_STREAM("Control output: " << ss.str());
    ROS_INFO_STREAM("---");


    // DEBUG
    // Test code by copying basic Kinova controller
    //pub_joint_velocity_.publish(unaltered_velocity_msg_);
    
    auto velocity_msg = unaltered_velocity_msg_;
    add_vector_to_joint_velocity_msg(control_output, velocity_msg);
    pub_joint_velocity_.publish(velocity_msg);

    const auto timestamp_message_published = ros::Time::now();
#if ADDITIONAL_OUTPUT
    ROS_INFO_STREAM(nanoseconds_to_string((timestamp_message_published - trajectory_starting_time_).toNSec()) << 
            ": Unaltered velocity message sent!");
#endif
    // Log time when velocity message was sent
    timestamps_message_sent_.push_back(timestamp_message_published);

#if PER_LOOP_OUTPUT
    // NOTE: Check current control output
    std::stringstream ss;
    ss << "## Control output:\t";
    for (const auto val : control_output)
        ss << val << "\t";
    ROS_INFO_STREAM(ss.str());
#endif

    // DEBUG
    const auto control_callback_duration = ros::Time::now() - starting_time_control_callback;
    control_callback_durations_.push_back(control_callback_duration);
#if PER_LOOP_OUTPUT
    ROS_INFO_STREAM("# Control callback duration: " << nanoseconds_to_string(
                        control_callback_duration.toNSec()));
#endif

#if (PER_LOOP_OUTPUT + ADDITIONAL_OUTPUT)
    ROS_INFO_STREAM("--------------");
#endif
};

PidController::~PidController()
{
    while(!isTrajectoryFinished())
    {
        ROS_INFO_STREAM("Waiting for controller shutdown as trajectory is not yet finished!");
        std::this_thread::sleep_for(std::chrono::seconds{1});
    }
    ROS_INFO_STREAM("Destructing PidController.");
}
