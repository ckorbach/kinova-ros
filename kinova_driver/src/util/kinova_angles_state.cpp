#include "util/kinova_angles_state.h"
#include "util/kinova_angles_state_inline.h"
#include <cmath>

//---------------------------------------------------------------------------------------------------------------------
KinovaAnglesState::KinovaAnglesState(kinova::KinovaComm* const comm)
{
    // Set timestamp upon requesting current joint angles
    timestamp_ = ros::Time::now();
    comm->getJointAngles(joint_positions_);
}
const ros::Time& KinovaAnglesState::getTimestamp() const
{
    return timestamp_;
}

const kinova::KinovaAngles& KinovaAnglesState::getJointPositions() const
{
    return joint_positions_;
}

const std::vector<double>& KinovaAnglesState::getErrorsInPosition() const
{
    return position_errors_;
}

const std::vector<double>& KinovaAnglesState::getVelocityErrors() const
{
    return velocity_errors_;
}

void KinovaAnglesState::computeErrorsInPosition(const kinova::KinovaAngles& goal)
{
    kinova::KinovaAngles diff;
    get_difference_in_angles(goal, joint_positions_, diff);
    copy_kinova_angles_to_vector(diff, position_errors_);
}
void KinovaAnglesState::computeVelocityErrors(
    const kinova::KinovaAngles& goal_positions, const KinovaAnglesState& previous_state)
{
    const auto& previous_positions = previous_state.getJointPositions();
    const auto delta_time = getTimestamp() - previous_state.getTimestamp();

    kinova::KinovaAngles delta_positions;
    get_difference_in_angles(joint_positions_, previous_positions, delta_positions);
    copy_kinova_angles_to_vector(delta_positions, velocity_errors_);

    // Joint positions to velocities
    const auto delta_ns = delta_time.toNSec();
    for (auto& pos : velocity_errors_)
        pos /= delta_ns;

    // TODO: Output current velocity errors and check their correctness!
}
//---------------------------------------------------------------------------------------------------------------------

