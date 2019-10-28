#include <vector>
#include "kinova_driver/kinova_api.h"
#include <kinova_msgs/JointVelocity.h>
//---------------------------------------------------------------------------------------------------------------------
namespace
{
inline double normalize_angle(double rad_angle)
{
  const auto TWO_PI = 2.0 * M_PI;
  if (rad_angle < 0.0)
    return TWO_PI - std::fmod(-rad_angle, TWO_PI);
  return std::fmod(rad_angle, TWO_PI);
}
}
//---------------------------------------------------------------------------------------------------------------------
inline kinova::KinovaAngles get_difference_in_angles(
    const kinova::KinovaAngles& a, const kinova::KinovaAngles& b,
    kinova::KinovaAngles& diff)
{
  /*
  diff.Actuator1 = normalize_angle(normalize_angle(a.Actuator1) - normalize_angle(b.Actuator1));
  diff.Actuator2 = normalize_angle(normalize_angle(a.Actuator2) - normalize_angle(b.Actuator2));
  diff.Actuator3 = normalize_angle(normalize_angle(a.Actuator3) - normalize_angle(b.Actuator3));
  diff.Actuator4 = normalize_angle(normalize_angle(a.Actuator4) - normalize_angle(b.Actuator4));
  diff.Actuator5 = normalize_angle(normalize_angle(a.Actuator5) - normalize_angle(b.Actuator5));
  diff.Actuator6 = normalize_angle(normalize_angle(a.Actuator6) - normalize_angle(b.Actuator6));
  */
  
  diff.Actuator1 = normalize_angle(a.Actuator1) - normalize_angle(b.Actuator1);
  diff.Actuator2 = normalize_angle(a.Actuator2) - normalize_angle(b.Actuator2);
  diff.Actuator3 = normalize_angle(a.Actuator3) - normalize_angle(b.Actuator3);
  diff.Actuator4 = normalize_angle(a.Actuator4) - normalize_angle(b.Actuator4);
  diff.Actuator5 = normalize_angle(a.Actuator5) - normalize_angle(b.Actuator5);
  diff.Actuator6 = normalize_angle(a.Actuator6) - normalize_angle(b.Actuator6);
}

inline void add_vector_to_joint_velocity_msg(const std::vector<double>& values, kinova_msgs::JointVelocity& msg)
{
    msg.joint1 += values[0];
    msg.joint2 += values[1];
    msg.joint3 += values[2];
    msg.joint4 += values[3];
    msg.joint5 += values[4];
    msg.joint6 += values[5];
}

inline void copy_kinova_angles_to_vector(const kinova::KinovaAngles& angles,
                                         std::vector<double>& target)
{
  target = std::vector<double>{ angles.Actuator1, angles.Actuator2,
                                angles.Actuator3, angles.Actuator4,
                                angles.Actuator5, angles.Actuator6 };
}
inline void copy_vector_to_kinova_angles(const std::vector<double>& values,
                                         kinova::KinovaAngles& angles)
{
  if (values.size() < 6u)
    throw std::runtime_error("copyVectorToKinovaAngles: Given vector contains "
                             "less values than joints are present!");

  angles.Actuator1 = values[0];
  angles.Actuator2 = values[1];
  angles.Actuator3 = values[2];
  angles.Actuator4 = values[3];
  angles.Actuator5 = values[4];
  angles.Actuator6 = values[5];
}

inline void generate_values_in_kinova_angles(
    const std::function<double(unsigned int)>& generator,
    kinova::KinovaAngles& angles)
{
  angles.Actuator1 = generator(0u);
  angles.Actuator2 = generator(1u);
  angles.Actuator3 = generator(2u);
  angles.Actuator4 = generator(3u);
  angles.Actuator5 = generator(4u);
  angles.Actuator6 = generator(5u);
}
//---------------------------------------------------------------------------------------------------------------------

