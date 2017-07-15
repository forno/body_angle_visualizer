#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <Eigen/SVD>

namespace
{

constexpr auto pi {3.141592653589793};

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "body_angle_visualizer");
  ros::NodeHandle n {};

  int target_number {1};
  std::string root_name {"openni_coordinater"};
  std::string to_name {"head"};
  std::string from_name {"torso"};
  {
    ros::NodeHandle pn {"~"};
    pn.getParam("target_number", target_number);
    pn.getParam("root", root_name);
    pn.getParam("to", to_name);
    pn.getParam("from", from_name);
  }

  const auto to_frame_name {to_name + '_' + std::to_string(target_number)};
  const auto from_frame_name {from_name + '_' + std::to_string(target_number)};

  ros::Publisher pub {n.advertise<geometry_msgs::Quaternion>("body_direction", 1)};
  ros::Rate r {5};
  tf2_ros::Buffer tfBuffer {};
  tf2_ros::TransformListener tfListener {tfBuffer};
  rviz_visual_tools::RvizVisualTools rvt {root_name, "rviz_visual_markers"};

  while (ros::ok()) {
    try {
      const auto to_pos {tf2::transformToEigen(tfBuffer.lookupTransform(root_name, to_frame_name, ros::Time{0}))};
      const auto from_pos {tf2::transformToEigen(tfBuffer.lookupTransform(root_name, from_frame_name, ros::Time{0}))};
      const auto from_ypr {from_pos.rotation().eulerAngles(1, 0, 2)};
      constexpr auto trim_half_rotation {[](double angle) {
        if (angle < -pi / 2)
          return angle + pi;
        if (angle > pi / 2)
          return angle - pi;
        return angle;
      }};
      const auto roll_angle {trim_half_rotation(from_ypr(2))};
      constexpr auto invert_half_rotation {[](double angle) {
        if (angle < -pi / 2)
          return -angle - pi;
        if (angle > pi / 2)
          return -angle + pi;
        return angle;
      }};
      const auto pitch_angle {invert_half_rotation(from_ypr(1))};
      const auto stand_vec {to_pos.translation() - from_pos.translation()};
      const auto stand_quaternion {Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), stand_vec)};

      pub.publish(tf2::toMsg(Eigen::Quaterniond{from_pos.rotation()}));

      rvt.deleteAllMarkers();
      rvt.publishArrow(Eigen::Affine3d{stand_quaternion}, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);

      Eigen::Affine3d text_pos {};
      text_pos.translation() = stand_vec.normalized() * .5;
      rvt.publishText(text_pos, std::to_string(roll_angle * 180 / pi), rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE, false);
      text_pos.translation() *= 1.1;
      rvt.publishText(text_pos, std::to_string(pitch_angle * 180 / pi), rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE, false);

      rvt.trigger();
    } catch (tf2::TransformException &e) {
      ROS_WARN("%s", e.what());
    }

    r.sleep();
  }
}
