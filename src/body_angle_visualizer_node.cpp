#include <cmath>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <Eigen/SVD>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "body_angle_visualizer");
  ros::NodeHandle n {};

  int target_number {1};
  std::string root_name {"openni_coordinater"};
  {
    ros::NodeHandle pn {"~"};
    pn.getParam("target_number", target_number);
    pn.getParam("root", root_name);
  }

  const auto head_name {"head_" + std::to_string(target_number)};
  const auto torso_name {"torso_" + std::to_string(target_number)};

  ros::Publisher pub {n.advertise<geometry_msgs::Point>("body_direction", 1)};
  ros::Rate r {5};
  tf2_ros::Buffer tfBuffer {};
  tf2_ros::TransformListener tfListener {tfBuffer};
  rviz_visual_tools::RvizVisualTools rvt {root_name, "rviz_visual_markers"};

  while (ros::ok()) {
    try {
      const auto head_pos {tf2::transformToEigen(tfBuffer.lookupTransform(root_name, head_name, ros::Time{0}))};
      const auto torso_pos {tf2::transformToEigen(tfBuffer.lookupTransform(root_name, torso_name, ros::Time{0}))};
      const auto torso_ypr {torso_pos.rotation().eulerAngles(1, 0, 2)};
      const auto trim_half_rotation {[](double angle){
        const auto pi {std::acos(-1)};
        if (angle < -pi / 2)
          return angle += pi;
        if (angle > pi / 2)
          return angle -= pi;
        return angle;
      }};
      const auto roll_angle {trim_half_rotation(torso_ypr(2))};
      const auto stand_vec {head_pos.translation() - torso_pos.translation()};
      const auto stand_quaternion {Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), stand_vec)};

      pub.publish(tf2::toMsg(stand_vec));

      rvt.deleteAllMarkers();
      rvt.publishArrow(Eigen::Affine3d{stand_quaternion}, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);

      Eigen::Affine3d text_pos {};
      text_pos.translation() = stand_vec;
      rvt.publishText(text_pos, std::to_string(roll_angle), rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);

      rvt.trigger();
    } catch (tf2::TransformException &e) {
      ROS_WARN("%s", e.what());
    }

    r.sleep();
  }
}
