#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <Eigen/SVD>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "relational_position_publisher");
  ros::NodeHandle n {};

  int target_number {1};
  std::string root_name {"openni_coordinater"};
  std::string to_name {"left_foot"};
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

  ros::Publisher pub {n.advertise<geometry_msgs::Point>(from_frame_name + "_direction", 1)};
  ros::Rate r {5};
  tf2_ros::Buffer tfBuffer {};
  tf2_ros::TransformListener tfListener {tfBuffer};

  while (ros::ok()) {
    try {
      const auto to_pos {tf2::transformToEigen(tfBuffer.lookupTransform(root_name, to_frame_name, ros::Time{0}))};
      const auto from_pos {tf2::transformToEigen(tfBuffer.lookupTransform(root_name, from_frame_name, ros::Time{0}))};
      const auto from_ypr {from_pos.rotation().eulerAngles(1, 0, 2)};
      const auto yaw_angle {from_ypr(0)};
      const Eigen::AngleAxisd yaw_inverse_rotation {-yaw_angle, Eigen::Vector3d::UnitY()};
      const auto stand_vec {to_pos.translation() - from_pos.translation()};
      const auto current_vec {yaw_inverse_rotation * stand_vec};

      pub.publish(tf2::toMsg(current_vec));
    } catch (tf2::TransformException &e) {
      ROS_WARN("%s", e.what());
    }

    r.sleep();
  }
}
