#include <string>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
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
  std::string body_name {"torso"};
  {
    ros::NodeHandle pn {"~"};
    pn.getParam("target_number", target_number);
    pn.getParam("root", root_name);
    pn.getParam("body", body_name);
  }

  const auto body_frame_name {body_name + '_' + std::to_string(target_number)};

  ros::Rate r {5};
  tf2_ros::Buffer tfBuffer {};
  tf2_ros::TransformListener tfListener {tfBuffer};
  tf2_ros::TransformBroadcaster br {};

  geometry_msgs::TransformStamped ts {};
  ts.header.frame_id = root_name;
  ts.child_frame_id = "sentry_frame_" + std::to_string(target_number);
  while (ros::ok()) {
    try {
      auto body_pos {tf2::transformToEigen(tfBuffer.lookupTransform(root_name, body_frame_name, ros::Time{0}))};
      body_pos.translation() = Eigen::Vector3d::Zero();
      const auto body_ypr {body_pos.linear().eulerAngles(1, 0, 2)};
      constexpr auto trim_half_rotation {[](double angle) {
        if (angle < -pi / 2)
          return angle + pi;
        if (angle > pi / 2)
          return angle - pi;
        return angle;
      }};
      const auto yaw_angle {(body_ypr(0))};

      ts.header.stamp = ros::Time::now();
      //ts.transform.rotation = tf2::toMsg(Eigen::Quaterniond{Eigen::AngleAxisd{yaw_angle, Eigen::Vector3d::UnitY()}});
      ts.transform.rotation = tf2::toMsg(Eigen::Quaterniond{body_pos.rotation()});

      br.sendTransform(ts);
    } catch (tf2::TransformException &e) {
      ROS_WARN("%s", e.what());
    }

    r.sleep();
  }
}
