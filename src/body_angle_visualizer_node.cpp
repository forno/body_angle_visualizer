#include <cmath>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "body_angle_visualizer");
  ros::NodeHandle n {};
  ros::Publisher pub {n.advertise<geometry_msgs::Quaternion>("body_direction", 1)};
  ros::Rate r {5};
  tf2_ros::Buffer tfBuffer {};
  tf2_ros::TransformListener tfListener {tfBuffer};
  rviz_visual_tools::RvizVisualTools rvt {"openni_depth_frame", "rviz_visual_markers"};

  while (ros::ok()) {
    try {
      const auto head_pos {tf2::transformToEigen(tfBuffer.lookupTransform("openni_depth_frame", "head_1", ros::Time {0}))};
      const auto torso_pos {tf2::transformToEigen(tfBuffer.lookupTransform("openni_depth_frame", "torso_1", ros::Time {0}))};
      const auto stand_vec {head_pos.translation() - torso_pos.translation()};
      const auto stand_quaternion {Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), stand_vec)};

      pub.publish(tf2::toMsg(stand_quaternion));
      rvt.deleteAllMarkers();
      rvt.publishArrow(Eigen::Affine3d{stand_quaternion});

      const auto stand_euler_rpy {stand_quaternion.toRotationMatrix().eulerAngles(0, 1, 2)};
      Eigen::Affine3d text_pos {};
      text_pos.translation() = Eigen::Vector3d::UnitZ();
      rvt.publishText(text_pos, std::to_string(stand_euler_rpy(0)));

      rvt.trigger();
    } catch (tf2::TransformException &e) {
      ROS_WARN("%s", e.what());
    }

    r.sleep();
  }
}
