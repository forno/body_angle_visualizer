#include <cmath>

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "body_angle_visualizer");
  ros::NodeHandle n {};
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

      rvt.deleteAllMarkers();
      rvt.publishArrow(Eigen::Affine3d{stand_quaternion});

      rvt.trigger();
    } catch (tf2::TransformException &e) {
      ROS_WARN("%s", e.what());
    }

    r.sleep();
  }
}
