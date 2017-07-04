#include <cmath>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <Eigen/Geometry>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "body_angle_visualizer");
  ros::NodeHandle n;
  ros::Rate r(5);
  rviz_visual_tools::RvizVisualTools rvt {"openni_depth_frame", "rviz_visual_markers"};

  auto count {.0};
  while (ros::ok()) {
    Eigen::Affine3d arrow {};
    arrow.translation() = Eigen::Vector3d(std::sin(count += 0.1) + 1, 0, 0);

    rvt.deleteAllMarkers();
    rvt.publishArrow(arrow);

    rvt.trigger();

    r.sleep();
  }
}
