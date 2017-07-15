# body_angle_visualizer

The body_angle_visualizer package for ROS.

## Description

body_angle_visualizer produce angles by body parts tf.

## Params

- root
  * the root frame name of body parts.
- to
  * the target frame name of body parts.
- from
  * the base frame name of body parts.

## Require

### tf2

This software require some tf frames. ex)

- /kinect_depth_frame (root)
- /torso (base)
- /head (target)

## Usage

```bash
roslaunch body_angle_visualizer body_angle_visualizer.launch # first turminal
rviz # second turminal
```

(Maybe) you see the visualize angle
