# body_angle_visualizer

The body_angle_visualizer package for ROS.

## Description

body_angle_visualizer produce angles by body parts tf.

## Params

- root
  * the root frame name of body parts.

## Require

### tf2

This software require some tf frames.

- /root
- /torso
- /head

There is a possibility that the increase.

## Usage

```bash
roscore # first turminal
rosrun openni_tracker openni_tracker # second turminal
rosrun body_angle_visualizer body_angle_visualizer # third turminal
rviz # foruth turminal
```

(Maybe) you see the visualize angle
