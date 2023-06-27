# Test

This file contains the commands to test the functionalities of the two nodes.

## Pick

Open 3 terminals, in the first one:

```bash
roslaunch tiago_gazebo tiago_gazebo.launch
```

in the second one:

```bash
rosrun pick_place pick_node
```

in the third one:

```bash
rostopic pub -1 /obj_pose geometry_msgs/PoseStamped "header:
  frame_id: 'base_footprint'
pose:
  position:
    x: 0.5
    y: -0.5
    z: 0.5
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```
