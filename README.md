# apriltag_location_to_map

About: This package continually subscribes to the frames produced by the ros_apriltags package. Then, the poses of the defined apriltags are registered and kept track within the memory of the node.

To run:

Execute the following commands
```
rosrun apriltag_location_to_map apriltag_location_to_map_convert
```

You can run the ros_apriltags package with the following line within a roslaunch file
```
<include file="$(find apriltag_ros)/launch/continuous_detection_front.launch"/>
```
