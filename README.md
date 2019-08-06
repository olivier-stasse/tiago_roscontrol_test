Test of ros-control on Tiago.

To run in gazebo:

    roslaunch tiago_roscontrol_test tiago_roscontrol_test_gazebo.launch use_mobile_base:=true

To run on the robot:

    roslaunch tiago_roscontrol_test tiago_roscontrol_test.launch use_mobile_base:=true

When `use_mobile_base` is set to `true`, the wheels are control in velocity, to their position when lauching the controller.
