# Lab Setup Notes

Unlock joints
Unlock kill switch

`roslaunch franka_example_controllers joint_position_example_controller`

Run azure kinect driver:
`roslaunch azure_kinect_ros_driver rectify_test.launch`


Restart (lock and unlock, reset controllers) if an error occurs or something else doesn't work

## Hand Movement

white light should be on; go once through safety border if it doesn't (sometimes it turns purple);
press all three buttons on the endeffector to move the arm

## Positioning
- Checkerboard: corner on the other side to the kinect power side (some centimeters to the side)
- fix cables at last joint and first joint
- phantom
- turn off light for infrared (whole time)

- note self collision after 18.th photo