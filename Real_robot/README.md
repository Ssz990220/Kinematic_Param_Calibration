# Real_robot
This folder contains scripts that calibrate the hand-eye transformation matrix

Normally, fnc_real_generate_eye_calib_pose will not be used independently.
## Real_generate_eye_calib_pose.m
Enter the ball position in tool frame, and the SE3 of the last joint (In our case, x-y-z and unit quaternion),
It will automatically generate measuring pose in joint space. All poses is designed to make the camera facing the object directly (with minor displacement to avoid singularity)
Output is in degree.
## Real_eye_calibration.m
Plugin measuring data (sequense of measuring data from camera (x-y-z of the center of the ball in tool-frame) and last-joint SE3 (same as above, 7 by 1 vector in x-y-z and unit quaternion))
It will calculate the hand-eye transformation matrix by solving an least squares optimization problem.
