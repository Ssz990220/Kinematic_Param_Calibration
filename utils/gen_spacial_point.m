function Ball_pos = gen_spacial_point(T_obj,robot_last_joint_pose,Tool_T)
%GEN_SPACIAL_POINT Summary of this function goes here
%   Detailed explanation goes here
T_obj = [T_obj;1];
SE3_lj = convert_real_robot_pos(robot_last_joint_pose);
Ball_pos = SE3_lj *Tool_T * T_obj;
Ball_pos = Ball_pos(1:3);
end

