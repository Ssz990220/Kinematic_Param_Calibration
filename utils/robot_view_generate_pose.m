function robot_view_generate_pose(robot, qs, pause_time)
%ROBOT_VIEW_GENERATE_POSE Summary of this function goes here
%   This function plot the generated pose for measuring in 3D
    if nargin<3
      pause_time = 0.5;
    end
    for i = 1:size(qs,1)
        robot.plot(qs(i,:));
        pause(pause_time);
    end
end

