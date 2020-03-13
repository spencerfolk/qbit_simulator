%%% This function will input waypoints and generate a trajectory for the
%%% qbit to follow. Eventually this will be replaced by some form of
%%% trajectory optimization.
%%% Spencer Folk 2020
function [traj, end_time] = qbit_trajectory_generator(waypoints, V_s)
% INPUTS
% waypoints [2xM] system of waypoints to generate a trajectory through
% V_s [1x1] desired average speed to travel between waypoints

[R,M] = size(waypoints);

times = zeros(1,M);
distances = zeros(1,M-1);

for j = 1:(M-1)
    % For each waypoint j, compute time corresponding to each waypoint and
    % construct a spline from those arrays
    distances(j) = sqrt((waypoints(1,j+1)-waypoints(1,j))^2 + (waypoints(2,j+1)-waypoints(2,j))^2);
    times(j+1) = times(j) + distances(j)/V_s;
end

traj = spline(times, horzcat([0;0],waypoints,[0;0]));
end_time = times(end);


end