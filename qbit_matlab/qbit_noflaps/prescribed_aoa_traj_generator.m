%%% This function designs a planar trajectory (y(t), ydot(t), yddot(t))
%%% For a constant height transition maneuver, based on a given time-
%%% valued function of alpha_e.
%%% Spencer Folk 2020
function [y_des, ydot_des, ydotdot_des]=prescribed_aoa_traj_generator(dt,time,alpha_e_des,cl_spline, cd_spline,rho,m,g,chord,span,accel_bool)
% INPUTS
% dt - sampling rate
% time - time vector corresponding to alpha_e_des
% alpha_e_des - alpha_e (in rad) corresponding to the i'th simulation step
% rho - air density [kg/m^3]
% m - vehicle mass [kg]
% g - gravity [m/s^2]
% chord - wing chord [m]
% span - wing span [m]
% R - rotor radius [m]
% accel_bool - boolean to determine whether or not we consider the
% acceleration

y_des = zeros(1,length(alpha_e_des));
ydot_des = zeros(1,length(alpha_e_des));
ydotdot_des = zeros(1,length(alpha_e_des));

% Get Cl, Cd for the desired alpha_e
cl = ppval(cl_spline,alpha_e_des*180/pi);
cd = ppval(cd_spline,alpha_e_des*180/pi);


if accel_bool == 0
    %%%%%%%%%%%%%%%%% ANALYTICAL WITH NO ACCELERATION %%%%%%%%%%%%%%%%%%%%%%%%%
    % Compute V_i(t) from desired
    V_i = sqrt((2*m*g*cot(alpha_e_des)./(rho*chord*span*(cd + cl.*cot(alpha_e_des)))));
    
    % From V_i we can assign our values for the trajectory:
    ydot_des = V_i;
    ydotdot_des_temp = diff(V_i)/dt;
    ydotdot_des = [ydotdot_des_temp , ydotdot_des_temp(end)];
    
    for i = 2:length(alpha_e_des)
        y_des(i) = y_des(i-1) + V_i(i)*dt;
    end
    
elseif accel_bool == 1
    %%%%%%%%%%%%%%%%% APPROXIMATE SOLN WITH ACCELERATION %%%%%%%%%%%%%%%%%%%%%%
    V_i = zeros(size(time));
    
    for i = 2:length(time)
        V_idot = g*cot(alpha_e_des(i)) - ((0.5*rho*chord*span*(cl(i)*cos(alpha_e_des(i)) + cd(i)*sin(alpha_e_des(i))))/...
            (m*sin(alpha_e_des(i))))*V_i(i-1)^2;
        V_i(i) = V_i(i-1) + V_idot*dt;
        
        ydotdot_des(i) = V_idot;
        ydot_des(i) = V_i(i);
        y_des(i) = y_des(i-1) + V_i(i)*dt;
    end
end

end
