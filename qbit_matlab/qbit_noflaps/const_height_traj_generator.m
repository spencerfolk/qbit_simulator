%%% This function designs a planar trajectory (x(t), xdot(t), xddot(t))
%%% For a constant height transition maneuver, based on a given time-
%%% valued function of alpha_e. 
%%% Spencer Folk 2020
function [x_des, xdot_des, xdotdot_des]=const_height_traj_generator(dt,time,alpha_e_des,cl_spline, cd_spline,rho,m,g,chord,span)
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

x_des = zeros(1,length(alpha_e_des));
xdot_des = zeros(1,length(alpha_e_des));
xdotdot_des = zeros(1,length(alpha_e_des));

% Get Cl, Cd for the desired alpha_e
cl = ppval(cl_spline,alpha_e_des*180/pi);
cd = ppval(cd_spline,alpha_e_des*180/pi);

%%%%%%%%%%%%%%%%% ANALYTICAL WITH NO ACCELERATION %%%%%%%%%%%%%%%%%%%%%%%%%
% Compute V_i(t) from desired 
V_i = sqrt((2*m*g*cot(alpha_e_des)./(rho*chord*span*(cd + cl.*cot(alpha_e_des)))));

% From V_i we can assign our values for the trajectory: 
xdot_des = V_i;
xdotdot_des_temp = diff(V_i)/dt;
xdotdot_des = [xdotdot_des_temp , xdotdot_des_temp(end)];

for i = 2:length(alpha_e_des)
    x_des(i) = x_des(i-1) + V_i(i)*dt;
end

% figure()
% plot(xdot_des,'linewidth',1.5)
% hold on

%%%%%%%%%%%%%%%%% APPROXIMATE SOLN WITH ACCELERATION %%%%%%%%%%%%%%%%%%%%%%
V_i = zeros(size(time));

for i = 2:length(time)
    V_idot = g*cot(alpha_e_des(i)) - ((0.5*rho*chord*span*(cl(i)*cos(alpha_e_des(i)) + cd(i)*sin(alpha_e_des(i))))/...
                                      (m*sin(alpha_e_des(i))))*V_i(i-1)^2;
    V_i(i) = V_i(i-1) + V_idot*dt;
    
    xdotdot_des(i) = V_idot;
    xdot_des(i) = V_i(i);
    x_des(i) = x_des(i-1) + V_i(i)*dt;
end

% plot(V_i,'linewidth',1.5)
% xlabel("Time [s]")
% ylabel("V_i [m/s]")
% legend("Steady State -- Analytical", "Dynamic -- Approximate")
% title("Steady State vs Dynamic Speed Functions")
% grid on

end
