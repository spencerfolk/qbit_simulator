%%% Constant height continuous transition maneuver
%%% Spencer Folk 2020

% The goal of this script is to generate a constant height transition
% maneuver by defining the horizontal acceleration as a function of time.
% This is hopefully accomplished by first defining a continuous function
% for alpha, then solving the nonlinear DE to find v_r(t), and differentiate
% to get a_r(t).

clear
clc
close all

%% Tunable parameters
V_s = 30;  % m/s final speed for forward flight
t_star = 60;  % s time we want to take for transition
alpha_0 = pi/2-0.1;  % start at hover position

%% Constants
in2m = 0.0254;
g = 9.81;
rho = 1.2;
stall_angle = 10;  % deg, identified from plot of cl vs alpha

eta = 0.0;   % Efficiency of the down wash on the wings from the propellers


% CRC 9in prop (CRC-3 from CAD)
% Compute a scaling factor based on change in wing span:
span = 0.508;
l = 0.244;
chord = 0.087;
R = 4.5*in2m;   % Estimated 9in prop

scaling_factor = span/(15*in2m);
m = (0.3650)*(scaling_factor^3);  % Mass scales with R^3
Ixx = (2.32e-3)*(scaling_factor^5);

%% Generate Airfoil Look-up
% This look up table data will be used to estimate lift, drag, moment given
% the angle of attack and interpolation from this data.
[cl_spline, cd_spline, cm_spline] = aero_fns("naca_0015_experimental_Re-160000.csv");

%% Final trim flight

% This involves solving for T_top, T_bot, theta
x0 = [m*g/2; m*g/2; pi/4];
fun = @(x) trim_flight(x, cl_spline, cd_spline, cm_spline, m,g,l, chord, span, rho, eta, R, V_s);
    options = optimoptions('fsolve','Display','iter');
% options = optimoptions('fsolve','Display','none','PlotFcn',@optimplotfirstorderopt);
[trim_soln,~,~,output] = fsolve(fun,x0,options);

% From this solver we get our trim solution
theta_star = trim_soln(3);

%% Construct DE solution
v_r0 = 0;  % Initially at rest
tspan = [0, t_star];

[t, v_r] = ode45(@(t,v_r) path_DE(t ,v_r, m, g, rho, span, chord, t_star, theta_star, alpha_0, cl_spline, cd_spline), ...
                tspan, v_r0);

function vdot_r = path_DE(t ,v_r, m, g, rho, span, chord, t_star, theta_star, alpha_0, cl_spline, cd_spline)
% This houses the differential equation for v_r(t) that we're drying to solve

% Given time t, theta_star, and alpha_0, find desired alpha (in rad):
alpha_d = ((theta_star - alpha_0)/t_star)*t + alpha_0;

% Now get Cl, Cd
Cl = ppval(cl_spline, alpha_d*180/pi);
Cd = ppval(cd_spline, alpha_d*180/pi);

vdot_r = g*cot(alpha_d) + ((0.5*rho*chord*span*(Cl*cos(alpha_d) + Cd*sin(alpha_d)))/(m*sin(alpha_d)))*v_r^2;

end
