%%% Script for testing the longitudinal dynamics.
%%% Spencer Folk 2020
clear
clc
close all

%% Constants

in2m = 0.0254;
span = 0.508;
l = 0.244;
chord = 0.087;
R = 4.5*in2m;   % Estimated 9in prop

g = 9.81;
rho = 1.2;
stall_angle = 10;  % deg, identified from plot of cl vs alpha

eta = 0.0;   % Efficiency of the down wash on the wings from the propellers;

scaling_factor = span/(15*in2m);
m = (0.3650)*(scaling_factor^3);  % Mass scales with R^3
Ixx = (2.32e-3)*(scaling_factor^5);

%% Generate Airfoil Look-up
% This look up table data will be used to estimate lift, drag, moment given
% the angle of attack and interpolation from this data.
[cl_spline, cd_spline, cm_spline] = aero_fns("naca_0015_experimental_Re-160000.csv");

%% Arrays
tf = 10;
dt = 0.01;

time = 0:dt:tf;

y = zeros(size(time));
z = zeros(size(time));
theta = zeros(size(time));

ydot = zeros(size(time));
zdot = zeros(size(time));
thetadot = zeros(size(time));
thetadotdot = zeros(size(time));

Vi = zeros(size(time));
gamma = zeros(size(time));
gammadot = zeros(size(time));

T_top = ones(size(time))*(m*g/2);
T_bot = ones(size(time))*(m*g/2);

qdot = zeros(6,length(time));

% Initial conditions:
Vi(1) = 0;
gamma(1) = pi/4;
theta(1) = pi/4;
thetadot(1) = 0;
y(1) = 0;
z(1) = 0;

%% Simulate

for i = 2:length(time)
    % Iterate through time
    q0 = [Vi(i-1) gamma(i-1) theta(i-1) thetadot(i-1) y(i-1) z(i-1)]';
    u = [T_top(i) T_bot(i)]';
    
    qdot(:,i) = qbit_longitudinal_dynamics(q0,u,m,g,Ixx,l,eta,rho,R,chord,span,cl_spline,cd_spline,cm_spline);
    
    Vidot = qdot(1,i);
    gammadot = qdot(2,i);
    thetadot(i) = qdot(3,i);
    thetadotdot(i) = qdot(4,i);
    ydot(i) = qdot(5,i);
    zdot(i) = qdot(6,i);
    
    q = q0 + qdot(:,i)*dt;
    
    Vi(i) = q(1);
    gamma(i) = q(2);
    theta(i) = q(3);
    thetadot(i) = q(4);
    y(i) = q(5);
    z(i) = q(6);
    
end

h = figure();
qbit_animate_trajectory(h, time,[y ; z ; theta], [min(y),max(y)], [min(z),max(z)],zeros(2,length(time)) , l , false)
