%%% Bifurcation analysis for symmetric VTOLs
%%% Purpose: understanding the role of the aerodynamics in finding an
%%% equilibrium angle of attack for a given forward airspeed.
%%% Spencer Folk 2020
clear
clc
close all

%% Analytical soln
[cl_spline, cd_spline, cm_spline] = aero_fns("naca_0015_experimental_Re-160000.csv");

alpha = 0:0.001:90;

cl = ppval(cl_spline, alpha);
cd = ppval(cd_spline, alpha);
cm = ppval(cm_spline, alpha);

a_v = cotd(alpha)./(cd + cl.*cotd(alpha));

% Use Lemma 7.6 from Pucci Thesis to determine equilibrium AoA based on cl
% and cd from experimental data. 

cl_spline_der = fnder(cl_spline,1);
cd_spline_der = fnder(cd_spline,1);

cl_der = ppval(cl_spline_der, alpha);
cd_der = ppval(cd_spline_der, alpha);

% Compute p(alpha), q(alpha)
p = 3*cd + cl_der;
q = cd.^2 + cl.^2 + cd.*cl_der - cd_der.*cl;

unstable_conds = find((p.*q < 0) | (p<0 & q<0));

a_v_unstable = a_v(unstable_conds);
alpha_unstable = alpha(unstable_conds);


%% Sweep data
% table = readtable("prop_wash_sweep.csv");
% alpha_e_exp = table.alpha_e;
% a_v_exp = table.a_v;
% alpha_exp = table.alpha;
% cl_exp = table.Cl;
% cd_exp = table.Cd;
% 
% a_v_wprop_exp = (cosd(alpha_exp)./sind(alpha_e_exp))./(cd_exp + cl_exp.*cotd(alpha_e_exp));

xpts = [0, max(a_v) , max(a_v) , 0];
ypts = [9.67 , 9.67 , 14.5, 14.5];
unstable_region = polyshape(xpts, ypts);

figure()
plot(a_v, alpha,'g-','linewidth',2)
hold on
plot(a_v_unstable, alpha(unstable_conds), 'r-', 'linewidth',2)
% plot(unstable_region,'FaceColor','red')
xlabel("a_v [ ]")
ylabel("\alpha [deg]")
legend("Stable", "Unstable")
xlim([0,5])
grid on

figure()
plot(alpha, cl, 'g-','linewidth',2)
hold on
plot(alpha_unstable, cl(unstable_conds), 'r-', 'linewidth',2)
xlabel("\alpha [deg]")
ylabel("Cl")
title("Lift Coefficient")
grid on
legend("Stable","Unstable")

%% Surface analysis for solver
% We're interested in what the surface looks like for our dynamics when
% we're trying to solve for trim flight.. in particular at higher speeds
% where the sovler seems to get caught in a local minima

in2m = 0.0254;
g = 9.81;
rho = 1.2;

span = 0.508;
l = 0.244;
chord = 0.087;
R = 4.5*in2m;   % Estimated 9in prop

scaling_factor = span/(15*in2m);
m = (0.3650)*(scaling_factor^3);  % Mass scales with R^3
Iyy = (2.32e-3)*(scaling_factor^5);

eta = 1;
V_s = 30;

T_top = linspace(0,10,50);
T_bot = linspace(0,10,50);
theta = linspace(0,pi/2,50);

F = zeros(3,length(T_top)*length(T_bot)*length(theta));


