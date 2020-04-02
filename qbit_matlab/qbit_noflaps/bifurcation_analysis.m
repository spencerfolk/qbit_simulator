%%% Bifurcation analysis for symmetric VTOLs
%%% Purpose: understanding the role of the aerodynamics in finding an
%%% equilibrium angle of attack for a given forward airspeed.
%%% Spencer Folk 2020
clear
clc
close all

%% Analytical soln
[cl_spline, cd_spline, cm_spline] = aero_fns("naca_0015_experimental_Re-160000.csv");

alpha = 0:0.1:90;

% Change these arrays:
cl_exp = ppval(cl_spline, alpha);
cd_exp = ppval(cd_spline, alpha);
cm_exp = ppval(cm_spline, alpha);

a_v = cotd(alpha)./(cd_exp + cl_exp.*cotd(alpha));

c0 = 4.80914;  % coeff acts as a scaling factor on Cl, Cm
c1 = 0.02;     % coeff acts as a shifting factor on Cd
c2 = 0.61929;  % coeff acts as a scaling factor on Cd

[cl_trig, cd_trig, cm_trig] = aero_fns_trig(c0, c1, c2, alpha*pi/180);

a_v_trig = cotd(alpha)./(cd_trig + cl_trig.*cotd(alpha));

%% Sweep data
table = readtable("prop_wash_sweep.csv");
alpha_e_exp = table.alpha_e;
a_v_exp = table.a_v;
alpha_exp = table.alpha;
cl_exp = table.Cl;
cd_exp = table.Cd;

a_v_wprop_exp = (cosd(alpha_exp)./sind(alpha_e_exp))./(cd_exp + cl_exp.*cotd(alpha_e_exp));

figure()
plot(a_v, alpha,'r-','linewidth',2)
hold on
% plot(a_v_trig, alpha,'b-','linewidth',2)
% scatter(a_v_exp, alpha_e_exp)
xlabel("a_v [ ]")
ylabel("\alpha [deg]")
xlim([0,20])
% legend("Experimental NACA 0015", "Simulation")
grid on

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
phi = linspace(0,pi/2,50);

F = zeros(3,length(T_top)*length(T_bot)*length(phi));


