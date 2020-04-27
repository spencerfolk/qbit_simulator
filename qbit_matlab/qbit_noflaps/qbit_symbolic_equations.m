%%% Using MATLAB symbolic to do the hard work of computing derivatives of
%%% our dynamic equations
%%% Spencer Folk 2020

clear
clc
close all

syms t x xdot xddot z zdot zddot theta thetadot thetaddot real
syms m g l l_E R A_wing A_E c rho real positive
syms c0 c1 c2 real positive
syms T_T T_B real positive
syms L D F_E M_air Cl Cd Cm real 
syms alpha_e gamma delta real
syms Va Vi Vw real

Vi = sqrt( xdot^2 + zdot^2 );
Vw = sqrt( (Vi*cos(theta - gamma))^2 + (0.5*(T_T+T_B))/(0.5*rho*pi*R^2) );
Va = sqrt( Vi^2 + Vw^2 + 2*Vi*Vw*cos(alpha_e));

alpha_e = asin(Vi*sin(theta-gamma)/Va);

L = 0.5*rho*Va*A_wing*(2*sin(c0*alpha_e)*cos(c0*alpha_e));
D = 0.5*rho*Va*A_wing*(c1 + 2*sin(c2*alpha_e)^2);
M_air = 0.5*rho*Va*A_wing*c*(0.5*sin(c0*alpha_e)*cos(c0*alpha_e));

F_E = (1/2)*rho*Va*A_E*(2*sin(c0*(alpha_e+delta))*cos(c0*(alpha_e+delta)));

