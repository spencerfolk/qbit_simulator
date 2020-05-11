function xdot = qbit_longitudinal_dynamics(x,u, m, g, Ixx, l, eta, rho, R, chord, span, cl_spline, cd_spline, cm_spline)
%%% Dynamics for the qbit in the state space form.
% Inputs --
% x [4x1] - [Vi, gamma, theta, thetadot,  x, z] 
%            m/s , rad , rad , rad/s  m, m
% u [2x1] - [T_top , T_bot]
%             N  ,   N
% cl, cd, cm splines - spline objects for getting cl, cd, cm

% Outputs --
% xdot [4x1] - [Vidot , gammadot, thetadot , thetadotdot, xdot, zdot]

%%% NOTE - the dynamics here are in longitudinal form (think path
%%% coordinates). 

Vw = eta*sqrt( (x(1)*cos(x(3)-x(2)))^2 + (u(1) + u(2))/(rho*pi*R^2) );
Va = sqrt( x(1)^2 + Vw + 2*x(1)*Vw*cos(x(3)-x(2)) );

if Va >= 1e-5
        alpha_e = asin((x(1)*sin(x(3)-x(2)))/Va);
    else
        alpha_e = 0;
end

% Aero
% [cl_spline, cd_spline, cm_spline] = aero_fns("naca_0015_experimental_Re-160000.csv");

Cl = ppval(cl_spline, alpha_e*180/pi);
Cd = ppval(cd_spline, alpha_e*180/pi);
Cm = ppval(cm_spline, alpha_e*180/pi);

Vidot = ( (u(1)+u(2))*cos(x(3)-x(2)) - (1/2)*rho*chord*span*Va^2*(Cl*sin(x(3)-x(2)-alpha_e) + Cd*cos(x(3)-x(2)-alpha_e)) - m*g*sin(x(2)) )/m;
gammadot = ( (u(1)+u(2))*sin(x(3)-x(2)) + (1/2)*rho*chord*span*Va^2*(Cl*cos(x(3)-x(2)-alpha_e) - Cd*sin(x(3)-x(2)-alpha_e)) - m*g*cos(x(2)) )/(m*x(1));
thetadot = x(4);
thetadotdot = ((1/2)*rho*chord*span*Va^2*Cm*chord + l*(u(2)-u(1)))/Ixx;
xdot = x(1)*cos(x(2));
zdot = x(1)*sin(x(2));

% Note that the dynamics at hover have to be accounted for... 
if x(1) <= 1e-2
   gammadot = thetadotdot;
end

xdot = [Vidot gammadot thetadot thetadotdot, xdot, zdot]';
end