function F = trim_flight(x, alpha_data, cl_data, cd_data, cm_data, m,g,l, cbar, s, rho, V_s)
% Steady state equations for solving for a rough estimate of trim flight
% conditions for the QBiT
% x [3x1] variables vector:
%   x(1) = T_top
%   x(2) = T_bot
%   x(3) = phi
% m [1x1] mass
% g [1x1] gravity
% l [1x1] distance between rotors
% cbar [1x1] wing chord
% s [1x1] wing span
% rho [1x1] air pressure
% V_s [1x1] steady state flight speed

% Get Cl, Cd, Cm from aero look up table
Cl = interp1(alpha_data, cl_data, x(3)*180/pi);
Cd = interp1(alpha_data, cd_data, x(3)*180/pi);
Cm = interp1(alpha_data, cm_data, x(3)*180/pi);

% Compute each 3 equations
F(1) = (x(1) + x(2))*cos(x(3)) - 1/2*rho*cbar*s*(V_s^2)*Cd;
F(2) = (x(1) + x(2))*sin(x(3)) + 1/2*rho*cbar*s*(V_s^2)*Cl - m*g;
F(3) = 1/2*rho*cbar*s*V_s^2*cbar*Cm + l*(x(2) - x(1));

end