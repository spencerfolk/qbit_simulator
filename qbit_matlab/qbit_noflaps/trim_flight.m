function F = trim_flight(x, cl_spline, cd_spline, cm_spline, m,g,l, cbar, s, rho, eta, R, V_s)
% Steady state equations for solving for a rough estimate of trim flight
% conditions for the QBiT
% x [3x1] variables vector:
%   x(1) = T_top
%   x(2) = T_bot
%   x(3) = theta
% m [1x1] mass
% g [1x1] gravity
% l [1x1] distance between rotors
% cbar [1x1] wing chord
% s [1x1] wing span
% rho [1x1] air pressure
% V_s [1x1] steady state flight speed

%% NO PROP WASH QUICK SOLN 
% % Get Cl, Cd, Cm from aero look up table
% Cl = interp1(alpha_data, cl_data, x(3)*180/pi);
% Cd = interp1(alpha_data, cd_data, x(3)*180/pi);
% Cm = interp1(alpha_data, cm_data, x(3)*180/pi);
% 
% % Compute each 3 equations
% F(1) = (x(1) + x(2))*cos(x(3)) - 1/2*rho*cbar*s*(V_s^2)*Cd;
% F(2) = (x(1) + x(2))*sin(x(3)) + 1/2*rho*cbar*s*(V_s^2)*Cl - m*g;
% F(3) = 1/2*rho*cbar*s*V_s^2*cbar*Cm + l*(x(2) - x(1));

%% PROP WASH 
% Now we have to compute some relevant relationships between the states
T_avg = 0.5*(x(1) + x(2));
V_w = eta*sqrt( (V_s*cos(x(3)))^2 + (T_avg/(0.5*rho*pi*R^2)) );
V_a = sqrt( V_s^2 + V_w^2 + 2*V_s*V_w*cos( x(3) ) );
alpha_e = asin(V_s*sin(x(3))/V_a); 
    
% Get Cl, Cd, Cm from aero look up table
Cl = ppval(cl_spline, alpha_e*180/pi);
Cd = ppval(cd_spline, alpha_e*180/pi);
Cm = ppval(cm_spline, alpha_e*180/pi);

D = 1/2*rho*cbar*s*(V_a^2)*Cd;
L = 1/2*rho*cbar*s*(V_a^2)*Cl;
M_air = 1/2*rho*cbar*s*V_a^2*cbar*Cm;

% Compute each 3 equations
F(1) = (x(1) + x(2))*cos(x(3)) - L*sin(x(3)-alpha_e) - D*cos(x(3)-alpha_e);
F(2) = (x(1) + x(2))*sin(x(3)) + L*cos(x(3)-alpha_e) - D*sin(x(3)-alpha_e) - m*g;
F(3) = M_air + l*(x(2) - x(1));
end