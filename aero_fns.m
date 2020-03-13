function [Cl, Cd, Cm] = aero_fns(c0, c1, c2, alpha)
% Computes aerodynamic coefficients given angle of attack alpha
% INPUT
% alpha [1x1] - angle of attack
% OUTPUT
% Cl [1x1] - lift coeff
% Cd [1x1] - Drag coeff
% Cm [1x1] - Pitch moment coeff
% c0 [1x1] - scaling coeff on Cl and Cm
% c1 [1x1] - shifting coeff on Cd
% c2 [1x1] - scaling coeff on Cd

Cl = 2*sin(c0*alpha).*cos(c0*alpha);
Cd = c1 + 2*sin(c2*alpha).^2;
Cm = 0.5*sin(c0*alpha).*cos(c0*alpha);

end