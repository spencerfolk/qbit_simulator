function qbit_plot_aero()
% This is a simple function to run in the command line to plot our
% aerodynamic functions
% Spencer Folk 2020

% alpha = (-pi/2-pi/8):0.001:(pi/2+pi/8);
alpha = 0:0.001:2.5;

[Cl, Cd, Cm] = aero_fns(4, alpha);

figure()

subplot(1,3,1)
plot(alpha*(180/pi), Cl,'k-',"linewidth",2)
xlabel("AoA (deg)")
ylabel("Lift Coeff ()")
title("Lift")
grid on

subplot(1,3,2)
plot(alpha*(180/pi), Cd,'k-',"linewidth",2)
xlabel("AoA (deg)")
ylabel("Drag Coeff ()")
title("Drag")
grid on

subplot(1,3,3)
plot(alpha*(180/pi), Cm,'k-',"linewidth",2)
xlabel("AoA (deg)")
ylabel("Moment Coeff ()")
title("Pitch Moment")
grid on


end