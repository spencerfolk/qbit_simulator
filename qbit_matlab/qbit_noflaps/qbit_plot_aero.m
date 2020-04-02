function qbit_plot_aero()
% This is a simple function to run in the command line to plot our
% aerodynamic functions
% Spencer Folk 2020

% alpha = (-pi/2-pi/8):0.001:(pi/2+pi/8);
alpha = 0:0.001:pi;

% [Cl, Cd, Cm] = aero_fns(4, alpha);
% 
% figure()
% 
% subplot(1,3,1)
% plot(alpha*(180/pi), Cl,'k-',"linewidth",2)
% xlabel("AoA (deg)")
% ylabel("Lift Coeff ()")
% title("Lift")
% grid on
% 
% subplot(1,3,2)
% plot(alpha*(180/pi), Cd,'k-',"linewidth",2)
% xlabel("AoA (deg)")
% ylabel("Drag Coeff ()")
% title("Drag")
% grid on
% 
% subplot(1,3,3)
% plot(alpha*(180/pi), Cm,'k-',"linewidth",2)
% xlabel("AoA (deg)")
% ylabel("Moment Coeff ()")
% title("Pitch Moment")
% grid on

filename = "naca_0015_experimental_Re-160000.csv";
polars = readmatrix(filename);

alpha_data = [-flipud(polars(:,1)) ; polars(2:end,1)];
cl_data = [-flipud(polars(:,2)) ; polars(2:end,2)];
cd_data = [flipud(polars(:,3)) ; polars(2:end,3)];
cm_data = [-flipud(polars(:,4)) ; polars(2:end,4)];

alpha = alpha_data(1):0.01:alpha_data(end);
cl = interp1(alpha_data, cl_data, alpha);
cd = interp1(alpha_data, cd_data, alpha);
cm = interp1(alpha_data, cm_data, alpha);

cl_spline = spline(alpha_data, cl_data);
cd_spline = spline(alpha_data, cd_data);
cm_spline = spline(alpha_data, cm_data);

figure()
plot(alpha_data, cl_data,'g*','linewidth',1.5)
hold on
% plot(alpha, cl, 'r-', 'linewidth',1.5)
plot(alpha, ppval(cl_spline,alpha), 'k-', 'linewidth', 2)
xlabel("AoA (deg)")
ylabel("Lift Coefficient")
title("Lift")
xlim([0,90])
grid on
legend("Data", "Spline Interpolation")

figure()
plot(alpha_data, cd_data,'g*','linewidth',1.5)
hold on
% plot(alpha, cd, 'r-', 'linewidth',1.5)
plot(alpha, ppval(cd_spline,alpha), 'k-', 'linewidth', 2)
xlabel("AoA (deg)")
ylabel("Drag Coefficient")
title("Drag")
xlim([0,90])
grid on
legend("Data", "Spline Interpolation")

figure()
plot(alpha_data, cm_data,'g*','linewidth',1.5)
hold on
% plot(alpha, cm, 'r-', 'linewidth',1.5)
plot(alpha, ppval(cm_spline,alpha), 'k-', 'linewidth', 2)
xlabel("AoA (deg)")
ylabel("Moment Coefficient")
title("Pitching Moment")
xlim([0,90])
grid on
legend("Data", "Spline Interpolation")


end