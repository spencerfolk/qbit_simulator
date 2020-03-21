function [alpha_data, cl_data, cd_data, cm_data] = aero_lookup(filename)
%%% Creating a look up table for the NACA 0015 airfoil based on wind tunnel
%%% test data at Re=160,000 for AoA from 0 to 180.
%%% Spencer Folk 2020

% Read in the polars and extract data
% filename = "naca_0015_experimental_Re-160000.csv";
polars = readmatrix(filename);

alpha_data = [-flipud(polars(:,1)) ; polars(2:end,1)];
cl_data = [-flipud(polars(:,2)) ; polars(2:end,2)];
cd_data = [flipud(polars(:,3)) ; polars(2:end,3)];
cm_data = [-flipud(polars(:,4)) ; polars(2:end,4)];

alpha = alpha_data(1):0.01:alpha_data(end);
cl = interp1(alpha_data, cl_data, alpha);
cd = interp1(alpha_data, cd_data, alpha);
cm = interp1(alpha_data, cm_data, alpha);

% figure()
% plot(alpha_data, cl_data,'ko','linewidth',1.5)
% hold on
% plot(alpha, cl, 'r-', 'linewidth',2)
% xlabel("AoA (deg)")
% ylabel("Lift Coefficient")
% grid on
% legend("Data", "Linear Interpolation")
% 
% figure()
% plot(alpha_data, cd_data,'ko','linewidth',1.5)
% hold on
% plot(alpha, cd, 'r-', 'linewidth',2)
% xlabel("AoA (deg)")
% ylabel("Drag Coefficient")
% grid on
% legend("Data", "Linear Interpolation")
% 
% figure()
% plot(alpha_data, cm_data,'ko','linewidth',1.5)
% hold on
% plot(alpha, cm, 'r-', 'linewidth',2)
% xlabel("AoA (deg)")
% ylabel("Moment Coefficient")
% grid on
% legend("Data", "Linear Interpolation")
end
