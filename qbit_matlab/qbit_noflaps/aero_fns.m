function [cl_spline, cd_spline, cm_spline] = aero_fns(filename)
%%% Creating a look up table for the NACA 0015 airfoil based on wind tunnel
%%% test data at Re=160,000 for AoA from 0 to 180.
%%% Spencer Folk 2020

% Read in the polars and extract data
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

% figure()
% plot(alpha_data, cl_data,'g*','linewidth',1.5)
% hold on
% % plot(alpha, cl, 'r-', 'linewidth',1.5)
% plot(alpha, ppval(cl_spline,alpha), 'k-', 'linewidth', 2)
% xlabel("AoA (deg)",'interpreter','latex')
% ylabel("$C_L$",'interpreter','latex')
% xlim([-180,180])
% % title("Lift Coefficient",'interpreter','latex')
% grid on
% legend("Data", "Spline Interpolation")
% 
% figure()
% plot(alpha_data, cd_data,'g*','linewidth',1.5)
% hold on
% % plot(alpha, cd, 'r-', 'linewidth',1.5)
% plot(alpha, ppval(cd_spline,alpha), 'k-', 'linewidth', 2)
% xlabel("AoA (deg)",'interpreter','latex')
% ylabel("$C_D$",'interpreter','latex')
% xlim([-180,180])
% % title("Drag Coefficient",'interpreter','latex')
% grid on
% legend("Data", "Spline Interpolation")
% 
% figure()
% plot(alpha_data, cm_data,'g*','linewidth',1.5)
% hold on
% % plot(alpha, cm, 'r-', 'linewidth',1.5)
% plot(alpha, ppval(cm_spline,alpha), 'k-', 'linewidth', 2)
% xlabel("AoA (deg)",'interpreter','latex')
% ylabel("$C_M$",'interpreter','latex')
% xlim([-180,180])
% % title("Pitching Moment Coefficient",'interpreter','latex')
% grid on
% legend("Data", "Spline Interpolation")
end
