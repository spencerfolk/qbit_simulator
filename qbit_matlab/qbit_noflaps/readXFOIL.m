% Reading in XFOIL data for NACA 0021
% Spencer Folk 2020
clear
clc
close all

%% Read in data file
sizeA = [7, inf];
formatSpec = '%f %f %f %f %f %f %f';


fileID = fopen('NACA_0021_1e6.txt');
A = fscanf(fileID, formatSpec, sizeA);
A = A';

fclose(fileID);

%% Separate into data and plot
alpha = A(:,1)*(pi/180);
cl = A(:,2);
cd = A(:,3);


%% Estimating coefficients on aero function based on 
c0 = 0;
c1 = cd(alpha == 0);
c2 = 0.5;

i = 0;
cl_error_deriv_table = [];
cl_error_deriv = 100;
cl_error_table = [];
cl_error = 100;

a = 0.02;  % Step size

while abs(cl_error_deriv) > 1e-3
%     [cl_fit, cd_fit, cm_fit] = aero_fns(10, alpha);
    
    % Compute upper bound:
    [cl_fit_upp, cd_fit_upp, cm_fit_upp] = aero_fns(c0+0.01, c1, c2, alpha);
    [cl_fit_low, cd_fit_low, cm_fit_low] = aero_fns(c0-0.01, c1, c2, alpha);
    
    error_upp = sum((cl_fit_upp - cl).^2);
    error_low = sum((cl_fit_low - cl).^2);
    
    cl_error_deriv = (error_upp - error_low)/a;
    
    c0 = c0 - cl_error_deriv*a;
    
    i = i+1;
    
    cl_error_deriv_table = [cl_error_deriv_table , cl_error_deriv];
    cl_error_table = [cl_error_table, 0.5*(error_upp + error_low)];
    if i > 1e4
        break
    end
end

i = 0;
cd_error_deriv_table = [];
cd_error_deriv = 100;
cd_error_table = [];
cd_error = 100;

while abs(cd_error_deriv) > 1e-3
%     [cl_fit, cd_fit, cm_fit] = aero_fns(10, alpha);
    
    % Compute upper bound:
    [cl_fit_upp, cd_fit_upp, cm_fit_upp] = aero_fns(c0, c1, c2+0.01, alpha);
    [cl_fit_low, cd_fit_low, cm_fit_low] = aero_fns(c0, c1, c2-0.01, alpha);
    
    error_upp = sum((cd_fit_upp - cd).^2);
    error_low = sum((cd_fit_low - cd).^2);
    
    cd_error_deriv = (error_upp - error_low)/a;
    
    c2 = c2 - cd_error_deriv*a;
    
    i = i+1;
    
    cd_error_deriv_table = [cd_error_deriv_table , cd_error_deriv];
    cd_error_table = [cd_error_table, 0.5*(error_upp + error_low)];
    if i > 1e4
        break
    end
end

[cl_fit, cd_fit, cm_fit] = aero_fns(c0,c1, c2,alpha);

fprintf("Final c0 coeff: c0 = %1.5f\n",c0)
fprintf("Final c1 coeff: c0 = %1.5f\n",c1)
fprintf("Final c2 coeff: c2 = %1.5f\n",c2)

%% Plotting

figure()
plot(cl_error_table,'r-','linewidth',2)
xlabel("Iter")
ylabel("Error")
grid on
title("LSQ Error")

figure()
subplot(2,1,1)
plot(alpha*180/pi, cl,'g*','linewidth',1)
hold on
plot(alpha*180/pi, cl_fit, 'k-','linewidth',2)
grid on
title("Lift")
xlim([-20, 20])
ylim([-1.5, 1.5])
ylabel("C_L")
legend("XFoil - NACA 0015 @ Re=1e6","Trig Functions",'location','northwest')

subplot(2,1,2)
plot(alpha*180/pi, cd,'g*','linewidth',1)
hold on
plot(alpha*180/pi, cd_fit, 'k-', 'linewidth',2)
grid on
title("Drag")
xlim([-20, 20])
ylim([0, 0.2])
xlabel("Angle of Attack (deg)")
ylabel("C_D")
