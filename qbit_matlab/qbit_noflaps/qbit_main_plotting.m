
%%% This code snippit will run all the plotting commands after running
%%% qbit_simulate_master. It plots the main data plots used for the report.
%%% Spencer Folk 2020

%% Animation

if animate == true
    h = figure();
    qbit_animate_trajectory(h, time,[y ; z ; theta], desired_state(1,:), desired_state(2,:),Fdes,l, save_animation)
    
    if traj_type == "cubic"
        hold on
        plot(waypoints(1,:),waypoints(2,:),'ko','linewidth',2)
    end
    axis equal
end

%% Plotting

% States
figure()
sgtitle("States",'interpreter','latex')

subplot(3,1,1)
plot(time, y, 'r-','linewidth',1.5)
hold on
plot(time, desired_state(1,:), 'k--', 'linewidth',1)
ylabel('y [m]','interpreter','latex')
xlim([0,time(end)])
grid on

subplot(3,1,2)
plot(time, z, 'k-','linewidth',1.5)
hold on
plot(time, desired_state(2,:), 'k--', 'linewidth',1)
ylabel('z [m]','interpreter','latex')
xlim([0,time(end)])
ylim([-0.5,0.5])
grid on

subplot(3,1,3)
plot(time, theta*180/pi, 'b-','linewidth',1.5)
hold on
% plot(time, ones(size(time))*90, 'k--', 'linewidth', 1)
ylabel('$\theta$ [deg]','interpreter','latex')
xlim([0,time(end)])
xlabel("Time [s]",'interpreter','latex')
grid on

figure()
sgtitle("State Derivatives",'interpreter','latex')

subplot(3,1,1)
plot(time, ydot, 'r-','linewidth',1.5)
hold on
plot(time, desired_state(3,:), 'k--', 'linewidth',1)
ylabel('$\dot{y}$ [m/s]','interpreter','latex')
xlim([0,time(end)])
grid on

subplot(3,1,2)
plot(time, zdot, 'k-','linewidth',1.5)
hold on
plot(time, desired_state(4,:), 'k--', 'linewidth',1)
ylabel('$\dot{z}$ [m/s]','interpreter','latex')
xlim([0,time(end)])
grid on

subplot(3,1,3)
plot(time, thetadot*180/pi, 'b-','linewidth',1.5)
ylabel('$\dot{\theta}$ [deg/s]','interpreter','latex')
xlim([0,time(end)])
xlabel("Time [s]",'interpreter','latex')
grid on

figure()
sgtitle("Body Acceleration",'interpreter','latex')

subplot(2,1,1)
plot(time,ydotdot,'r-','linewidth',1.5)
hold on
plot(time,desired_state(5,:),'k--','linewidth',1.5)
ylabel("$\ddot{y}$ [$m/s^2$]",'interpreter','latex')
xlim([0,time(end)]);
legend("Actual", "Desired")
grid on

subplot(2,1,2)
plot(time,zdotdot,'k-','linewidth',1.5)
hold on
plot(time,desired_state(6,:),'k--','linewidth',1.5)
ylabel("$\ddot{z}$ [$m/s^2$]",'interpreter','latex')
xlim([0,time(end)]);
legend("Actual", "Desired")
xlabel("Time [s]",'interpreter','latex')
grid on

figure()
sgtitle("Aero Forces/Moments",'interpreter','latex')

subplot(3,1,1)
plot(time, L, 'r-','linewidth',1.5)
ylabel('Lift [N]','interpreter','latex')
xlim([0,time(end)])
grid on

subplot(3,1,2)
plot(time, D, 'k-','linewidth',1.5)
ylabel('Drag [N]','interpreter','latex')
xlim([0,time(end)])
grid on

subplot(3,1,3)
plot(time, M_air, 'b-','linewidth',1.5)
ylabel('$M_{air}$ [Nm]','interpreter','latex')
xlim([0,time(end)])
xlabel("Time [s]",'interpreter','latex')
grid on

figure()
sgtitle("Airflow Over Wing",'interpreter','latex')

subplot(3,1,1)
plot(time, Va, 'r-','linewidth',1.5)
ylabel('$V_a$ [m/s]','interpreter','latex')
xlim([0,time(end)])
grid on

subplot(3,1,2)
plot(time, Vi, 'k-','linewidth',1.5)
ylabel('$V_i$ [m/s]','interpreter','latex')
xlim([0,time(end)])
grid on

subplot(3,1,3)
plot(time, Vw, 'b-','linewidth',1.5)
hold on
plot(time, Vw_top, 'k--', 'linewidth', 1.5)
plot(time, Vw_bot, 'r--', 'linewidth', 1.5)
ylabel('$V_w$ [m/s]','interpreter','latex')
xlim([0,time(end)])
xlabel("Time (s)")
legend("Average","Top","Bottom")
grid on

figure()
titl = strcat("Misc Angles, $\eta$ = ",num2str(eta));
sgtitle(titl,'interpreter','latex')

subplot(3,1,1)
plot(time, alpha*180/pi, 'r-','linewidth',1.5)
hold on
plot(time, ones(size(time))*pi, 'k--', 'linewidth', 1)
plot(time, ones(size(time))*(-pi), 'k--', 'linewidth', 1)
ylabel('$\alpha$ [deg]','interpreter','latex')
xlim([0,time(end)])
grid on

subplot(3,1,2)
plot(time, alpha_e*180/pi, 'k-','linewidth',1.5)
hold on
if traj_type == "prescribed_aoa"
    plot(time, asin(Vi.*sin(alpha_des)./Va)*180/pi, 'k--', 'linewidth', 1.5)
    plot(time, ones(size(time))*stall_angle, 'g--', 'linewidth', 1)
    legend("Actual","Desired","Stall")
end
% plot(time, ones(size(time))*pi, 'k--', 'linewidth', 1)
% plot(time, ones(size(time))*(-pi), 'k--', 'linewidth', 1)
ylabel('$\alpha_e$ [deg]','interpreter','latex')
xlim([0,time(end)])
grid on
% maxi = find(alpha_e == max(alpha_e));
% plot(time(maxi),alpha_e(maxi),'ro','linewidth',2)
% text(end_time/2,-1,strcat("(\alpha_e)_{SS} = ",num2str(mean(alpha_e((end-100):end))),"-rad"))

subplot(3,1,3)
plot(time, gamma*180/pi, 'b-','linewidth',1.5)
hold on
plot(time, ones(size(time))*90, 'k--', 'linewidth', 1)
plot(time, ones(size(time))*(-90), 'k--', 'linewidth', 1)
ylabel('$\gamma$ [deg]','interpreter','latex')
xlim([0,time(end)])
% xlim([0,18])
xlabel("Time [s]",'interpreter','latex')
grid on

figure()
sgtitle("Thrust Commands",'interpreter','latex')

plot(time, T_top, 'k-', 'linewidth', 1.5)
hold on
plot(time, T_bot, 'r-', 'linewidth', 1.5)
plot(time, 0.5*(T_top + T_bot), 'g-', 'linewidth', 1.5)
xlim([0,time(end)])
xlabel("Time [s]",'interpreter','latex')
ylabel("Thrust [N]",'interpreter','latex')
legend("T_{top}", "T_{bot}", "T_{avg}")
grid on

figure()
sgtitle("Desired Thrust Vector",'interpreter','latex')
plot(time, Fdes(1,:),'-','linewidth',1.5)
hold on
plot(time, Fdes(2,:),'-','linewidth',1.5)
normFdes = zeros(size(time));
for i = 1:length(time)
    normFdes(i) = norm(Fdes(:,i));
end
plot(time, normFdes, 'k--','linewidth',1.5)
xlabel("Time [s]",'interpreter','latex')
ylabel("Force [N]",'interpreter','latex')
legend("F_y","F_z","||F^{net}||")
% title("Thrust Vector from Controller")
grid on

% Comparison with Trim Data

figure()
plot(trim_a_v_Va, trim_alpha_e, 'ro', 'linewidth', 1.5)
hold on
plot(a_v_Va(alpha_e ~= 0), alpha_e(alpha_e ~= 0)*180/pi, 'k-', 'linewidth', 2)
if traj_type == "increasing" || traj_type == "decreasing"
    plot(trim_a_v_Va_shift, trim_alpha_e, 'g*', 'linewidth', 1.5)
    title(strcat("Comparison with Trim Data, acc = ",num2str(a_s),"-$m/s^2$"),'interpreter','latex')
    legend("Trim Condition", "Simulation", "Acceleration-Shifted")
end
xlabel("$a_{v}$",'interpreter','latex')
ylabel("$\alpha_e$ [deg]",'interpreter','latex')
grid on
xlim([0,max(trim_a_v_Va)])
% text(3.1,66,strcat("\eta = ",num2str(eta)))

%% MISC Printouts
if traj_type == "trim"
    fprintf("\nData points of interest: \n")
    fprintf("T_top = %3.4f\n",T_top(end))
    fprintf("T_bot = %3.4f\n",T_bot(end))
    fprintf("theta = %3.4f\n",theta(end))
    fprintf("alpha = %3.4f\n",mean(alpha))
    fprintf("alpha_e = %3.4f\n",mean(alpha_e))
    fprintf("V_w = %3.4f\n",mean(Vw))
    fprintf("V_a = %3.4f\n",mean(Va))
    fprintf("L = %3.4f\n",mean(L))
    fprintf("D = %3.4f\n",mean(D))
    fprintf("M_air = %3.4f\n",mean(M_air))
end