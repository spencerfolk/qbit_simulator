figure()
sgtitle("Forward Airspeed vs Time",'interpreter','latex')
% plot(time, xdot, 'r-','linewidth',1.5)
% hold on
plot(time, desired_state(3,:), 'k--', 'linewidth',1)
ylabel('$\dot{x}$ [m/s]','interpreter','latex')
xlabel('Time [s]','interpreter','latex')
xlim([0,time(end)])
% legend("Actual","Desired")
grid on

figure()
sgtitle("Body Orientation vs Time",'interpreter','latex')
plot(time,theta*180/pi,'b-','linewidth',1.5)
xlim([0,time(end)])
xlabel('Time [s]','interpreter','latex')
ylabel('$\theta$ [deg]','interpreter','latex')
if traj_type == "const_height"
    hold on
    plot(time,alpha_des*180/pi,'k--','linewidth',1.5)
    legend("Actual","Desired")
end
grid on

figure()
sgtitle("Forward Acceleration vs Time",'interpreter','latex')
plot(time,xdotdot,'r-','linewidth',1.5)
hold on
plot(time,desired_state(5,:),'k--','linewidth',1.5)
ylabel("$\ddot{x}$ [$m/s^2$]",'interpreter','latex')
xlim([0,time(end)]);
xlabel('Time [s]','interpreter','latex')
legend("Actual", "Desired")
grid on

figure()
plot(time, theta, 'b-','linewidth',1.5)
hold on
if traj_type == "stepA_FF"
    sgtitle("Pitch Step Response in Forward Flight",'interpreter','latex')
    plot(time,init_conds(3)*ones(size(time)),'k--','linewidth',1)
    ylim([min(theta),1])
else
    sgtitle("Pitch Step Response at Hover",'interpreter','latex')
    plot(time, ones(size(time))*pi/2, 'k--', 'linewidth', 1)
    ylim([0,2])
end
ylabel('$\theta$ [rad]','interpreter','latex')
xlim([0,time(end)])
xlabel("Time [s]",'interpreter','latex')
legend("Actual","Desired")
grid on

figure()
sgtitle("Airspeed Step Response",'interpreter','latex')
plot(time, Vi, 'k-','linewidth',1.5)
hold on
plot(time, desired_state(3,:), 'k--', 'linewidth',1)
ylabel('$V_i$ [m/s]','interpreter','latex')
xlabel("Time [s]",'interpreter','latex')
xlim([0,time(end)])
ylim([10,17])
legend("Actual","Desired")
grid on

% States
figure()
if traj_type == "const_height"
    sgtitle("Horizontal Transition: Prescribed AoA",'interpreter','latex')
else
    sgtitle("Horizontal Transition: Constant Acceleration",'interpreter','latex')
end
subplot(3,1,1)
plot(time, desired_state(1,:), 'k--', 'linewidth',1.5)
hold on
ylabel('x [m]','interpreter','latex')
xlim([0,time(end)])
grid on

subplot(3,1,2)
plot(time, desired_state(3,:), 'k--', 'linewidth',1.5)
ylabel('$\dot{x}$ [m/s]','interpreter','latex')
xlim([0,time(end)])
grid on

subplot(3,1,3)
plot(time,desired_state(5,:),'k--','linewidth',1.5)
ylabel("$\ddot{x}$ [$m/s^2$]",'interpreter','latex')
xlim([0,time(end)]);
grid on
xlabel("Time [s]",'interpreter','latex')

