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
