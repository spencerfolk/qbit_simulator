%%% Simulating the dynamics of the qbit. This script will establish state
%%% variables, get a trajectory, input that trajectory into a controller to
%%% get commands, and simulate the dynamis subject to those inputs.
%%% Spencer Folk 2020

clear
clc
close all

aero = true;  % This bool determines whether or not we compute aerodynamic forces
animate = false; % Bool for making an animation of the vehicle.
save_animation = false; % Bool for saving the animation as a gif
traj_type = "const_height"; % Type of trajectory:
%                           "cubic",
%                           "trim" (for steady state flight),
%                           "increasing" (const acceleration)
%                           "decreasing" (const decelleration)
%                           "const_height" (constant height)
%                           "stepP" (step response in position at hover)
%                           "stepA" (step response in angle at hover)

%% Initialize Constants
in2m = 0.0254;
g = 9.81;
rho = 1.2;
stall_angle = 10;  % deg, identified from plot of cl vs alpha
dt = 0.01;   % Simulation time step

eta = 0.0;   % Efficiency of the down wash on the wings from the propellers

% Ritz tailsitter
% m = 0.150;
% Iyy = 2.32e-3;
% span = 15*in2m;
% l = 6*in2m;
% chord = 5*in2m;
% R = 2.5*in2m;

% UMD QBiT
% m = 3.76;
% Iyy = 2.32e-1;  % Estimated with scaling laws based on mass and chord
% span = 1.02;
% chord = 0.254;
% l = 19*in2m;
% R = 15/2*in2m;

% UMD QBiT Refined (thrust from motors don't even balance the weight...)
% m = 1.264;
% Iyy = 2.32e-2;  % Estimated with scaling laws based on mass and chord
% span = 1.02;
% chord = 0.254;
% l = 19*in2m;
% R = 15/2*in2m;

% CRC 5in prop
% m_airframe = 0.215;
% m_battery = 0.150;
% m = m_airframe + m_battery;
% 
% Iyy = 2.32e-3;
% span = 15*in2m;
% l = 6*in2m;
% chord = 5*in2m;
% R = 2.5*in2m;

% CRC 9in prop (CRC-3 from CAD)
% Compute a scaling factor based on change in wing span:
span = 0.508;
l = 0.244;
chord = 0.087;
R = 4.5*in2m;   % Estimated 9in prop

scaling_factor = span/(15*in2m);
m = (0.3650)*(scaling_factor^3);  % Mass scales with R^3
Iyy = (2.32e-3)*(scaling_factor^5);

% Aero coefficients that act on the lift/drag coefficients to match that of
% "experimental" (in reality, simulation) data from XFOIL

% c0 = 4.80914;  % coeff acts as a scaling factor on Cl, Cm
% c1 = 0.02;     % coeff acts as a shifting factor on Cd
% c2 = 0.61929;  % coeff acts as a scaling factor on Cd

% c0 = 1;
% c1 = 0;
% c2 = 1;

%% Generate Airfoil Look-up
% This look up table data will be used to estimate lift, drag, moment given
% the angle of attack and interpolation from this data.
[cl_spline, cd_spline, cm_spline] = aero_fns("naca_0015_experimental_Re-160000.csv");

%% Trajectory Generation
% Generate a trajectory based on the method selected. If cubic, use cubic
% splines. If trim, create a constant speed, trim flight.
V_s = 50;  % Target velocity
end_time = 10;   % Duration of trajectory, this will be rewritten if cubic spline is selected

if traj_type == "cubic"
    waypoints = [0,40; 0,0];
    % waypoints = [0,0,10 ; 0,10,10];  % aggressive maneuver
    % waypoints = [0,20,40 ; 0,0,0];  % Straight line horizontal trajectory
    % waypoints = [0,80,160 ; 0,0,0];  % Straight line horizontal trajectory, longer
    % waypoints = [0,0,0 ; 0, 20, 40]; % Straight line vertical trajectory
    % waypoints = [0,10,40 ; 0,10,10];  % Larger distance shows off lift benefit
    % waypoints = [0,20,40 ; 0,5,10]; % diagonal
    % waypoints = [0,10,20,30,40 ; 0,10,0,-10,0]; % zigzag
    % waypoints = [0,0 ; 0, -10];  % Drop
    % waypoints = [0,0 ; 0, 10];  % rise
    
    [traj_obj, end_time] = qbit_trajectory_generator(waypoints, V_s);
    
    % Use this traj_obj to get our desired x,z at a given time t
    traj_obj_dot = fnder(traj_obj,1);
    traj_obj_dotdot = fnder(traj_obj,2);
    
    init_conds = [m*g/2; m*g/2 ; pi/2];
    
    % Time vector
    t_f = end_time;
    time = 0:dt:t_f;
    
    fprintf("\nTrajectory type: Cubic Spline")
    fprintf("\n-----------------------------\n")
    
elseif traj_type == "trim"
    % In the trim mode, we have to have a good initial guess for the trim
    % condition, so that the QBiT isn't too far from the steady state value
    % at the beginning of the trajectory!
    
    % This involves solving for T_top(0), T_bot(0), theta(0)
    x0 = [m*g/2; m*g/2; pi/4];
    fun = @(x) trim_flight(x, cl_spline, cd_spline, cm_spline, m,g,l, chord, span, rho, eta, R, V_s);
    %     options = optimoptions('fsolve','Display','iter');
    options = optimoptions('fsolve','Display','none','PlotFcn',@optimplotfirstorderopt);
    [init_conds,~,~,output] = fsolve(fun,x0,options);
    
    %     output.iterations
    
    % Time vector
    t_f = end_time;
    time = 0:dt:t_f;
    
    fprintf("\nTrajectory type: Trim")
    fprintf("\n---------------------\n")
    fprintf("\nTrim estimate solved: \n")
    fprintf("\nT_top = %3.4f",init_conds(1))
    fprintf("\nT_bot = %3.4f",init_conds(2))
    fprintf("\ntheta   = %3.4f\n",init_conds(3))
    
    waypoints = [0 , V_s*end_time ; 0, 0];
elseif traj_type == "increasing"
    % In this mode we use a constant acceleration to go from hover to V_s.
    % Therefore just set the initial condition to 0.
    
    init_conds = [m*g/2; m*g/2 ; pi/2];
    V_end = V_s;
    a_s = 3;  % m/s^2, acceleration used for transition
    
    end_time = V_end/a_s;
    
    % Time vector
    t_f = end_time;
    time = 0:dt:t_f;
    
    fprintf("\nTrajectory type: Linear Increasing")
    fprintf("\n----------------------------------\n")
    
elseif traj_type == "decreasing"
    % Constant deceleration from some beginning speed, V_start, to hover.
    
    % Need to solve for an estimate of trim flight:
    x0 = [m*g/2; m*g/2; pi/4];
    fun = @(x) trim_flight(x, cl_spline, cd_spline, cm_spline, m,g,l, chord, span, rho, eta, R, V_s);
    %     options = optimoptions('fsolve','Display','iter');
    options = optimoptions('fsolve','Display','none','PlotFcn',@optimplotfirstorderopt);
    [init_conds,~,~,output] = fsolve(fun,x0,options);
    
    V_start = V_s;
    a_s = 1;   % m/s^2, decelleration used for transition
    end_time = V_start/a_s;
    
    % Time vector
    t_f = end_time;
    time = 0:dt:t_f;
    
    fprintf("\nTrajectory type: Linear Decreasing")
    fprintf("\n----------------------------------\n")
    
elseif traj_type == "const_height"
    % If it's constant height, design a desired AoA function
    % return a corresponding v(t), a(t), x/z(t) from that.
    
    % Need to solve for an estimate of trim flight:
    x0 = [m*g/2; m*g/2; pi/4];
    fun = @(x) trim_flight(x, cl_spline, cd_spline, cm_spline, m,g,l, chord, span, rho, eta, R, V_s);
    %     options = optimoptions('fsolve','Display','iter');
    options = optimoptions('fsolve','Display','none');
    [init_conds,~,~,output] = fsolve(fun,x0,options);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Constructing alpha_des:
    alpha_f = init_conds(end);  % Final value for alpha_des
    alpha_i = pi/2;  % Initial value for alpha_des
    
    alpha_traj_type = "linear";
    aoa_rate = 5*(pi/180);  % Rate of change of AoA, first number in degrees
    if alpha_traj_type == "linear"
        end_time = abs(alpha_f - alpha_i)/aoa_rate;
        time = 0:dt:end_time;
        
        alpha_des = alpha_i - aoa_rate*time;
    elseif alpha_traj_type == "parabolic"
        end_time = 30;  % s
        time = 0:dt:end_time;
        
        alpha_des = alpha_i + ((alpha_f-alpha_i)/end_time^2)*time.^2;
        
    end
    
    % Get temp trajectory variables and save them
    accel_bool = true;  % Consider acceleration when generating the trajectory
    [x_des, xdot_des, xdotdot_des]=const_height_traj_generator(dt,time,alpha_des,cl_spline, cd_spline,rho,m,g,chord,span, accel_bool);
    
    fprintf("\nTrajectory type: Continuous Constant Height")
    fprintf("\n-------------------------------------------\n")
    
elseif traj_type == "stepP" || traj_type == "stepA"
    % For step hover, this is easy, we just need to set our trajectory to
    % zeros for all time
    time = 0:dt:end_time;
    
else
    error("Incorrect trajectory type -- check traj_type variable")
end


%% Initialize Arrays

%%% TIME IS INTITALIZED IN THE SECTION ABOVE

% States
x = zeros(size(time));
z = zeros(size(time));
theta = zeros(size(time));

xdot = zeros(size(time));
zdot = zeros(size(time));
thetadot = zeros(size(time));

xdotdot = zeros(size(time));
zdotdot = zeros(size(time));
thetadotdot = zeros(size(time));

% Inputs

if traj_type == "trim" || traj_type == "decreasing"
    T_top = init_conds(1)*ones(size(time));
    T_bot = init_conds(2)*ones(size(time));
    
else
    T_top = m*g*ones(size(time));
    T_bot = m*g*ones(size(time));
end
% Misc Variables (also important)
alpha = zeros(size(time));
alpha_e = zeros(size(time));
gamma = zeros(size(time));

L = zeros(size(time));
D = zeros(size(time));
M_air = zeros(size(time));

Vi = zeros(size(time));
Va = zeros(size(time));
Vw = zeros(size(time));

% Bookkeeping the airflow over the top and bottom wings
Vw_top = zeros(size(time));
Vw_bot = zeros(size(time));

Fdes = zeros(2,length(time));  % Desired force vector

% Power consumption
Ptop = zeros(size(time));
Pbot = zeros(size(time));

% Initial conditions:
theta(1) = pi/2;
x(1) = 0;
z(1) = 0;
if traj_type == "trim" || traj_type == "decreasing"
    xdot(1) = V_s;
    theta(1) = init_conds(3);
elseif traj_type == "stepP"
    x(1) = 0;
    z(1) = -1;
elseif traj_type == "stepA"
    theta(1) = pi/2 - pi/4;
end
zdot(1) = 0;

% Trajectory state
desired_state = zeros(6,length(time));  % [x, z, xdot, zdot, xdotdot, zdotdot]
desired_state(:,1) = [x(1);z(1);xdot(1);zdot(1);xdotdot(1);zdotdot(1)];

%% Main Simulation

for i = 2:length(time)
    
    % Retrieve the command thrust from desired trajectory
    current_state = [x(i-1), z(i-1), theta(i-1), xdot(i-1), zdot(i-1), thetadot(i-1)];
    current_time = time(i);
    
    % Get our desired state at time(i)
    
    if traj_type == "cubic"
        if time(i) < end_time
            xz_temp = ppval(traj_obj,time(i));
            xzdot_temp = ppval(traj_obj_dot,time(i));
            xzdotdot_temp = ppval(traj_obj_dotdot,time(i));
        else
            xz_temp = waypoints(:,end);
            xzdot_temp = [0;0];
            xzdotdot_temp = [0;0];
        end
    elseif traj_type == "trim"
        xzdotdot_temp = [0 ; 0];
        xzdot_temp = [V_s ; 0];
        xz_temp = [V_s*time(i-1) ; 0];
    elseif traj_type == "increasing"
        if time(i) < end_time
            xzdotdot_temp = [a_s ; 0];
            xzdot_temp = [a_s*time(i-1) ; 0];
            xz_temp = [(1/2)*a_s*(time(i-1)^2) ; 0];
        else
            xzdotdot_temp = [0 ; 0];
            xzdot_temp = [V_s ; 0];
            xz_temp = [(1/2)*a_s*(end_time^2) + V_s*(time(i) - end_time) ; 0];
        end
    elseif traj_type == "decreasing"
        xzdotdot_temp = [-a_s ; 0];
        xzdot_temp = [V_start-a_s*time(i-1) ; 0];
        xz_temp = [V_start*time(i-1)-(1/2)*a_s*(time(i-1)^2) ; 0];
    elseif traj_type == "const_height"
        % Take the trajectory and read from there
        
        xzdotdot_temp = [xdotdot_des(i); 0];
        xzdot_temp = [xdot_des(i); 0];
        xz_temp = [x_des(i); 0];
    elseif traj_type == "stepA" || traj_type == "stepP"
        xzdotdot_temp = [0 ; 0];
        xzdot_temp = [0 ; 0];
        xz_temp = [0 ; 0];
    end
    
    desired_state(:,i) = [xz_temp' , xzdot_temp' , xzdotdot_temp']; % 6x1
    
    % Find the current airspeed and prop wash speed
    Vi(i-1) = sqrt( xdot(i-1)^2 + zdot(i-1)^2 );
    
    % Compute orientations
    if abs(Vi(i-1)) >= 1e-10
        gamma(i-1) = atan2(zdot(i-1), xdot(i-1));  % Inertial orientation
    else
        gamma(i-1) = 0;
    end
    alpha(i-1) = theta(i-1) - gamma(i-1);  % Angle of attack strictly based on inertial speed
    
    % Get prop wash over wing via momentum theory
    T_avg = 0.5*(T_top(i-1) + T_bot(i-1));
    
    %     Vw(i-1) = 1.2*sqrt( T_avg/(8*rho*pi*R^2) );
    Vw(i-1) = eta*sqrt( (Vi(i-1)*cos(theta(i-1)-gamma(i-1)))^2 + (T_avg/(0.5*rho*pi*R^2)) );
    Vw_top(i-1) = eta*sqrt( (Vi(i-1)*cos(theta(i-1)-gamma(i-1)))^2 + (T_top(i-1)/(0.5*rho*pi*R^2)) );
    Vw_bot(i-1) = eta*sqrt( (Vi(i-1)*cos(theta(i-1)-gamma(i-1)))^2 + (T_bot(i-1)/(0.5*rho*pi*R^2)) );

    % Compute true airspeed over the wings using law of cosines
    Va(i-1) = sqrt( Vi(i-1)^2 + Vw(i-1)^2 + 2*Vi(i-1)*Vw(i-1)*cos( alpha(i-1)) );
    
    % Use this check to avoid errors in asin
    if Va(i-1) >= 1e-10
        alpha_e(i-1) = asin(Vi(i-1)*sin(alpha(i-1))/Va(i-1));
    else
        alpha_e(i-1) = 0;
    end
    
    if aero == true
        %         [Cl, Cd, Cm] = aero_fns(c0, c1, c2, alpha_e(i-1));
        %         Cl = interp1(alpha_data, cl_data, alpha_e(i-1)*180/pi);
        %         Cd = interp1(alpha_data, cd_data, alpha_e(i-1)*180/pi);
        %         Cm = interp1(alpha_data, cm_data, alpha_e(i-1)*180/pi);
        Cl = ppval(cl_spline, alpha_e(i-1)*180/pi);
        Cd = ppval(cd_spline, alpha_e(i-1)*180/pi);
        Cm = ppval(cm_spline, alpha_e(i-1)*180/pi);
    else
        Cl = 0;
        Cd = 0;
        Cm = 0;
        alpha_e(i-1) = alpha(i-1);  % The traditional angle of attack is now true.
    end
    
    L(i-1) = 0.5*rho*Va(i-1)^2*(chord*span)*Cl;
    D(i-1) = 0.5*rho*Va(i-1)^2*(chord*span)*Cd;
    M_air(i-1) = 0.5*rho*Va(i-1)^2*(chord*span)*chord*Cm;
    
    %     fprintf("\nIndex = %d",i)
    %     if i == 226
    %         xxx = 50;
    %     end
    [T_top(i), T_bot(i), Fdes(:,i)] = qbit_controller(current_state, ...
        desired_state(:,i), L(i-1), D(i-1), M_air(i-1), alpha_e(i-1), m, ...
        Iyy, l);
    
    xdotdot(i) = ((T_top(i) + T_bot(i))*cos(theta(i-1)) - D(i-1)*cos(theta(i-1) - alpha_e(i-1)) - L(i-1)*sin(theta(i-1) - alpha_e(i-1)))/m;
    zdotdot(i) = ( -m*g + (T_top(i) + T_bot(i))*sin(theta(i-1)) - D(i-1)*sin(theta(i-1) - alpha_e(i-1)) + L(i-1)*cos(theta(i-1) - alpha_e(i-1)))/m;
    thetadotdot(i) = (M_air(i-1) + l*(T_bot(i) - T_top(i)))/Iyy;
    
    % Euler integration
    xdot(i) = xdot(i-1) + xdotdot(i)*dt;
    zdot(i) = zdot(i-1) + zdotdot(i)*dt;
    thetadot(i) = thetadot(i-1) + thetadotdot(i)*dt;
    
    x(i) = x(i-1) + xdot(i)*dt;
    z(i) = z(i-1) + zdot(i)*dt;
    theta(i) = theta(i-1) + thetadot(i)*dt;
    
end

% Padding
L(end) = L(end-1);
D(end) = D(end-1);
M_air(end) = M_air(end-1);

Va(end) = Va(end-1);
Vi(end) = Vi(end-1);
Vw(end) = Vw(end-1);
Vw_top(end) = Vw_top(end-1);
Vw_bot(end) = Vw_bot(end-1);

alpha(end) = alpha(end-1);
alpha_e(end) = alpha_e(end-1);
gamma(end) = gamma(end-1);

T_top(end) = T_top(end-1);
T_bot(end) = T_bot(end-1);

Fdes(:,end) = Fdes(:,end-1);
Fdes(:,1) = Fdes(:,2);

alpha_e_startidx = find(alpha_e ~= 0,1,'first');
alpha_e(1:(alpha_e_startidx-1)) = alpha_e(alpha_e_startidx);

Va(1) = Va(2);
Vw(1) = Vw(2);
Vw_top(1) = Vw_top(2);
Vw_bot(1) = Vw_bot(2);
T_top(1) = T_top(2);
T_bot(1) = T_bot(2);
xdotdot(1) = xdotdot(2);
zdotdot(1) = zdotdot(2);

a_v_Va = (1/2)*rho*(chord*span)*Va.^2/(m*g);

%% Animation

if animate == true
    h = figure();
    qbit_animate_trajectory(h, time,[x ; z ; theta], desired_state(1,:), desired_state(2,:),Fdes,l, save_animation)
    
    if traj_type == "cubic"
        hold on
        plot(waypoints(1,:),waypoints(2,:),'ko','linewidth',2)
    end
    axis equal
end

%% Trim Comparison
% Take the data from the trim analysis for the particular flight condition
% we're interested in (based on eta)

table = readtable("prop_wash_sweep.csv");
trim_eta = table.eta;
trim_alpha_e = table.alpha_e(trim_eta == eta);
trim_theta = table.theta(trim_eta == eta);
trim_alpha = table.alpha(trim_eta == eta);
trim_Vi = table.V_i(trim_eta == eta);
trim_a_v_Va = table.a_v_Va(trim_eta == eta);
trim_Cl = table.Cl(trim_eta == eta);
trim_Cd = table.Cd(trim_eta == eta);

if traj_type == "increasing" || traj_type == "decreasing"
    % Apply the acceleration shift based on derivation of a_v relationship
    % with alpha.
    if traj_type == "decreasing"
        a_s = -a_s;
    end
    trim_a_v_Va_shift = trim_a_v_Va - (a_s/g)./(trim_Cd + trim_Cl.*cot(trim_alpha_e*pi/180));
end

%% Plotting

% States
figure()
sgtitle("States",'interpreter','latex')

subplot(3,1,1)
plot(time, x, 'r-','linewidth',1.5)
hold on
plot(time, desired_state(1,:), 'k--', 'linewidth',1)
ylabel('x [m]','interpreter','latex')
xlim([0,time(end)])
grid on

subplot(3,1,2)
plot(time, z, 'k-','linewidth',1.5)
hold on
plot(time, desired_state(2,:), 'k--', 'linewidth',1)
ylabel('z [m]','interpreter','latex')
xlim([0,time(end)])
grid on

subplot(3,1,3)
plot(time, theta, 'b-','linewidth',1.5)
hold on
plot(time, ones(size(time))*pi/2, 'k--', 'linewidth', 1)
ylabel('$\theta$ [rad]','interpreter','latex')
xlim([0,time(end)])
xlabel("Time [s]",'interpreter','latex')
grid on

figure()
sgtitle("State Derivatives",'interpreter','latex')

subplot(3,1,1)
plot(time, xdot, 'r-','linewidth',1.5)
hold on
plot(time, desired_state(3,:), 'k--', 'linewidth',1)
ylabel('$\dot{x}$ [m/s]','interpreter','latex')
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
plot(time, thetadot, 'b-','linewidth',1.5)
ylabel('$\dot{\theta}$ [rad/s]','interpreter','latex')
xlim([0,time(end)])
xlabel("Time [s]",'interpreter','latex')
grid on

figure()
sgtitle("Body Acceleration",'interpreter','latex')

subplot(2,1,1)
plot(time,xdotdot,'r-','linewidth',1.5)
hold on
plot(time,desired_state(5,:),'k--','linewidth',1.5)
ylabel("$\ddot{x}$ [$m/s^2$]",'interpreter','latex')
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
plot(time, alpha, 'r-','linewidth',1.5)
hold on
plot(time, ones(size(time))*pi, 'k--', 'linewidth', 1)
plot(time, ones(size(time))*(-pi), 'k--', 'linewidth', 1)
ylabel('$\alpha$ [rad]','interpreter','latex')
xlim([0,time(end)])
grid on

subplot(3,1,2)
plot(time, alpha_e, 'k-','linewidth',1.5)
hold on
if traj_type == "const_height"
    plot(time, asin(Vi.*sin(alpha_des)./Va), 'k--', 'linewidth', 1.5)
    plot(time, ones(size(time))*stall_angle*pi/180, 'g--', 'linewidth', 1)
    legend("Actual","Desired","Stall")
end
% plot(time, ones(size(time))*pi, 'k--', 'linewidth', 1)
% plot(time, ones(size(time))*(-pi), 'k--', 'linewidth', 1)
ylabel('$\alpha_e$ [rad]','interpreter','latex')
xlim([0,time(end)])
grid on
% maxi = find(alpha_e == max(alpha_e));
% plot(time(maxi),alpha_e(maxi),'ro','linewidth',2)
% text(end_time/2,-1,strcat("(\alpha_e)_{SS} = ",num2str(mean(alpha_e((end-100):end))),"-rad"))

subplot(3,1,3)
plot(time, gamma, 'b-','linewidth',1.5)
hold on
plot(time, ones(size(time))*pi, 'k--', 'linewidth', 1)
plot(time, ones(size(time))*(-pi), 'k--', 'linewidth', 1)
ylabel('$\gamma$ [rad]','interpreter','latex')
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
legend("F_x","F_z","||F^{net}||")
% title("Thrust Vector from Controller")
grid on

% Comparison with Trim Data

figure()
plot(trim_a_v_Va, trim_alpha_e, 'ro', 'linewidth', 1.5)
hold on
plot(a_v_Va(alpha_e ~= 0), alpha_e(alpha_e ~= 0)*180/pi, 'k-', 'linewidth', 2)
if traj_type == "increasing" || traj_type == "decreasing"
    plot(trim_a_v_Va_shift, trim_alpha_e, 'g*', 'linewidth', 1.5)
    title(strcat("Comparison with Trim Data, acc = ",num2str(a_s),"-m/s^2"))
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

