function data_out = qbit_simulate_master_sweep(eta, V_s)
%%% Simulating the dynamics of the qbit. This script will establish state
%%% variables, get a trajectory, input that trajectory into a controller to
%%% get commands, and simulate the dynamis subject to those inputs.
%%% Spencer Folk 2020

% clear
close all

aero = true;  % This bool determines whether or not we compute aerodynamic forces
animate = false; % Bool for making an animation of the vehicle.
save_animation = false; % Bool for saving the animation as a gif
traj_type = "trim"; % Type of trajectory, "cubic" or "trim" (for steady state flight)

%% Initialize Constants
in2m = 0.0254;
g = 9.81;
rho = 1.2;
% eta = 0.8;   % Efficiency of the down wash on the wings from the propellers

% CRC 5in prop
% m_airframe = 0.215;
% m_battery = 0.150;
% m = m_airframe + m_battery;
% 
% Ixx = 2.32e-3;
% span = 15*in2m;
% l = 6*in2m;
% chord = 5*in2m;
% R = 2.5*in2m;

% CRC 9in prop (CRC-3 from CAD)
% Compute a scaling factor based on change in wing span:
span = 2*0.508;  % Doubled for biplane set up
l = 0.244;
chord = 0.087;
R = 4.5*in2m;   % Estimated 9in prop

scaling_factor = span/(15*in2m);
% m = (0.3650)*(scaling_factor^3);  % Mass scales with R^3
m = 0.8652;  % This is the value of expression above^ but we want it fixed
%              so we can change the span without worry
% Ixx = (2.32e-3)*(scaling_factor^5);
Ixx = 0.009776460905350; % This is the value of expression above^ but we want it fixed
%                          so we can change the span without worry


%% Generate Airfoil Look-up
% This look up table data will be used to estimate lift, drag, moment given
% the angle of attack and interpolation from this data.
[cl_spline, cd_spline, cm_spline] = aero_fns("naca_0015_experimental_Re-160000.csv");

%% Trajectory Generation
% Generate a trajectory based on the method selected. If cubic, use cubic
% splines. If trim, create a constant speed, trim flight.
% V_s = 1;
end_time = 10;   % Duration of trajectory, this will be rewritten if cubic spline is selected

if traj_type == "cubic"
    waypoints = [0,40; 0,0];
    
    [traj_obj, end_time] = qbit_spline_generator(waypoints, V_s);
    
    % Use this traj_obj to get our desired y,z at a given time t
    traj_obj_dot = fnder(traj_obj,1);
    traj_obj_dotdot = fnder(traj_obj,2);
    
    init_conds = [m*g/2; m*g/2 ; 0];
elseif traj_type == "trim"
    % In the trim mode, we have to have a good initial guess for the trim
    % condition, so that the QBiT isn't too far from the steady state value
    % at the beginning of the trajectory!
    
    % This involves solving for T_top(0), T_bot(0), theta(0)
    x0 = [m*g/2; m*g/2; pi/4];
    fun = @(x) trim_flight(x, cl_spline, cd_spline, cm_spline, m,g,l, chord, span, rho, eta, R, V_s);
    options = optimoptions('fsolve','Display','none');
    [init_conds,~,~,output] = fsolve(fun,x0,options);
    
    iter = output.iterations;
    
    waypoints = [0 , V_s*end_time ; 0, 0];
end

%% Initialize Arrays

% Time vector
dt = 0.01;
t_f = end_time+3;
time = 0:dt:t_f;

% States
y = zeros(size(time));
z = zeros(size(time));
theta = zeros(size(time));

ydot = zeros(size(time));
zdot = zeros(size(time));
thetadot = zeros(size(time));

ydotdot = zeros(size(time));
zdotdot = zeros(size(time));
thetadotdot = zeros(size(time));

% Inputs

T_top = init_conds(1)*ones(size(time));
T_bot = init_conds(2)*ones(size(time));

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
theta(1) = init_conds(3);
y(1) = 0;
z(1) = 0;
if traj_type == "cubic"
    ydot(1) = 0;
elseif traj_type == "trim"
    ydot(1) = V_s;
end
zdot(1) = 0;

% Trajectory state
desired_state = zeros(6,length(time));  % [y, z, ydot, zdot, ydotdot, zdotdot]
desired_state(:,1) = [y(1);z(1);ydot(1);zdot(1);ydotdot(1);zdotdot(1)];

%% Main Simulation

for i = 2:length(time)
    
    % Retrieve the command thrust from desired trajectory
    current_state = [y(i-1), z(i-1), theta(i-1), ydot(i-1), zdot(i-1), thetadot(i-1)];
    current_time = time(i);
    
    % Get our desired state at time(i)
    
    if traj_type == "cubic"
        if time(i) < end_time
            yz_temp = ppval(traj_obj,time(i));
            yzdot_temp = ppval(traj_obj_dot,time(i));
            yzdotdot_temp = ppval(traj_obj_dotdot,time(i));
        else
            yz_temp = waypoints(:,end);
            yzdot_temp = [0;0];
            yzdotdot_temp = [0;0];
        end
    elseif traj_type == "trim"
        yzdotdot_temp = [0 ; 0];
        yzdot_temp = [V_s ; 0];
        yz_temp = [V_s*time(i-1) ; 0];
    end
    
    desired_state(:,i) = [yz_temp' , yzdot_temp' , yzdotdot_temp']; % 6x1
    
    % Find the current airspeed and prop wash speed
    Vi(i-1) = sqrt( ydot(i-1)^2 + zdot(i-1)^2 );
    
    % Compute orientations
    if abs(Vi(i-1)) >= 1e-5
        gamma(i-1) = atan2(zdot(i-1), ydot(i-1));  % Inertial orientation
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
    if Va(i-1) >= 1e-5
        alpha_e(i-1) = asin(Vi(i-1)*sin(alpha(i-1))/Va(i-1));
    else
        alpha_e(i-1) = 0;
    end
    
    if aero == true
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
   
    [T_top(i), T_bot(i), Fdes(:,i)] = qbit_controller(current_state, ...
        desired_state(:,i), L(i-1), D(i-1), M_air(i-1), alpha_e(i-1), m, ...
        Ixx, l);
    
    ydotdot(i) = ((T_top(i) + T_bot(i))*cos(theta(i-1)) - D(i-1)*cos(theta(i-1) - alpha_e(i-1)) - L(i-1)*sin(theta(i-1) - alpha_e(i-1)))/m;
    zdotdot(i) = ( -m*g + (T_top(i) + T_bot(i))*sin(theta(i-1)) - D(i-1)*sin(theta(i-1) - alpha_e(i-1)) + L(i-1)*cos(theta(i-1) - alpha_e(i-1)))/m;
    thetadotdot(i) = (M_air(i-1) + l*(T_bot(i) - T_top(i)))/Ixx;
    
    % Euler integration
    ydot(i) = ydot(i-1) + ydotdot(i)*dt;
    zdot(i) = zdot(i-1) + zdotdot(i)*dt;
    thetadot(i) = thetadot(i-1) + thetadotdot(i)*dt;
    
    y(i) = y(i-1) + ydot(i)*dt;
    z(i) = z(i-1) + zdot(i)*dt;
    theta(i) = theta(i-1) + thetadot(i)*dt;
    
end

L(end) = L(end-1);
D(end) = D(end-1);
M_air(end) = M_air(end-1);

Va(end) = Va(end-1);
Vi(end) = Vi(end-1);
Vw(end) = Vw(end-1);

alpha(end) = alpha(end-1);
alpha_e(end) = alpha_e(end-1);
gamma(end) = gamma(end-1);

T_top(end) = T_top(end-1);
T_bot(end) = T_bot(end-1);

Fdes(:,end) = Fdes(:,end-1);
Fdes(:,1) = Fdes(:,2);


%% Animation

if animate == true
    h = figure();
    qbit_animate_trajectory(h, time,[y ; z ; theta], desired_state(1,:), desired_state(2,:),Fdes,l, save_animation)
    hold on
    plot(waypoints(1,:),waypoints(2,:),'ko','linewidth',2)
    axis equal
end

a_v_analytic = cot(alpha_e(end))/(Cd + Cl*cot(alpha_e(end)));
a_v_Va = (1/2)*rho*(chord*span)*Va(end)^2/(m*g);
a_v_Vi = (1/2)*rho*(chord*span)*Vi(end)^2/(m*g);
T_avg = (1/2)*(T_top(end) + T_bot(end));
data_out = [eta V_s T_top(end) T_bot(end) T_avg theta(end) alpha(end) alpha_e(end) Vw(end) ...
            Va(end) L(end) D(end) M_air(end) Cl Cd Cm iter a_v_analytic a_v_Va a_v_Vi std(alpha_e)];

% fprintf("\nData points of interest: \n")
% fprintf("T_top = %3.4f\n",T_top(end))
% fprintf("T_bot = %3.4f\n",T_bot(end))
% fprintf("theta = %3.4f\n",theta(end))
% fprintf("alpha = %3.4f\n",mean(alpha))
% fprintf("alpha_e = %3.4f\n",mean(alpha_e))
% fprintf("V_w = %3.4f\n",mean(Vw))
% fprintf("V_a = %3.4f\n",mean(Va))
% fprintf("L = %3.4f\n",mean(L))
% fprintf("D = %3.4f\n",mean(D))
% fprintf("M_air = %3.4f\n",mean(M_air))
% qbit_main_plotting

end
