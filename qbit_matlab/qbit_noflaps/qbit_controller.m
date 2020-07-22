%%% This function will output thrust commands based on a nonlinear
%%% geometric controller that tracks orientation [theta] and position [x,z].
%%% Spencer Folk 2020
function [T_top, T_bot, Fdes] = qbit_controller(current_state, desired_state, L, D, M_air, alpha_e, m, Ixx, l)

% INPUTS -
% current_state = [x z theta xdot zdot thetadot]'
% current_time - current time step (time(i))
% L - current lift force
% D - current drag force
% M_air - current pitch moment
% alpha_e - current effective angle of attack
% m - vehicle mass
% Ixx - vehicle inertia about x axis
% l - distance between each rotor

%% Gains and constants
K_p = [5.8*2 , 0 ; 0 , 5.8*3];
% K_d = [8.41*2 , 0 ; 0 , 8.41*3];
K_d = 2*sqrt(K_p(1,1))*eye(2);

K_R = 373.6489/5;
% K_w = 19.333;
K_w = 2*sqrt(K_R);

g = 9.81;
max_motor_thrust = 0.30*9.81*2; % N, determined by estimating max thrust of a single motor and multiplying by 2

% Booleans -- for clarity, true nominally means it will be allowed or enabled.
aero = true;   % This bool determines whether or not the controller is aware of aerodynamic forces
neg_thrust_bool = true;  % Boolean for allowing negative thrusts by the motor (unrealistic, but for debugging purposes)
motor_sat_bool = false;  % If motor thrust goes above saturation limit, this will limit it.


%% Extract current and trajectory states for a given time
y = current_state(1);
z = current_state(2);
theta = current_state(3);
ydot = current_state(4);
zdot = current_state(5);
thetadot = current_state(6);

rT = desired_state;  % Put function here trajectory(current_time)
yT = rT(1);
zT = rT(2);
ydotT = rT(3);
zdotT = rT(4);
ydotdotT = rT(5);
zdotdotT = rT(6);

%% Construct rotation matrices
iRb = [cos(theta) , -sin(theta) ; sin(theta) , cos(theta)];
iRe = [cos(theta - alpha_e) , -sin(theta - alpha_e) ; sin(theta - alpha_e) , cos(theta - alpha_e)];

%% Computing u1

% Compute desired accelerations
rdotdot_des = [ydotdotT ; zdotdotT] - K_d*[ydot - ydotT ; zdot - zdotT] - K_p*[y - yT ; z - zT];

% Now compute Fdes
if aero == true
    Fdes = m*rdotdot_des + [0 ; m*g] - iRe*[-D ; L];
else
    Fdes = m*rdotdot_des + [0 ; m*g];
end

% From there compute u1 (magnitude)
b1 = iRb*[1;0];
u1 = b1'*Fdes;

%% Computing u2

% Solve for b1_des:
b1_des = Fdes/norm(Fdes);

% Compute error
% e_theta = acos(dot(b1,b1_des));
e_theta = -atan2(b1(1)*b1_des(2) - b1(2)*b1_des(1), b1(1)*b1_des(1) + b1(2)*b1_des(2));

% Compute u2:
u2 = Ixx*(-K_R*e_theta - K_w*thetadot) - M_air;

%% Computing actuator outputs
% We can readily solve for thrust by solving this system:
% [A]*[T_top ; T_bot] = [u1 ; u2]

T = inv([1 , 1 ; -l , l])*[u1 ; u2];

T_top = T(1);
T_bot = T(2);

% Negative thrust constraint
if neg_thrust_bool == false
    if T_top < 0
        T_top = 0;
    end
    if T_bot < 0
        T_bot = 0;
    end
end

% Motor saturation constraint
if motor_sat_bool == true
    if T_top >= max_motor_thrust
        T_top = max_motor_thrust;
    end
    if T_bot >= max_motor_thrust
        T_bot = max_motor_thrust;
    end
end


end