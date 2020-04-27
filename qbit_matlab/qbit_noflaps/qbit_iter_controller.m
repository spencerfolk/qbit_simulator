%%% This function will output thrust commands based on a nonlinear
%%% geometric controller that tracks orientation [theta] and position [x,z].
%%% Notably, this particular controller iterates to converge on a T_top and
%%% T_bot that properly anticipates lift and drag at the new state
%%% Spencer Folk 2020
function [T_top, T_bot, Fdes] = qbit_iter_controller(current_state, desired_state, L, D, M_air, alpha_e, m, Iyy, l)

% INPUTS -
% current_state = [x z theta xdot zdot thetadot]'
% current_time - current time step (time(i))
% L - current lift force
% D - current drag force
% M_air - current pitch moment
% alpha_e - current effective angle of attack
% m - vehicle mass
% Iyy - vehicle inertia about y axis
% l - distance between each rotor

in2m = 0.0254;

c0 = 4.80914;  % coeff acts as a scaling factor on Cl, Cm
c1 = 0.02;     % coeff acts as a shifting factor on Cd
c2 = 0.61929;  % coeff acts as a scaling factor on Cd

chord = 5*in2m;
span = 15*in2m;
R = 2.5*in2m;

rho = 1.2;

%% Gains and constants
K_p = [5.8*2 , 0 ; 0 , 5.8*3];
% K_d = [8.41*2 , 0 ; 0 , 8.41*3];
K_d = 2*sqrt(K_p(1,1));

K_R = 373.6489;
% K_w = 19.333;
K_w = 2*sqrt(K_R);

g = 9.81;
max_motor_thrust = 0.15*9.81*2; % N, determined by estimating max thrust of a single motor and multiplying by 2

% Booleans -- for clarity, true nominally means it will be allowed or enabled.
aero = true;   % This bool determines whether or not the controller is aware of aerodynamic forces
neg_thrust_bool = false;  % Boolean for allowing negative thrusts by the motor (unrealistic, but for debugging purposes)
motor_sat_bool = true;  % If motor thrust goes above saturation limit, this will limit it.


%% Extract current and trajectory states for a given time
x = current_state(1);
z = current_state(2);
theta = current_state(3);
xdot = current_state(4);
zdot = current_state(5);
thetadot = current_state(6);

rT = desired_state;
xT = rT(1);
zT = rT(2);
xdotT = rT(3);
zdotT = rT(4);
xdotdotT = rT(5);
zdotdotT = rT(6);

%% Construct rotation matrices
iRb = [cos(theta) , -sin(theta) ; sin(theta) , cos(theta)];
iRe = [cos(theta - alpha_e) , -sin(theta - alpha_e) ; sin(theta - alpha_e) , cos(theta - alpha_e)];

%% First iteration

% Compute u1
rdotdot_des = [xdotdotT ; zdotdotT] - K_d*[xdot - xdotT ; zdot - zdotT] - K_p*[x - xT ; z - zT];

if aero == true
    Fdes = m*rdotdot_des + [0 ; m*g] - iRe*[-D ; L];
else
    Fdes = m*rdotdot_des + [0 ; m*g];
end

b1 = iRb*[1;0];
u1 = b1'*Fdes;

b1_des = Fdes/norm(Fdes);

% Get desired theta
theta_des = atan2(b1_des(2),b1_des(1));

% Compute error
e_theta = -atan2(b1(1)*b1_des(2) - b1(2)*b1_des(1), b1(1)*b1_des(1) + b1(2)*b1_des(2));

% Compute u2:
u2 = Iyy*(-K_R*e_theta - K_w*thetadot) - M_air;

T = inv([1 , 1 ; -l , l])*[u1 ; u2];

T_top = T(1);
T_bot = T(2);

%% Iterate!

% Save values of T_top and T_bot for debugging... can comment if too slow
T_top_hist = [T_top];
T_bot_hist = [T_bot];

error = 1e5;     % This error is the difference between T_top/T_bot for each iteration
idx = 0;  % Safety idx just in case we get stuck in infinite loop

if aero == true
    while error > 1e-5 && idx <= 1000
        % In each iteration, recompute L, D, M_air
        % Then recompute Fdes and theta_des --> T_top, T_bot
        % Repeat until T_top and T_bot are no longer changing
        
        % Recompute airspeeds given new orientation (theta_des) and T_top, T_bot
        T_avg = 0.5*(T_top + T_bot);
        Vi = sqrt(xdotT^2 + zdotT^2);
        gamma = atan2(zdotT, xdotT);
        alpha = theta_des - gamma;
        Vw = sqrt( (Vi*cos(theta_des-gamma))^2 + (T_avg/(0.5*rho*pi*R^2)) );
        Va = sqrt( Vi^2 + Vw^2 + 2*Vi*Vw*cos(alpha) );
        
        if Va >= 1e-6 && Vi < Va
            alpha_e = asin(Vi*sin(alpha)/Va);
        else
            alpha_e = 0;
        end
        
        [Cl, Cd, Cm] = aero_fns(c0, c1, c2, alpha_e);
        
        L_new = 0.5*rho*Va^2*(chord*span)*Cl;
        D_new = 0.5*rho*Va^2*(chord*span)*Cd;
        M_air_new = 0.5*rho*Va^2*(chord*span)*chord*Cm*0;
        
        % Now recompute Fdes based on this new L, D, and M_air
        iRe_des = [cos(theta_des - alpha_e) , -sin(theta_des - alpha_e) ; sin(theta_des - alpha_e) , cos(theta_des - alpha_e)];
        
        Fdes = m*rdotdot_des + [0 ; m*g] - iRe_des*[-D_new ; L_new];
        
        b1 = iRb*[1;0];
        u1 = b1'*Fdes;
        
        b1_des = Fdes/norm(Fdes);
        
        % Get desired theta
        theta_des = atan2(b1_des(2),b1_des(1));
        
        % Compute error
        e_theta = -atan2(b1(1)*b1_des(2) - b1(2)*b1_des(1), b1(1)*b1_des(1) + b1(2)*b1_des(2));
        
        % Compute u2:
        u2 = Iyy*(-K_R*e_theta - K_w*thetadot) - M_air_new;
        
        T = inv([1 , 1 ; -l , l])*[u1 ; u2];
        
        T_top_new = T(1);
        T_bot_new = T(2);
        
        T_top_hist = [T_top_hist ; T_top_new];
        T_bot_hist = [T_bot_hist ; T_bot_new];
        
        error = abs(max(T_top_new - T_top, T_bot_new - T_bot));
        error_top = T_top_new - T_top;
        error_bot = T_bot_new - T_bot;
        
        T_top = T_top_new;
        T_bot = T_bot_new;
        
        idx = idx + 1;
    end
end
fprintf("\tConvergent in %d Iterations",idx)

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

if T_bot <= 0
    temp = 1;
end


end