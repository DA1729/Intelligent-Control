% Parameters
l1 = 1;  % Length of link 1
l2 = 1;  % Length of link 2
m1 = 1;  % Mass of link 1
m2 = 1;  % Mass of link 2
g = 9.81; % Gravity
r = 0.5; % Radius of the circular trajectory
F_d = [0; -5]; % Desired constant force in x and y

% Desired trajectory parameters
omega = 0.1; % Angular speed for circular trajectory
tspan = 0:0.01:20; % Time span

% Joint space variables
theta1 = zeros(length(tspan), 1);
theta2 = zeros(length(tspan), 1);

% Controller gains
Kp = diag([100, 100]);  % Position control gain
Kd = diag([20, 20]);    % Derivative gain
Kf = diag([10, 10]);    % Force control gain

% Simulation loop
for i = 1:length(tspan)
    t = tspan(i);
    
    % Desired position on circular trajectory
    x_d = r * cos(omega * t);
    y_d = r * sin(omega * t);
    x_dot_d = -r * omega * sin(omega * t);
    y_dot_d = r * omega * cos(omega * t);
    
    % Compute desired joint angles using inverse kinematics
    c2 = (x_d^2 + y_d^2 - l1^2 - l2^2) / (2 * l1 * l2);
    s2 = sqrt(1 - c2^2);
    theta2_d = atan2(s2, c2);
    
    k1 = l1 + l2 * cos(theta2_d);
    k2 = l2 * sin(theta2_d);
    theta1_d = atan2(y_d, x_d) - atan2(k2, k1);
    
    % Forward kinematics for current joint angles
    x = l1 * cos(theta1(i)) + l2 * cos(theta1(i) + theta2(i));
    y = l1 * cos(theta1(i)) + l2 * cos(theta1(i) + theta2(i));
    
    % Error in position
    e_p = [x_d - x; y_d - y];
    
    % Jacobian of the manipulator
    J = [-l1*sin(theta1(i)) - l2*sin(theta1(i) + theta2(i)), -l2*sin(theta1(i) + theta2(i));
          l1*cos(theta1(i)) + l2*cos(theta1(i) + theta2(i)),  l2*cos(theta1(i) + theta2(i))];
    
    % Joint velocities
    theta_dot = J' * [x_dot_d; y_dot_d];
    
    % Control law: Hybrid force/position
    tau = Kp * e_p + Kd * theta_dot + J' * (F_d - Kf * (J * theta_dot));
    
    % Update the joint angles using numerical integration (Euler method)
    theta1(i+1) = theta1(i) + tau(1) * 0.01;
    theta2(i+1) = theta2(i) + tau(2) * 0.01;
end

% Plotting the results
figure;
plot(tspan, theta1(1:end-1), 'r', 'DisplayName', 'Theta1');
hold on;
plot(tspan, theta2(1:end-1), 'b', 'DisplayName', 'Theta2');
xlabel('Time (s)');
ylabel('Joint Angles (rad)');
title('Joint Angles vs. Time');
legend show;
grid on;
