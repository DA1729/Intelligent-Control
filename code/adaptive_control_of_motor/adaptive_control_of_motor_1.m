clear; clc;

m = 2; % mass of the motor
m_hat(1) = 0; % initial estimation of the mass

% set of parameters
gamma = 0.5; 
lambda_1 = 10; 
lambda_2 = 25; 
lambda = 6; 
dt = 0.01; % time step
time_steps = 1000;

% Initial values
x(1) = 0.5;
dx(1) = 0; 
ddx(1) = 0; 

% Commanded position and velocity functions
r = @(t) sin(4 * t);
x_m = @(t) (3681 * exp(-5 * t)) / 3362 - (25 * cos(4 * t + atan(9 / 40))) / 41 + (405 * t * exp(-5 * t)) / 82;
dx_m = @(t) (100 * sin(4 * t + atan(9 / 40))) / 41 - (900 * exp(-5 * t)) / 1681 - (2025 * t * exp(-5 * t)) / 82;

% Adaptive control loop
for t = 1:time_steps
    time = (t - 1) * dt; % current time
    r_t = r(time);       % commanded position at time t
    x_m_t = x_m(time);   % desired position from function
    dx_m_t = dx_m(time); % desired velocity from function
    
    % Tracking error
    x_til(t) = x(t) - x_m_t;
    dx_til(t) = dx(t) - dx_m_t;
    s(t) = dx_til(t) + lambda * x_til(t);

    % Desired acceleration
    ddx_m = lambda_2 * r_t - lambda_1 * dx_m_t - lambda_2 * x_m_t;
    
    % Control law
    v = ddx_m - 2 * lambda * dx_til(t) - (lambda^2) * x_til(t);
    
    % Mass update law
    dm_hat(t) = -gamma * v * s(t);
    m_hat(t + 1) = m_hat(t) + dm_hat(t) * dt;
    
    % Control input and system dynamics
    u(t) = m_hat(t) * v;
    ddx(t) = u(t) / m;
    dx(t + 1) = dx(t) + ddx(t) * dt;
    x(t + 1) = x(t) + dx(t + 1) * dt;
end


time_values = 0:dt:(time_steps - 1) * dt;

% plots
figure;
subplot(2, 1, 1);
plot(time_values, x(1:end-1), 'b', 'DisplayName', 'Actual Position (x)');
hold on;
plot(time_values, arrayfun(x_m, time_values), '--r', 'DisplayName', 'Commanded Position (x_m)');
xlabel('Time (s)'); ylabel('Position');
title('Position Tracking');
legend;
grid on;

subplot(2, 1, 2);
plot(time_values, m_hat(1:end-1), 'b', 'DisplayName', 'Estimated Mass (m\_hat)');
hold on;
yline(m, 'r--', 'DisplayName', 'Actual Mass (m)');
xlabel('Time (s)'); ylabel('Mass Estimate');
title('Mass Estimation');
legend;
grid on;
