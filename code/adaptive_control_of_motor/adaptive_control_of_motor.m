clear; clc;

m = 2; % mass of the motor
m_hat(1) = 0; % initial estimation of the mass

% set of parameters
gamma = 0.5; 
lambda_1 = 10; 
lambda_2 = 25; 
lambda = 6; 
% set of parameters

r(1) = 0; % initial position of the motor

r = @(t) sin(4*t); % commanded position

dx(1) = 0; % initial velocity
dx_m(1) = 0; % initial 
x(1) = 0.5; 
x_m(1) = 0.5; 

x_m = @(t) (3681*exp(-5*t))/3362 - (25*cos(4*t + atan(9/40)))/41 + (405*t*exp(-5*t))/82;
dx_m = @(t) (100*sin(4*t + atan(9/40)))/41 - (900*exp(-5*t))/1681 - (2025*t*exp(-5*t))/82;

%ddx(1) = 0; % initial acceleration
ddx_m(1) = lambda_2*r(1) - lambda_1*dx_m(1) - lambda_2*x_m(1); % initial acceleration

dt = 0.01; % time step 

for t = 1:200
    % if t > 1
    %     dx(t) = (x(t) - x(t - 1))/dt;
    %     dx_m(t) = (x_m(t) - x_m(t - 1))/dt; 
    % end

    time = (t)*dt; 



    x_til(t) = x(t) - x_m(t); % tracking error
    dx_til(t) = dx(t) - dx_m(t); % rate of change of tracking error

    s(t) = dx_til(t) + lambda*x_til(t);

    % if t > 1
    %     ddx(t) = (dx(t) - dx(t - 1))/dt;
    %     ddx_m(t) = (dx_m(t) - dx_m(t - 1))/dt; 
    % end

    if t > 1
        ddx_m(t) = lambda_2*r(time) - lambda_1*dx_m(time) - lambda_2*x_m(time);
    end


    v(t) = ddx_m(t) - 2*lambda*dx_til(t) - (lambda^2)*x_til(t); 

    dm_hat(t) = -gamma*v(t)*s(t);

    m_hat(t + 1) = m_hat(t) + dm_hat(t) * dt;

    u(t) = m_hat(t)*(ddx_m(t) - 2*lambda*dx_til(t) - (lambda^2)*x_til(t));

    ddx(t) = u(t)/m;

    dx(t + 1) = dx(t) + ddx(t) * dt; 

    x(t + 1) = x(t) + dx(t + 1)*dt; 





end


    