clear, clc

% the chosen plant G(s) = 133/(s^2 + 25s)
y = @(t) 133/25 - (3325*cos(t))/626 - (133*sin(t))/626 - (133*exp(-25*t))/15650;

alpha = 1.6; 
eta = 0.8; 

w = rand(5, 1); % randomly initialized weights matrix

c = [-1 -10; -5 -0.5; 0 0; 0.5 5; 1 10]; % center matrix for all the rows
b = [1.5; 1.5; 1.5; 1.5; 1.5]; % the bandwidth matrix

u = @(t) sin(t);

dt = 0.001; % time step
time_sim = 0:dt:10; % simulation time from 0 to 2 seconds
y_real = zeros(size(time_sim));
y_model = zeros(size(time_sim));

for i = 1:length(time_sim)
    time = time_sim(i); 

    % input vector
    x = [u(time) y(time)];

    % the output of the hidden layer
    for j = 1:5
        norm_sq = norm(x - c(j, 1:2))^2;
        h(j) = exp(-(norm_sq)/(2*(b(j))^2)); 
    end

    % model output
    y_m = sum(w.*(h'));
    
    % store results for comparison
    y_real(i) = y(time);
    y_model(i) = y_m;

    % performance index
    E = 0.5*(y_real(i) - y_m)^2; 

    % update weights
    for j = 1:5
        del_w(j) = eta*(y_real(i) - y_m)*h(j);
        w(j) = w(j) + alpha*del_w(j);
    end
end

% Plotting results
figure;
plot(time_sim, y_real, 'r', 'LineWidth', 1.5); hold on;
plot(time_sim, y_model, 'b--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Output');
legend('y(t) - Actual System', 'y_m(t) - Model Output');
title('Comparison of Actual System Output and Model Output');
grid on;
