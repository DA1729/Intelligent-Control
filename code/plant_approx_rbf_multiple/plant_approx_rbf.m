clear, clc
% discrete plant
u = @(k) sin(k);
y = @(k) (u(k))^3 + (y(k-1))\(1 + (y(k-1))^2);

y(0) = 1; 