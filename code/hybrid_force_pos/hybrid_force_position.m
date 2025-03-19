clear, clc

m = 1;
n = 2; 

alpha = 0.8;
beta = 0.32; 
neta = 0.4;
a1 = 1;  % link length 1
a2 = 0.8;  % link length 2
g = 9.8;  % acceleration due to gravity 

e1 = g/a1;
m2 = beta/(a2^2);  % mass 1
m1 = (alpha/(a1^2))-m2;  % mass 2

q_1(1) = -70;  % intial joint angle 1
q_2(1) = 80;  % initial joint angle 2
dq_1(1) = 0;  % initial joint velocity 1
dq_2 = 0;  % intitial joint velocity 2
ddq_1(1) = 0;  % initial joint 1 acceleration 

lambda = 20; 
Kv = 100;
Kf = 20;
F = 10; 
G = 10; 
kappa = 1; 
Kz = 1; 
Z_B = 4;

q1_d = @(t) -90 + 52.5*(1-cos(1.26*t)*m);

dq1_d = @(t) 52.5*1.26*sin(1.26*t);

ddq1_d = @(t) 52.5*1.26*1.26*cos(1.26*t);

lambda_d = @(t) 10*n*t;

radius = 1.9178; 
L = [1;0];  % extended jacobian 

J = [0 -2*a1*a2*sind(q_2(1))];

dt = 0.001;

for i = 1:2500
    t = dt*i; 
    e = q1_d(t) - q1(i);
    de = dq1_d(t) - dq_1(i);

    r = de + lambda*e;

end
