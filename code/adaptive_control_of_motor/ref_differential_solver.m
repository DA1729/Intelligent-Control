syms x_m(t)
lambda_1 = 10; 
lambda_2 = 25; 

Dx_m = diff(x_m);

ode = diff(x_m, t, 2) + lambda_1*diff(x_m, t, 1) + lambda_2*x_m == lambda_2*sin(4*t);
cond1 = x_m(0) == 0.5; 
cond2 = Dx_m(0) == 0;

cond = [cond1 cond2];
x_m_Sol(t) = dsolve(ode, cond);
x_m_Sol = simplify(x_m_Sol)