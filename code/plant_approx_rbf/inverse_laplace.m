clear, clc
syms s; 

F = 133/((s^2 + 25*s)*(1 + s^2));

f = ilaplace(F);