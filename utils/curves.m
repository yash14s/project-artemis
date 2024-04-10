syms x
v_min = 2;
k = 5;
y = k*(x-3)^2 + v_min;
range = [0.5,3];
fplot(y,range);