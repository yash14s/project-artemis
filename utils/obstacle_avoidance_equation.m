clc
syms d
%Define d_min,d_avoid,k1,k2,v_min,v_max
d_min = 0.02;
d_avoid = 0.6;
k1 = 0.33;
k2 = 0.95;
v_min = 0.3;
v_max = 0.5;
range = [d_min d_avoid];

%Define v
v = k1 *(d - (d_avoid + k2))^2;
%Bound v between v_min and v_max
v = piecewise(v < v_min, v_min, v > v_max, v_max, v);
figure
fplot(v,range)
hold on
title('Avoidance speed vs Distance to obstacle')
xlabel('Distance (m)') 
ylabel('Speed (m/s)')
fplot(v,range)



