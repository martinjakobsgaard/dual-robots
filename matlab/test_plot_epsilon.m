clc;clear;close;
table = readtable('/home/martin/Desktop/test_epsilon_rrt_connect.csv');

x = table.eps;
y = table.t;

xi = linspace(min(x), max(x), 100);
yi = interp1(x, y, xi, 'spline', 'extrap');

figure(1)
hold on
plot(xi, yi)
hold off
grid
xlabel('Epsilon') 
ylabel('Time [ms]')