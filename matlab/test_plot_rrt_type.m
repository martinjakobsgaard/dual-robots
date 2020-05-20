clc;clear;close;
table = readtable('/home/martin/Desktop/test_type_rrt_connect.csv');
test_size = 1000;

rrt_eps = table.eps(1:test_size/2);
rrt_t = table.t(1:test_size/2);
rrt_connect_eps = table.eps(test_size/2+1:test_size);
rrt_connect_t = table.t(test_size/2+1:test_size);

hold on
edges1 = linspace(0, 30000, 50);
edges2 = linspace(0, 1000, 50);

figure(1);                 

subplot(121), histogram(rrt_t, 'BinEdges',edges1);
grid on;
%xlim([0, 7000]);
%ylim([0, 20]);
ylabel('Frequency', 'FontSize', 14);
xlabel('Time [ms]', 'FontSize', 14);
title('Histogram of RRT', 'FontSize', 14);

subplot(122), histogram(rrt_connect_t, 'BinEdges',edges2);
grid on;
%xlim([0, 500]);
%ylim([0, 20]);
ylabel('Frequency', 'FontSize', 14);
xlabel('Time [ms]', 'FontSize', 14);
title('Histogram of RRT Connect', 'FontSize', 14);

hold off
