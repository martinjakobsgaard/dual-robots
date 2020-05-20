clc;clear;close;
table = readtable('/home/martin/Desktop/test_RRT_Qlimits.csv');
test_size = 1000;

t_free = table.eps(1:test_size/2);
q_free = table.t(1:test_size/2);
t_limited = table.eps(test_size/2+1:test_size);
q_limited = table.t(test_size/2+1:test_size);

hold on
edges1 = linspace(0, 1500, 50);
edges2 = linspace(0, 1500, 50);

figure(1);                 

subplot(121), histogram(q_free, 'BinEdges',edges1);
grid on;
%xlim([0, 7000]);
%ylim([0, 20]);
ylabel('Frequency', 'FontSize', 14);
xlabel('Time [ms]', 'FontSize', 14);
title('Histogram of no limits', 'FontSize', 14);

subplot(122), histogram(q_limited, 'BinEdges',edges2);
grid on;
%xlim([0, 500]);
%ylim([0, 20]);
ylabel('Frequency', 'FontSize', 14);
xlabel('Time [ms]', 'FontSize', 14);
title('Histogram of limits', 'FontSize', 14);

hold off

free_avg = mean(q_free)
limited_avg = mean(q_limited)