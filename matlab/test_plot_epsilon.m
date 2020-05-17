clc;clear;close;
table = readtable('/tmp/test_RRT_epsilon.csv');
hold on
plot(table.eps,table.t,'LineWidth',2)
xlabel('Epsilon')
ylabel('Time (ms)')
%xlim([0.05 0.55])
%ylim([0 800])
hold off
