clc;clear;close;
table = readtable('/tmp/test_RRT_Qlimits.csv');
test_size = 1000;

t_free = table.eps(1:test_size/2);
q_free = table.t(1:test_size/2);
t_limited = table.eps(test_size/2+1:test_size);
q_limited = table.t(test_size/2+1:test_size);

free_avg = mean(q_free)
limited_avg = mean(q_limited)