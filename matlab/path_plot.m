clc;clear;close;
rrt_nodes = readtable('/tmp/RRT_tree_nodes.csv');
rrt_path = readtable('/tmp/RRT_tree_path.csv');
rrt_qnodes = readtable('/tmp/RRT_tree_qnodes.csv');
rrt_qpath = readtable('/tmp/RRT_tree_qpath.csv');
opti_path = readtable('/tmp/RRT_tree_optimized_path.csv');

RRT_tree_optimized_path.csv


hold on
plot3(rrt_nodes.x,rrt_nodes.y,rrt_nodes.z,'o')
plot3(rrt_path.x,rrt_path.y,rrt_path.z)
plot3(opti_path.x,opti_path.y,opti_path.z)

xlabel('Distance [m]') 
ylabel('Distance [m]')
zlabel('Distance [m]')
titel('3D Plot of samples and path used for path planning')

hold off
