clc;clear;close;
rrt_nodes = readtable('/tmp/RRT_tree_nodes.csv');
rrt_path = readtable('/tmp/RRT_tree_path.csv');
rrt_qnodes = readtable('/tmp/RRT_tree_qnodes.csv');
rrt_qpath = readtable('/tmp/RRT_tree_qpath.csv');
opti_path = readtable('/tmp/RRT_tree_optimized_path.csv');

%hold on
%plot3(rrt_nodes.x,rrt_nodes.y,rrt_nodes.z,'o')
%plot3(rrt_path.x,rrt_path.y,rrt_path.z,'LineWidth',2)
%plot3(opti_path.x,opti_path.y,opti_path.z,'LineWidth',2)

%xlabel('Distance [m]') 
%ylabel('Distance [m]')
%zlabel('Distance [m]')
%title('3D Plot of samples and path used for path planning')

%hold off

points = [rrt_path.x, rrt_path.y, rrt_path.z];
points_opti = [opti_path.x, opti_path.y, opti_path.z];

dist = 0;
dist_opti = 0;

d = size(rrt_path.x,1);
d_opti = size(opti_path.x,1);

for i = 1:d-1
    v    = points(i, :) - points(i+1, :);
    dist_two = sqrt(sum(v .^ 2));
    dist = dist + dist_two;
end

for i = 1:d_opti-1
    v_opti    = points_opti(i, :) - points_opti(i+1, :);
    dist_two_opti = sqrt(sum(v_opti .^ 2));
    dist_opti = dist_opti + dist_two_opti;
end



