function worm_motion()
%用途说明：线虫运动一步
%参数说明：
% 
global nodes;
global xy_nodes;
global coordinate_nodes;
global xy_nodes_world;
       for i = 2 : nodes
            xy_nodes(1:2,i-1) = [xy_nodes(1:2,i)];                  %除头结点(nodes为头结点)外，其余结点一次传递坐标值(nodes->nodes-1...2->1)
            coordinate_nodes(1:3,i-1) = coordinate_nodes(1:3,i);    %除头结点外，其余结点对应坐标系原点及角度值
       end
       xy_nodes(1:2,nodes) = [xy_nodes(1,nodes)+pi/4;sin(xy_nodes(1,nodes)+pi/4)];
       coordinate_nodes(1:3,nodes) = coordinate_nodes(1:3,nodes);
       for i = 1 : nodes                                                      %转换为主坐标系中
            xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))] ...
                               * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
       end
end