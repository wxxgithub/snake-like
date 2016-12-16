function worm_turn(Ctheta)
%用途说明：线虫运动偏转（-50 - 50度）
%参数说明：
%         Ctheta（例如45-顺时针偏转角度--向右偏45度）
%         Ctheta取值范围：-50<=Ctheta<=50
global nodes;
global xy_nodes;
global coordinate_nodes;
global xy_nodes_world;
       theta = -2*pi*Ctheta/360;
       for i = 2 : nodes
            xy_nodes(1:2,i-1) = [xy_nodes(1:2,i)];                  %除头结点(nodes为头结点)外，其余结点一次传递坐标值(nodes->nodes-1...2->1)
            coordinate_nodes(1:3,i-1) = coordinate_nodes(1:3,i);    %除头结点外，其余结点对应坐标系原点及角度值
       end

        xy_nodes(1:2,nodes) = [pi/4;sin(pi/4)];
        coordinate_nodes(1:3,nodes) = [xy_nodes_world(1:2,nodes);mod(coordinate_nodes(3,1)+theta,2*pi)];            %%坐标系的角度是顺时针
        for i = 1 : nodes                                                      %转换为主坐标系中
            xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))] ...
                               * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
        end
end