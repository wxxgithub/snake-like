function worm_motion()
%��;˵�����߳��˶�һ��
%����˵����
% 
global nodes;
global xy_nodes;
global coordinate_nodes;
global xy_nodes_world;
       for i = 2 : nodes
            xy_nodes(1:2,i-1) = [xy_nodes(1:2,i)];                  %��ͷ���(nodesΪͷ���)�⣬������һ�δ�������ֵ(nodes->nodes-1...2->1)
            coordinate_nodes(1:3,i-1) = coordinate_nodes(1:3,i);    %��ͷ����⣬�������Ӧ����ϵԭ�㼰�Ƕ�ֵ
       end
       xy_nodes(1:2,nodes) = [xy_nodes(1,nodes)+pi/4;sin(xy_nodes(1,nodes)+pi/4)];
       coordinate_nodes(1:3,nodes) = coordinate_nodes(1:3,nodes);
       for i = 1 : nodes                                                      %ת��Ϊ������ϵ��
            xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))] ...
                               * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
       end
end