function worm_turn(Ctheta)
%��;˵�����߳��˶�ƫת��-50 - 50�ȣ�
%����˵����
%         Ctheta������45-˳ʱ��ƫת�Ƕ�--����ƫ45�ȣ�
%         Cthetaȡֵ��Χ��-50<=Ctheta<=50
global nodes;
global xy_nodes;
global coordinate_nodes;
global xy_nodes_world;
       theta = -2*pi*Ctheta/360;
       for i = 2 : nodes
            xy_nodes(1:2,i-1) = [xy_nodes(1:2,i)];                  %��ͷ���(nodesΪͷ���)�⣬������һ�δ�������ֵ(nodes->nodes-1...2->1)
            coordinate_nodes(1:3,i-1) = coordinate_nodes(1:3,i);    %��ͷ����⣬�������Ӧ����ϵԭ�㼰�Ƕ�ֵ
       end

        xy_nodes(1:2,nodes) = [pi/4;sin(pi/4)];
        coordinate_nodes(1:3,nodes) = [xy_nodes_world(1:2,nodes);mod(coordinate_nodes(3,1)+theta,2*pi)];            %%����ϵ�ĽǶ���˳ʱ��
        for i = 1 : nodes                                                      %ת��Ϊ������ϵ��
            xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))] ...
                               * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
        end
end