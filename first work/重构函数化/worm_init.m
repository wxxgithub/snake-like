function worm_init(x,y,theta)
%��;˵������ʼ���߳��������꣬����ϵƫתֵ
%����˵����
%worm_init(ͷ���(����ϵԭ��)����-x,ͷ���(����ϵԭ��)����-y,
%          ͷ�������ϵ��������ϵ��ƫת�Ƕ�theta,nodes)
%pi/4(˳ʱ��ƫת45��)��ԭ����ϵ����������ϵ����һ�� 
global nodes;
global xy_nodes;
global coordinate_nodes;
global xy_nodes_world; 
    theta = -2*pi*theta/360;
    xy_nodes(1:2,1) = [x;y];                                               %�������һ��ͷ���        
    coordinate_nodes(1:3,1) = [xy_nodes(1:2,1);theta];                     %��ʼ��һ������ϵ������ϵԭ��+ƫת�Ƕȣ�%ͷ�������ϵ��������ϵ��ƫת�Ƕȣ�ǰ�з���
    coordinate_nodes(1:3,1:nodes) = repmat(coordinate_nodes(1:3,1),1,nodes);                                                     
    for i = 1 : nodes             %nodes=1Ϊͷ��������,ͼ����ʾΪ-����-���
        xy_nodes(1:2,i) = [pi/4*(i-1);sin(pi/4*(i-1))];   
        xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))]...
                                 * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
    end
end