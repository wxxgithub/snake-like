%%
%��ʼ��������
bodys = 0 : 0.1 : 1;            %���峤��0.1
nodes = size(bodys,2);          %�ڵ����
xy_nodes = zeros(2,nodes);      %�ڵ�[x;y]����
angle_head = zeros(1,1);        %ͷ����Ӧƫת�Ƕ�
theta = pi/8;
%coordinate_nodes = zeros(ndoes,3);    %�ڵ�����ϵԭ�㼰�Ƕ�

%%
%��ʼ������snake--�����޸��˶����̲���
%��һ����������װ��ͨ�ú��������������
for i = 1 : 11
    xy_nodes(:,i) = [pi/4*(i-1);sin(pi/4*(i-1)+pi/2)];
end
plot(xy_nodes(1,:),xy_nodes(2,:),'-*');
axis([-10 40 -10 40]);
%hold on

%%
%��ƫת�����ǰ�� 
for count = 1:5
for i = 1 : nodes-1
    xy_nodes(:,i) = xy_nodes(:,i+1);
end
    xy_nodes(:,nodes) = [xy_nodes(1,nodes)+pi/4;sin(xy_nodes(1,nodes)+pi/4+pi/2)];  
    plot(xy_nodes(1,:),xy_nodes(2,:),'-k.');
    axis([-10 40 -10 40]);
    pause(0.5);
end

%%
%��ƫת�Ƕ���ǰ��
for count = 1:5
for i = 1 : nodes-1
    xy_nodes(:,i) = xy_nodes(:,i+1);
end
    %xy_nodes(:,nodes) = [xy_nodes(1,nodes)+pi/4;sin(xy_nodes(1,nodes)+pi/4+pi/2)]; 
    if count == 1
        xy_nodes(1,nodes) = (xy_nodes(1,nodes) + pi/4 - xy_nodes(1,nodes)) * cos(theta) - (y_tmp - sin(xy_nodes(1,nodes)+pi/4+pi/2)) * sin(theta) + xy_nodes(1,nodes);
        xy_nodes(2,nodes) = (y_tmp - sin(xy_nodes(1,nodes)+pi/4+pi/2)) * cos(theta) + (x_tmp - xy_nodes(1,nodes)) * sin(theta) + sin(xy_nodes(1,nodes)+pi/4+pi/2);
    end
    plot(xy_nodes(1,:),xy_nodes(2,:),'-k.');
    axis([-10 40 -10 40]);
    pause(0.5);
end
