function snake_01()
%����ʵ��0.1��
%%
%���
clear;
clc;
%%
%��ʼ��������
bodys = 0 : 0.1 : 1;                    %���峤��0.1
nodes = size(bodys,2);                  %�ڵ����
xy_nodes = zeros(2,nodes);              %�ڵ�[x;y] 
coordinate_nodes = zeros(3,nodes);      %�����ڵ������ϵԭ��[x0;y0;ͷ�������ϵ��������ϵ��ƫת�Ƕ�theta]

global nodes;
global nodes_trajectory;
nodes_trajectory = [];                  %�켣����Ĵ���

[xy_nodes,coordinate_nodes,xy_nodes_world] = snake_init(rand(1)*25,rand(1)*25,pi/4);
for count = 1:100
    [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_forward(xy_nodes,coordinate_nodes);
    display(xy_nodes_world);
    if mod(count,30) == 0 
        [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_turn(xy_nodes,coordinate_nodes,xy_nodes_world,pi/4);
    end
        %display(xy_nodes_world);
end

end
%%
%��ʼ��λ�ã�����������
%snake_init(ͷ���(����ϵԭ��)����-x,ͷ���(����ϵԭ��)����-y,ͷ�������ϵ��������ϵ��ƫת�Ƕ�theta,nodes)
function [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_init(x,y,theta)
    global nodes;
    %��ʼ��ͷ������ϵ
    xy_nodes(1:2,1) = [x;y];                                                   %�������һ��ͷ���
    coordinate_nodes(1:3,1) = [xy_nodes(1:2,1);theta];                         %��ʼ��һ������ϵ������ϵԭ��+ƫת�Ƕȣ�%ͷ�������ϵ��������ϵ��ƫת�Ƕȣ�ǰ�з���
    coordinate_nodes(1:3,1:nodes) = repmat(coordinate_nodes(1:3,1),1,nodes);                                                     
    for i = 1 : nodes                                                          %nodes=1Ϊͷ��������,ͼ����ʾΪ-����-���
        xy_nodes(1:2,i) = [pi/4*(i-1);sin(pi/4*(i-1))];                        %ת��Ϊ������ϵ��
        xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))]...
                                 * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
    end
    %plot(xy_nodes_world(1,:),xy_nodes_world(2,:),'-k.');%���������--��ʼ����������ģ��
    %axis([-40 40 -40 40]);
    %hold on
end

%%
%����ǰ���˶�
function [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_forward(xy_nodes,coordinate_nodes)
       global nodes;
       global nodes_trajectory;
       
       for i = 1 : nodes-1
            xy_nodes(1:2,nodes+1-i) = [xy_nodes(1:2,nodes-i)];                  %��ͷ����⣬������һ�δ�������ֵ
            coordinate_nodes(1:3,nodes+1-i) = coordinate_nodes(1:3,nodes-i);    %��ͷ����⣬�������Ӧ����ϵԭ�㼰�Ƕ�ֵ
       end
       %��������ԭ������ϵ�У���һ�����ָ�������ϵ�е�ͷβ��㼰��仯���ڶ�����λ֮��ı仯��
       xy_nodes(1:2,1) = [xy_nodes(1,1)-pi/4;sin(xy_nodes(1,1)-pi/4)];
       coordinate_nodes(1:3,1) = coordinate_nodes(1:3,1);
       for i = 1 : nodes                                                      %ת��Ϊ������ϵ��
            xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))] ...
                               * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
       end
       %������...
       %plot(xy_nodes(1,:),xy_nodes(2,:),'-r.');
       %hold on
            %nodes_trajectory = [nodes_trajectory xy_nodes_world];
            %plot(xy_nodes_world(1,:),xy_nodes_world(2,:),'-r.');
       %hold on
            %plot(nodes_trajectory(1,:),nodes_trajectory(2,:),'k.');
       %hold off
            %axis([-40 40 -40 40]);
            %axis square;
            %pause(0.1); 
end
%%
%�˶�ת��
%Ctheta-˳ʱ��ƫת�Ƕ�
function [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_turn(xy_nodes,coordinate_nodes,xy_nodes_world,Ctheta)
        global nodes;
    
        for i = 1 : nodes-1
            xy_nodes(1:2,nodes+1-i) = [xy_nodes(1:2,nodes-i)];                  %��ͷ����⣬������һ�δ�������ֵ
            coordinate_nodes(1:3,nodes+1-i) = coordinate_nodes(1:3,nodes-i);    %��ͷ����⣬�������Ӧ����ϵԭ�㼰�Ƕ�ֵ
        end
        %��������ԭ������ϵ�У���һ�����ָ�������ϵ�е�ͷβ��㼰��仯���ڶ�����λ֮��ı仯��
        xy_nodes(1:2,1) = [0;0];
        coordinate_nodes(1:3,1) = [xy_nodes_world(1:2,1);coordinate_nodes(3,i)-Ctheta];            %%����ϵ�ĽǶ���˳ʱ��
        %plot(xy_nodes(1,:),xy_nodes(2,:),'-k.');%������
        %axis([-10 40 -10 40]);
        for i = 1 : nodes                                                      %ת��Ϊ������ϵ��
            xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))] ...
                               * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
        end
        %������...
        %plot(xy_nodes(1,:),xy_nodes(2,:),'-r.');
        %hold on
        %plot(xy_nodes_world(1,:),xy_nodes_world(2,:),'-k.');%ʵ���в�����ʾ�������ͷ����ͣ��һ��
end
    
%%
%�ƶ��켣��ʾ
function display(xy_nodes_world)
    global nodes_trajectory;
    global nodes;    
      %������...
      %plot(xy_nodes(1,:),xy_nodes(2,:),'-r.');
      %hold on
      nodes_trajectory = [nodes_trajectory xy_nodes_world(:,nodes)];
      
      %plot(xy_nodes_world(1,:),xy_nodes_world(2,:),'-r.');
      %plot(nodes_trajectory(1,:),nodes_trajectory(2,:),'-k.');
      plot(xy_nodes_world(1,:),xy_nodes_world(2,:),'-r.',nodes_trajectory(1,:),nodes_trajectory(2,:),'-k.');
      
      axis([-40 40 -40 40]);
      axis square;
      pause(0.1);
end


