function snake_01()
%仿真实验0.1版
%%
%清除
clear;
clc;
%%
%初始参数设置
bodys = 0 : 0.1 : 1;                    %身体长度0.1
nodes = size(bodys,2);                  %节点个数
xy_nodes = zeros(2,nodes);              %节点[x;y] 
coordinate_nodes = zeros(3,nodes);      %各个节点次坐标系原点[x0;y0;头结点坐标系与主坐标系的偏转角度theta]

global nodes;
global nodes_trajectory;
nodes_trajectory = [];                  %轨迹坐标寄存器

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
%初始化位置，可自行设置
%snake_init(头结点(坐标系原点)坐标-x,头结点(坐标系原点)坐标-y,头结点坐标系与主坐标系的偏转角度theta,nodes)
function [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_init(x,y,theta)
    global nodes;
    %初始化头部坐标系
    xy_nodes(1:2,1) = [x;y];                                                   %随机生成一个头结点
    coordinate_nodes(1:3,1) = [xy_nodes(1:2,1);theta];                         %初始化一个坐标系（坐标系原点+偏转角度）%头结点坐标系与主坐标系的偏转角度，前行方向
    coordinate_nodes(1:3,1:nodes) = repmat(coordinate_nodes(1:3,1),1,nodes);                                                     
    for i = 1 : nodes                                                          %nodes=1为头结点的坐标,图中显示为-最左-结点
        xy_nodes(1:2,i) = [pi/4*(i-1);sin(pi/4*(i-1))];                        %转换为主坐标系中
        xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))]...
                                 * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
    end
    %plot(xy_nodes_world(1,:),xy_nodes_world(2,:),'-k.');%输出结果检查--初始化构造身体模型
    %axis([-40 40 -40 40]);
    %hold on
end

%%
%保持前行运动
function [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_forward(xy_nodes,coordinate_nodes)
       global nodes;
       global nodes_trajectory;
       
       for i = 1 : nodes-1
            xy_nodes(1:2,nodes+1-i) = [xy_nodes(1:2,nodes-i)];                  %除头结点外，其余结点一次传递坐标值
            coordinate_nodes(1:3,nodes+1-i) = coordinate_nodes(1:3,nodes-i);    %除头结点外，其余结点对应坐标系原点及角度值
       end
       %！！！在原次坐标系中，第一，区分各个坐标系中的头尾结点及其变化，第二，相位之间的变化。
       xy_nodes(1:2,1) = [xy_nodes(1,1)-pi/4;sin(xy_nodes(1,1)-pi/4)];
       coordinate_nodes(1:3,1) = coordinate_nodes(1:3,1);
       for i = 1 : nodes                                                      %转换为主坐标系中
            xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))] ...
                               * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
       end
       %调试用...
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
%运动转向
%Ctheta-顺时针偏转角度
function [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_turn(xy_nodes,coordinate_nodes,xy_nodes_world,Ctheta)
        global nodes;
    
        for i = 1 : nodes-1
            xy_nodes(1:2,nodes+1-i) = [xy_nodes(1:2,nodes-i)];                  %除头结点外，其余结点一次传递坐标值
            coordinate_nodes(1:3,nodes+1-i) = coordinate_nodes(1:3,nodes-i);    %除头结点外，其余结点对应坐标系原点及角度值
        end
        %！！！在原次坐标系中，第一，区分各个坐标系中的头尾结点及其变化，第二，相位之间的变化。
        xy_nodes(1:2,1) = [0;0];
        coordinate_nodes(1:3,1) = [xy_nodes_world(1:2,1);coordinate_nodes(3,i)-Ctheta];            %%坐标系的角度是顺时针
        %plot(xy_nodes(1,:),xy_nodes(2,:),'-k.');%调试用
        %axis([-10 40 -10 40]);
        for i = 1 : nodes                                                      %转换为主坐标系中
            xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))] ...
                               * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
        end
        %调试用...
        %plot(xy_nodes(1,:),xy_nodes(2,:),'-r.');
        %hold on
        %plot(xy_nodes_world(1,:),xy_nodes_world(2,:),'-k.');%实际中不能显示这条命令，头结点会停留一下
end
    
%%
%移动轨迹显示
function display(xy_nodes_world)
    global nodes_trajectory;
    global nodes;    
      %调试用...
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


