clear;
clc;
%%
%初始参数设置
bodys = 0 : 0.1 : 1;                    %身体长度0.1
nodes = size(bodys,2);                  %节点个数
xy_nodes = zeros(2,nodes);              %节点[x;y] 
coordinate_nodes = zeros(3,nodes);      %各个节点次坐标系原点[x0;y0;头结点坐标系与主坐标系的偏转角度theta]

head_position = zeros(2,1);             %每次偏转前的头结点坐标寄存
beta = 0;

%%
%初始化头部坐标系
xy_nodes(1:2,1) = rand(2,1)*25;                                            %随机生成一个头结点
coordinate_nodes(1:3,1) = [xy_nodes(1:2,1);rand(1)*pi];                    %初始化一个坐标系（坐标系原点+偏转角度）%头结点坐标系与主坐标系的偏转角度，前行方向
coordinate_nodes(1:3,:) = repmat(coordinate_nodes(1:3,1),1,nodes);                                                     
for i = 1 : nodes                                                          %此处为头结点按波形走过的位置，nodes=11为头结点的坐标
    xy_nodes(1:2,i) = [pi/4*(i-1);sin(pi/4*(i-1))];
end
%plot(xy_nodes(1,:),xy_nodes(2,:),'-*');%观察输出效果，中间过程
%hold on
for i = 1 : nodes                                                          %转换为主坐标系中
    xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))]...
                       * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
end
plot(xy_nodes_world(1,:),xy_nodes_world(2,:),'-k.');%输出结果检查--初始化构造身体模型
axis([-20 40 -20 40]);
%hold on

%%
%无偏转情况下前行 
%for count = 1:20
%    for i = 1 : nodes-1
%        xy_nodes(1:3,nodes+1-i) = [xy_nodes(1:2,nodes-i);coordinate_nodes(3,i)];
%    end
%        %在原次坐标系中，第一，区分各个坐标系中的头尾结点及其变化，第二，相位之间的变化。
%        xy_nodes(1:3,1) = [xy_nodes(1,1)-pi/4;sin(xy_nodes(1,1)-pi/4);coordinate_nodes(3,1)];    
%    for i = 1 : nodes                                                      %转换为主坐标系中
%        xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))]...
%                       * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
%    end
%       %调试用...
%       %plot(xy_nodes(1,:),xy_nodes(2,:),'-r.');
%       %hold on
%       plot(xy_nodes_world(1,:),xy_nodes_world(2,:),'-k.');
%      axis([-10 40 -10 40]);
%      pause(0.1);
%end

%%
%有偏转情况下前行,偏转角度固定
%for count = 1:50
%    for i = 1 : nodes-1
%       xy_nodes(1:2,nodes+1-i) = [xy_nodes(1:2,nodes-i)];                  %除头结点外，其余结点一次传递坐标值
%       coordinate_nodes(1:3,nodes+1-i) = coordinate_nodes(1:3,nodes-i);    %除头结点外，其余结点对应坐标系原点及角度值
%    end
%       %！！！在原次坐标系中，第一，区分各个坐标系中的头尾结点及其变化，第二，相位之间的变化。
%       xy_nodes(1:2,1) = [xy_nodes(1,1)-pi/4;sin(xy_nodes(1,1)-pi/4)];
%       coordinate_nodes(1:3,1) = coordinate_nodes(1:3,1);
%    for i = 1 : nodes                                                      %转换为主坐标系中
%       xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))]...
%                       * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
%    end
%       %调试用...
%       %plot(xy_nodes(1,:),xy_nodes(2,:),'-r.');
%       %hold on
%       plot(xy_nodes_world(1,:),xy_nodes_world(2,:),'-k.');
%       axis([-10 40 -10 40]);
%       pause(0.1);
%end

%%
%
for count = 1:5
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
       plot(xy_nodes_world(1,:),xy_nodes_world(2,:),'-k.');
       axis([-10 40 -10 40]);
       pause(0.1);
end
for count = 1:1
    for i = 1 : nodes-1
       xy_nodes(1:2,nodes+1-i) = [xy_nodes(1:2,nodes-i)];                  %除头结点外，其余结点一次传递坐标值
       coordinate_nodes(1:3,nodes+1-i) = coordinate_nodes(1:3,nodes-i);    %除头结点外，其余结点对应坐标系原点及角度值
    end
       %！！！在原次坐标系中，第一，区分各个坐标系中的头尾结点及其变化，第二，相位之间的变化。
       xy_nodes(1:2,1) = [0;0];
       coordinate_nodes(1:3,1) = [xy_nodes_world(1:2,1);coordinate_nodes(3,i)-pi/4];            %%坐标系的角度是逆时针
       %plot(xy_nodes(1,:),xy_nodes(2,:),'-k.');%调试用
       axis([-10 40 -10 40]);
    for i = 1 : nodes                                                      %转换为主坐标系中
       xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))] ...
                               * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
    end
       %调试用...
       %plot(xy_nodes(1,:),xy_nodes(2,:),'-r.');
       %hold on
       %plot(xy_nodes_world(1,:),xy_nodes_world(2,:),'-k.');%实际中不能显示这条命令，头结点会停留一下
       axis([-10 40 -10 40]);
       pause(0.1);
end
for count = 1:20
    %break;%调试用
    for i = 1 : nodes-1
       xy_nodes(1:2,nodes+1-i) = [xy_nodes(1:2,nodes-i)];                  %除头结点外，其余结点依次传递坐标值
       coordinate_nodes(1:3,nodes+1-i) = coordinate_nodes(1:3,nodes-i);    %除头结点外，其余结点对应坐标系原点及角度值
    end
       %！！！在原次坐标系中，第一，区分各个坐标系中的头尾结点及其变化，第二，相位之间的变化。
       xy_nodes(1:2,1) = [xy_nodes(1,1)-pi/4;sin(xy_nodes(1,1)-pi/4+pi/2)];%%问题出现在这里
       coordinate_nodes(1:3,1) = coordinate_nodes(1:3,1);
    for i = 1 : nodes                                                      %转换为主坐标系中
       xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))] ...
                               * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
    end
       %调试用...
       %plot(xy_nodes(1,:),xy_nodes(2,:),'-r.');
       hold on
       plot(xy_nodes_world(1,:),xy_nodes_world(2,:),'-k.');
       axis([-10 40 -10 40]);
       pause(0.1);
end