function snake_01()
%仿真实验0.2版
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

global obstacle_num;
obstacle_num = 5;                       %随机生成障碍物数目

global nodes;
global nodes_trajectory;
nodes_trajectory = [];                  %轨迹坐标寄存器

global obstacle_xy;
obstacle_xy = zeros(2,obstacle_num);
obstacle_xy = [30 -20 -5 -25 25;-20 -20 -5 25 0;5 5 5 5 5];%测试

global detect_t;
detect_t = 5;

[xy_nodes,coordinate_nodes,xy_nodes_world] = snake_init(rand(1)*25,rand(1)*25,pi/4);

for count = 1:60
    [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_forward(xy_nodes,coordinate_nodes);
    display(xy_nodes_world);
    %调试
    %if mod(count,30) == 0 
    %    [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_turn(xy_nodes,coordinate_nodes,xy_nodes_world,pi/4);
    %end
    [xy_nodes,coordinate_nodes,xy_nodes_world] = decision(xy_nodes,coordinate_nodes,xy_nodes_world);
    
    [xy_nodes,coordinate_nodes,xy_nodes_world] = fun(xy_nodes,coordinate_nodes,xy_nodes_world);
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
end

%%
%保持前行运动
function [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_forward(xy_nodes,coordinate_nodes)
       global nodes;
       global nodes_trajectory;
       global detect_t;
       
       for i = 1 : nodes-1
            xy_nodes(1:2,nodes+1-i) = [xy_nodes(1:2,nodes-i)];                  %除头结点外，其余结点一次传递坐标值
            coordinate_nodes(1:3,nodes+1-i) = coordinate_nodes(1:3,nodes-i);    %除头结点外，其余结点对应坐标系原点及角度值
       end
       xy_nodes(1:2,1) = [xy_nodes(1,1)-pi/4;sin(xy_nodes(1,1)-pi/4)];
       coordinate_nodes(1:3,1) = coordinate_nodes(1:3,1);
       for i = 1 : nodes                                                      %转换为主坐标系中
            xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))] ...
                               * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
       end
       detect_t = detect_t - 1;
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
        coordinate_nodes(1:3,1) = [xy_nodes_world(1:2,1);coordinate_nodes(3,1)-Ctheta];            %%坐标系的角度是顺时针
        for i = 1 : nodes                                                      %转换为主坐标系中
            xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))] ...
                               * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
        end
end
    
%%
%移动轨迹显示
function display(xy_nodes_world,obstacle_inf)
    global nodes_trajectory;
    global nodes;  
    global obstacle_xy;
      nodes_trajectory = [nodes_trajectory xy_nodes_world(:,nodes)];
      plot(xy_nodes_world(1,:),xy_nodes_world(2,:),'-r.',nodes_trajectory(1,:),nodes_trajectory(2,:),'-k.');
      
      obstacle(obstacle_xy(3,1),obstacle_xy(1,1),obstacle_xy(2,1),300,-pi/6);
      obstacle(obstacle_xy(3,2),obstacle_xy(1,2),obstacle_xy(2,2),300,-pi/4);
      obstacle(obstacle_xy(3,3),obstacle_xy(1,3),obstacle_xy(2,3),300,-pi/6);
      obstacle(obstacle_xy(3,4),obstacle_xy(1,4),obstacle_xy(2,4),300,-pi/6);
      obstacle(obstacle_xy(3,5),obstacle_xy(1,5),obstacle_xy(2,5),300,-pi/6);
      hold off;
      
      axis([-40 40 -40 40]);
      axis square;
      pause(0.05);
end

%%
%随机生成一中心店作为障碍物中心点坐标，有三个形状可供选择
function obstacle(radius,x0,y0,shape,rotate)
    theta=linspace(0,2*pi,shape+1)+rotate;
    hold on;
    line(radius*cos(theta)+x0,radius*sin(theta)+y0);
end

%%
%计算与各个障碍物的距离
function neardist = calculte(xy_nodes_world)
    global obstacle_num;
    global obstacle_xy;
        %obstacle_xy = rand(3,5)*25;
        obstacle = [xy_nodes_world(1:2,1) obstacle_xy(1:2,:)]';
        D = pdist(obstacle,'euclidean');
        D_obstacle = D(1,1:obstacle_num);
        [neardist(1,1),neardist(1,2)] = min(D_obstacle);                                %返回最小距离
end

%%
%最近距离与转角的关系--关键
function [xy_nodes,coordinate_nodes,xy_nodes_world] = decision(xy_nodes,coordinate_nodes,xy_nodes_world)
    global obstacle_xy;
    global detect_t;

    neardist = calculte(xy_nodes_world);
        if neardist(1,1) < obstacle_xy(3,neardist(1,2)) + obstacle_xy(3,neardist(1,2))
            angle = pi/4;
            else if neardist(1,1) < obstacle_xy(3,neardist(1,2)) + obstacle_xy(3,neardist(1,2))*2
                angle = pi/8;
                else if neardist(1,1) < obstacle_xy(3,neardist(1,2)) + obstacle_xy(3,neardist(1,2))*3
                    angle = pi/10; 
                end
            end
        end
        
    if neardist(1,1) <  (obstacle_xy(3,neardist(1,2)) + 5) && detect_t <= 0%obstacle_xy(r,neardist(1,2)+1)
       [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_turn(xy_nodes,coordinate_nodes,xy_nodes_world,angle);
       detect_t = 5;
    end
end


%%测试玩
%碰到边界后转大角度
function [xy_nodes,coordinate_nodes,xy_nodes_world] = fun(xy_nodes,coordinate_nodes,xy_nodes_world)
    if abs(xy_nodes_world(1,1)) > 40 || abs(xy_nodes_world(2,1)) > 40
        [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_turn(xy_nodes,coordinate_nodes,xy_nodes_world,-5*pi/6);
    end
end


