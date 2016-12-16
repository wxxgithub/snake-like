function snake_20()
%仿真实验0.2版
%结合仿真出的距离传感器，做相应的控制算法if-else仿真
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
obstacle_xy = [-20 20 0 -20 20;20 20 0 -20 -20;5 5 5 5 5];%测试

global detect_t;
detect_t = 5;

[xy_nodes,coordinate_nodes,xy_nodes_world] = snake_init(25,0,0);
     %figure ;
for count = 1:350
    [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_forward(xy_nodes,coordinate_nodes);
    
    %disp('count = ');disp(count);
    disp(sprintf('\ncount = %d',count));
    display(xy_nodes,coordinate_nodes,xy_nodes_world);
    %调试
    %if mod(count,30) == 0 
    %    [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_turn(xy_nodes,coordinate_nodes,xy_nodes_world,pi/4);
    %end
    [xy_nodes,coordinate_nodes,xy_nodes_world] = decision(xy_nodes,coordinate_nodes,xy_nodes_world);
    
    %[xy_nodes,coordinate_nodes,xy_nodes_world]=fun(xy_nodes,coordinate_nodes,xy_nodes_world);%测试边界玩的
    %%参数查看

    %disp('xy_nodes_world = ');disp(xy_nodes_world);
    %disp('coordinate_nodes = ');disp(coordinate_nodes);
    %real_distance = detect_distance(xy_nodes,coordinate_nodes,xy_nodes_world);
    %disp('real_distance = 文字可否');disp(real_distance(1,:,:));
end

end

%%
%初始化位置，可自行设置
%snake_init(头结点(坐标系原点)坐标-x,头结点(坐标系原点)坐标-y,头结点坐标系与主坐标系的偏转角度theta,nodes)
%pi/4(逆时针偏转45度)，原坐标系与世界坐标系方向一致
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
            xy_nodes(1:2,nodes+1-i) = [xy_nodes(1:2,nodes-i)];                  %除头结点外，其余结点一次传递坐标值(...、2->3、1->2)
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
%Ctheta（例如pi/4）-逆时针偏转角度
%偏转角度取值范围：0<=Ctheta<=90
function [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_turn(xy_nodes,coordinate_nodes,xy_nodes_world,Ctheta)
        global nodes;
    
        for i = 1 : nodes-1
            xy_nodes(1:2,nodes+1-i) = [xy_nodes(1:2,nodes-i)];                  %除头结点外，其余结点一次传递坐标值
            coordinate_nodes(1:3,nodes+1-i) = coordinate_nodes(1:3,nodes-i);    %除头结点外，其余结点对应坐标系原点及角度值
        end
        %！！！在原次坐标系中，第一，区分各个坐标系中的头尾结点及其变化，第二，相位之间的变化。
        xy_nodes(1:2,1) = [0;0];
        coordinate_nodes(1:3,1) = [xy_nodes_world(1:2,1);mod(coordinate_nodes(3,1)+Ctheta,2*pi)];            %%坐标系的角度是顺时针
        for i = 1 : nodes                                                      %转换为主坐标系中
            xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))] ...
                               * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
        end
end
    
%%
%移动轨迹显示
function display(xy_nodes,coordinate_nodes,xy_nodes_world)
    global nodes_trajectory;
    global nodes;  
    global obstacle_xy;
    axisnum = 40;
      nodes_trajectory = [nodes_trajectory xy_nodes_world(:,nodes)];

      %plot(xy_nodes_world(1,:),xy_nodes_world(2,:),'-b.',nodes_trajectory(1,:),nodes_trajectory(2,:),'-k');
      plot(xy_nodes_world(1,:),xy_nodes_world(2,:),'-b.');
      
      obstacle(obstacle_xy(3,1),obstacle_xy(1,1),obstacle_xy(2,1),300,-pi/6);
      obstacle(obstacle_xy(3,2),obstacle_xy(1,2),obstacle_xy(2,2),300,-pi/4);
      obstacle(obstacle_xy(3,3),obstacle_xy(1,3),obstacle_xy(2,3),300,-pi/6);
      obstacle(obstacle_xy(3,4),obstacle_xy(1,4),obstacle_xy(2,4),300,-pi/6);
      obstacle(obstacle_xy(3,5),obstacle_xy(1,5),obstacle_xy(2,5),300,-pi/6);
      
            %x=-20:1:20;
            %plot(x,tan(coordinate_nodes(3,1))*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1),'g-.',...
            %x,tan(coordinate_nodes(3,1)-pi/2)*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1),'r-.');
      %end
      %plot(x,xy_nodes_world(2,1),'r-.');
      %plot(x,(x-xy_nodes_world(1,1))+xy_nodes_world(2,1),'r-.');
      
      dis_test(xy_nodes,coordinate_nodes,xy_nodes_world);
      
      hold off;
      
      axis([-axisnum axisnum -axisnum axisnum]);
      axis square;
      pause(0.1);
end

%%
%随机生成一中心店作为障碍物中心点坐标，有三个形状可供选择
function obstacle(radius,x0,y0,shape,rotate)
    theta=linspace(0,2*pi,shape+1)+rotate;
    hold on;
    line(radius*cos(theta)+x0,radius*sin(theta)+y0);
    hold off;
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
function [xy_nodes,coordinate_nodes,xy_nodes_world] = decision0(xy_nodes,coordinate_nodes,xy_nodes_world)
%最初的决策试验
    global obstacle_xy;
    global detect_t;

    neardist = calculte(xy_nodes_world);
        if neardist(1,1) < obstacle_xy(3,neardist(1,2)) + obstacle_xy(3,neardist(1,2))
            angle = pi/3;
            else if neardist(1,1) < obstacle_xy(3,neardist(1,2)) + obstacle_xy(3,neardist(1,2))*2
                angle = pi/8;
                else if neardist(1,1) < obstacle_xy(3,neardist(1,2)) + obstacle_xy(3,neardist(1,2))*3
                    angle = pi/10; 
                end
            end
        end
        
    if neardist(1,1) <  (obstacle_xy(3,neardist(1,2)) + 5) && detect_t <= 0%obstacle_xy(r,neardist(1,2)+1)
       [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_turn(xy_nodes,coordinate_nodes,xy_nodes_world,angle);
       detect_t = 3;
    end
end

function [xy_nodes,coordinate_nodes,xy_nodes_world] = decision1(xy_nodes,coordinate_nodes,xy_nodes_world)
%根据是三个方向上的探测的距离信息
    global detect_t;
    [real_distance,min_distance]= distance_123(xy_nodes,coordinate_nodes,xy_nodes_world);
    forward_dis = min_distance(1,1);
    left_dis = min_distance(1,2);
    right_dis = min_distance(1,3);
    if forward_dis <= 5
        if left_dis <= 5
            if right_dis <= 5
                angle = pi;
            elseif right_dis <= 10
                angle = 7*pi/4;
            else
                angle = 11*pi/5;
            end
        elseif left_dis <= 10
            if right_dis <= 5
                angle = pi/4;
            elseif right_dis <= 10
                angle = 7*pi/4;
            else
                angle = 11*pi/5;
            end           
        else
            if right_dis <= 5
                angle = pi/4;
            elseif right_dis <= 10
                angle = pi/4;
            else
                angle = pi/4;
            end
        end
    elseif forward_dis <= 10
         if left_dis <= 5
            if right_dis <= 5
                angle = 0;
            elseif right_dis <= 10
                angle = pi/6;
            else
                angle = pi/5;
            end
        elseif left_dis <= 10
            if right_dis <= 5
                angle = 7*pi/6;
            elseif right_dis <= 10
                angle = 7*pi/6;
            else
                angle = 7*pi/6;
            end           
        else
            if right_dis <= 5
                angle = 7*pi/4;
            elseif right_dis <= 10
                angle = 7*pi/4;
            else
                angle = 7*pi/4;
            end
        end
    else 
         if left_dis <= 5
            if right_dis <= 5
                angle = 7*pi/4;
            elseif right_dis <= 10
                angle = 7*pi/4;
            else
                angle = 7*pi/4;
            end
        elseif left_dis <= 10
            if right_dis <= 5
                angle = 7*pi/4;
            elseif right_dis <= 10
                angle = 7*pi/4;
            else
                angle = 7*pi/4;
            end           
        else
            if right_dis <= 5
                angle = 7*pi/4;
            elseif right_dis <= 10
                angle = 7*pi/4;
            else
                angle = 7*pi/4;
            end
        end
    end
    disp(sprintf('探测的距离：前方 = %f  左方 = %f   右方 = %f',min_distance(1,1),min_distance(1,2),min_distance(1,3)));
    if detect_t <= 0
       [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_turn(xy_nodes,coordinate_nodes,xy_nodes_world,angle);
       detect_t = 5;
    end
end

function [xy_nodes,coordinate_nodes,xy_nodes_world] = decision(xy_nodes,coordinate_nodes,xy_nodes_world)
%根据是三个方向上的探测的距离信息
    global detect_t;
    [real_distance,min_distance]= distance_123(xy_nodes,coordinate_nodes,xy_nodes_world);
    forward_dis = min_distance(1,1);
    left_dis = min_distance(1,2);
    right_dis = min_distance(1,3);
    if forward_dis <= 15
        if left_dis <= 15
            if left_dis <= right_dis
                angle = 7*pi/4;%右转
            else
                angle = 7*pi/4;
            end
        else
            if left_dis <= right_dis
                angle = 7*pi/4;
            else
                angle = 7*pi/4;
            end
        end
    else
                angle = 0;
    end
    disp(sprintf('探测的距离：前方 = %f  左方 = %f   右方 = %f',min_distance(1,1),min_distance(1,2),min_distance(1,3)));
    if detect_t <= 0 && angle~=0
       [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_turn(xy_nodes,coordinate_nodes,xy_nodes_world,angle);
       detect_t = 6;
    end
end
%%
%%测试玩
%碰到边界后转大角度
function [xy_nodes,coordinate_nodes,xy_nodes_world] = fun(xy_nodes,coordinate_nodes,xy_nodes_world)
    if abs(xy_nodes_world(1,1)) > 40 || abs(xy_nodes_world(2,1)) > 40
        [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_turn(xy_nodes,coordinate_nodes,xy_nodes_world,pi/2);
    end
end

%%
function [x,y] = one_direction(xy_nodes,coordinate_nodes,xy_nodes_world,alpha)
%检测方向判断
    theta = mod(coordinate_nodes(3,1)+alpha,2*pi);
    %k = tan(theta);
    %disp(theta);
    if theta >= 0 && theta <= pi/4
        x = 40;
        y = tan(theta)*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1);
        if y > 40
            y = 40;
            x = (1/tan(theta))*(y-xy_nodes_world(2,1))+xy_nodes_world(1,1);
        elseif y < -40
            y = -40;
            x = (1/tan(theta))*(y-xy_nodes_world(2,1))+xy_nodes_world(1,1);
        end
    elseif theta < pi/2
        y = 40;  
        x = (1/tan(theta))*(y-xy_nodes_world(2,1))+xy_nodes_world(1,1);%k/1应该是，想当然害死人，严谨哈
        if x > 40
            x =40;
            y = tan(theta)*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1);            
        elseif x < -40
            x = -40;
            y = tan(theta)*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1);            
        end
    elseif theta == pi/2
        x = xy_nodes_world(1,1);  
        y = 40;       
    elseif theta <= 3*pi/4
        y = 40;  
        x = (1/tan(theta))*(y-xy_nodes_world(2,1))+xy_nodes_world(1,1);
        if x > 40
            x =40;
            y = tan(theta)*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1);            
        elseif x < -40
            x = -40;
            y = tan(theta)*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1);            
        end       
    elseif theta <= pi
        x = -40; 
        y = tan(theta)*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1);
        if y > 40
            y = 40;
            x = (1/tan(theta))*(y-xy_nodes_world(2,1))+xy_nodes_world(1,1);
        elseif y < -40
            y = -40;
            x = (1/tan(theta))*(y-xy_nodes_world(2,1))+xy_nodes_world(1,1);
        end        
     elseif theta <= 5*pi/4
        x = -40; 
        y = tan(theta)*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1);
        if y > 40
            y = 40;
            x = (1/tan(theta))*(y-xy_nodes_world(2,1))+xy_nodes_world(1,1);
        elseif y < -40
            y = -40;
            x = (1/tan(theta))*(y-xy_nodes_world(2,1))+xy_nodes_world(1,1);
        end         
     elseif theta < 3*pi/2
        y = -40;  
        x = (1/tan(theta))*(y-xy_nodes_world(2,1))+xy_nodes_world(1,1);
        if x > 40
            x =40;
            y = tan(theta)*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1);            
        elseif x < -40
            x = -40;
            y = tan(theta)*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1);            
        end
     elseif theta == 3*pi/2
        x = xy_nodes_world(1,1);  
        y = -40; 
     elseif theta <= 7*pi/4
        y = -40;  
        x = (1/tan(theta))*(y-xy_nodes_world(2,1))+xy_nodes_world(1,1);
        if x > 40
            x =40;
            y = tan(theta)*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1);            
        elseif x < -40
            x = -40;
            y = tan(theta)*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1);            
        end
     elseif theta <= 2*pi
        x = 40; 
        y = tan(theta)*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1);
        if y > 40
            y = 40;
            x = (1/tan(theta))*(y-xy_nodes_world(2,1))+xy_nodes_world(1,1);
        elseif y < -40
            y = -40;
            x = (1/tan(theta))*(y-xy_nodes_world(2,1))+xy_nodes_world(1,1);
        end          
    end
end

function [real_distance,min_distance]= distance_123(xy_nodes,coordinate_nodes,xy_nodes_world)
%求得三个方向上的距离信息
    terminal_pos = three_direction(xy_nodes,coordinate_nodes,xy_nodes_world);
    real_distance = detect_distance(xy_nodes,coordinate_nodes,xy_nodes_world);
    [min_distance(1,1),min_distance(2,1)]= min(real_distance(1,:,3));%前方探测距离与坐标索引
    [min_distance(1,2),min_distance(2,2)]= min(real_distance(2,:,3));%左方探测距离与坐标索引
    [min_distance(1,3),min_distance(2,3)]= min(real_distance(3,:,3));%右方探测距离与坐标索引
        %disp(sprintf('探测的距离：前方 = %f  左方 = %f   右方 = %f',min_distance(1,1),min_distance(1,2),min_distance(1,3)));
end

function dis_test(xy_nodes,coordinate_nodes,xy_nodes_world)
%三个方向上的射线显示
    [real_distance,min_distance]= distance_123(xy_nodes,coordinate_nodes,xy_nodes_world);
    line([xy_nodes_world(1,1) real_distance(1,min_distance(2,1),1)],[xy_nodes_world(2,1) real_distance(1,min_distance(2,1),2)],'LineStyle','-.','Color','r');
    line([xy_nodes_world(1,1) real_distance(2,min_distance(2,2),1)],[xy_nodes_world(2,1) real_distance(2,min_distance(2,2),2)],'LineStyle','-.','Color','r');
    line([xy_nodes_world(1,1) real_distance(3,min_distance(2,3),1)],[xy_nodes_world(2,1) real_distance(3,min_distance(2,3),2)],'LineStyle','-.','Color','r');  
        %disp('对应坐标：x y');disp(real_distance(1,min_distance(2,1),1:2));
        disp(sprintf('对应坐标： x =     %f         %f          %f\n          y =     %f        %f         %f',...
                                real_distance(1,min_distance(2,1),1),real_distance(2,min_distance(2,2),1),real_distance(3,min_distance(2,3),1),...
                                real_distance(1,min_distance(2,1),2),real_distance(2,min_distance(2,2),2),real_distance(3,min_distance(2,3),2)));
end

function terminal_pos = three_direction(xy_nodes,coordinate_nodes,xy_nodes_world)
%模拟三个方向上的探测
%得出三个方向边界点
    [terminal_pos(1,1),terminal_pos(2,1)] = one_direction(xy_nodes,coordinate_nodes,xy_nodes_world,pi);     %前
    [terminal_pos(1,2),terminal_pos(2,2)] = one_direction(xy_nodes,coordinate_nodes,xy_nodes_world,3*pi/2); %左
    [terminal_pos(1,3),terminal_pos(2,3)] = one_direction(xy_nodes,coordinate_nodes,xy_nodes_world,pi/2);   %右
end

function real_xyd = segmentAndcicle(xy1,xy2,xy3,r)
%一段线段一个圆
%线段端点分别为:xy1(x1；y1)--头部,xy2(x2；y2)--边界点,圆的圆心为xy3(x3；y3),r
%计算A,B,C
%求相交坐标，并返回最小距离和与之对应的坐标
    x1=xy1(1,1);x2=xy2(1,1);x3=xy3(1,1);
    y1=xy1(2,1);y2=xy2(2,1);y3=xy3(2,1);
    A = (x2-x1)^2+(y2-y1)^2;
    B = 2*((x2-x1)*(x1-x3)+(y2-y1)*(y1-y3));
    C = x3^2+y3^2+x1^2+y1^2-2*(x3*x1+y3*y1)-r^2;
    u1 = (-B+(B^2-4*A*C)^(1/2))/(2*A);
    u2 = (-B-(B^2-4*A*C)^(1/2))/(2*A);
    
    delta = B^2-4*A*C;
    %线段所在直线和圆的相交情况
    if delta < 0        %表示没有交点
        real_xyd(1:2,1) = [x2;y2];
        real_xyd(3,1) = sqrt((real_xyd(1,1)-x1)^2+(real_xyd(2,1)-y1)^2);
    elseif delta == 0   %表示相切
            if u1 < 1 
                real_xyd(1:2,1) = [x1+u1*(x2-x1);y1+u1*(y2-y1)];
                real_xyd(3,1) = sqrt((real_xyd(1,1)-x1)^2+(real_xyd(2,1)-y1)^2);
            elseif u1 > 1
                real_xyd(1:2,1) = [x2;y2];
                real_xyd(3,1) = sqrt((x2-x1)^2+(y2-y1)^2);
            end
    else                %表示相交两点
            if (u1 < 0) && (u2 < 0)                                %线段在圆外，其反向延长线相交（无效）
                real_xyd(1:2,1) = [x2;y2];
                real_xyd(3,1) = sqrt((x2-x1)^2+(y2-y1)^2);
            elseif (u1 > 1) && (u2 > 1)                            %线段在圆外，其正向延长线相交（无效）
                real_xyd(1:2,1) = [x2;y2];
                real_xyd(3,1) = sqrt((x2-x1)^2+(y2-y1)^2); 
            elseif (u1 > 0 && u1 < 1) && (u2 > 0 && u2 < 1)    %线段与圆相交两点（有效）
                cross1(1:2,1) = [x1+u1*(x2-x1);y1+u1*(y2-y1)];
                cross1(3,1) = sqrt((cross1(1,1)-x1)^2+(cross1(2,1)-y1)^2);
                cross2(1:2,1) = [x1+u2*(x2-x1);y1+u2*(y2-y1)];
                cross2(3,1) = sqrt((cross2(1,1)-x1)^2+(cross2(2,1)-y1)^2);
                if cross1(3,1) > cross2(3,1)
                     real_xyd(1:2,1) = cross2(1:2,1);
                     real_xyd(3,1) = cross2(3,1); 
                else
                     real_xyd(1:2,1) = cross1(1:2,1);
                     real_xyd(3,1) = cross1(3,1); 
                end
            end
    end  
end

function real_distance = detect_distance(xy_nodes,coordinate_nodes,xy_nodes_world)
%三条线段与五个圆-----[有效坐标点x,y,有效距离]
    global obstacle_xy;   
    terminal_pos = three_direction(xy_nodes,coordinate_nodes,xy_nodes_world);
    for i = 1:3
        for j = 1:5
            real_distance(i,j,1:3) = segmentAndcicle(xy_nodes_world(1:2,1),terminal_pos(1:2,i),obstacle_xy(1:2,j),obstacle_xy(3,j));
        end
    end
end
