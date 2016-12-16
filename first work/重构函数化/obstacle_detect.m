function [three_dis,real_distance,min_distance]=obstacle_detect()
%函数功能：三个方向距离值
%函数参数：
%           three_dis    :前、左、右探测到的距离
%           real_distance：（三个方向线段：五个障碍物圆；相交坐标和距离（x;y;d））
%           min_distance ：[前方距离，左方距离，右方距离；前方坐标索引，左方坐标索引，右方坐标索引]
%三个方向上的射线显示
global nodes;
global xy_nodes;
global coordinate_nodes;
global xy_nodes_world;
global obstacle_num;
global obstacle_xy;
    [real_distance,min_distance]= distance_123();
    forward_dis = min_distance(1,1);
    left_dis = min_distance(1,2);
    right_dis = min_distance(1,3);
    three_dis=[forward_dis;left_dis;right_dis];
%hold on;


%hold off;
    %disp('对应坐标：x y');disp(real_distance(1,min_distance(2,1),1:2));
    %     disp(sprintf('对应坐标： x =     %f         %f          %f\n          y =     %f        %f         %f',...
    %                           real_distance(1,min_distance(2,1),1),real_distance(2,min_distance(2,2),1),real_distance(3,min_distance(2,3),1),...
    %                            real_distance(1,min_distance(2,1),2),real_distance(2,min_distance(2,2),2),real_distance(3,min_distance(2,3),2)));

%%
function [real_distance,min_distance]= distance_123()
%求得三个方向上的距离信息
    terminal_pos = three_direction();
    real_distance = detect_distance();
    [min_distance(1,1),min_distance(2,1)]= min(real_distance(1,:,3));%前方探测距离与坐标索引
    [min_distance(1,2),min_distance(2,2)]= min(real_distance(2,:,3));%左方探测距离与坐标索引
    [min_distance(1,3),min_distance(2,3)]= min(real_distance(3,:,3));%右方探测距离与坐标索引
        %disp(sprintf('探测的距离：前方 = %f  左方 = %f   右方 = %f',min_distance(1,1),min_distance(1,2),min_distance(1,3)));
end
%%

function real_distance = detect_distance()
%
%三条线段与五个圆-----[有效坐标点x,y,有效距离]
    terminal_pos = three_direction();
    for i = 1:3
        for j = 1:5
            real_distance(i,j,1:3) = segmentAndcicle(xy_nodes_world(1:2,nodes),terminal_pos(1:2,i),obstacle_xy(1:2,j),obstacle_xy(3,j));
        end
    end
end
%%
function terminal_pos = three_direction()
%模拟三个方向上的探测
%得出三个方向边界点
    [terminal_pos(1,1),terminal_pos(2,1)] = one_direction(0);     %前
    [terminal_pos(1,2),terminal_pos(2,2)] = one_direction(pi/2); %左
    [terminal_pos(1,3),terminal_pos(2,3)] = one_direction(-pi/2);   %右
end

function [x,y] = one_direction(alpha)
%函数说明：检测某方向（前，左、右）头部与边界的距离
%         返回射线边界点坐标
%检测方向判断
    theta = mod(coordinate_nodes(3,nodes)+alpha,2*pi);
    if theta >= 0 && theta <= pi/4
        x = 40;
        y = tan(theta)*(x-xy_nodes_world(1,nodes))+xy_nodes_world(2,nodes);
        if y > 40
            y = 40;
            x = (1/tan(theta))*(y-xy_nodes_world(2,nodes))+xy_nodes_world(1,nodes);
        elseif y < -40
            y = -40;
            x = (1/tan(theta))*(y-xy_nodes_world(2,nodes))+xy_nodes_world(1,nodes);
        end
    elseif theta < pi/2
        y = 40;  
        x = (1/tan(theta))*(y-xy_nodes_world(2,nodes))+xy_nodes_world(1,nodes);%k/1应该是，想当然害死人，严谨哈
        if x > 40
            x =40;
            y = tan(theta)*(x-xy_nodes_world(1,nodes))+xy_nodes_world(2,nodes);            
        elseif x < -40
            x = -40;
            y = tan(theta)*(x-xy_nodes_world(1,nodes))+xy_nodes_world(2,nodes);            
        end
    elseif theta == pi/2
        x = xy_nodes_world(1,nodes);  
        y = 40;       
    elseif theta <= 3*pi/4
        y = 40;  
        x = (1/tan(theta))*(y-xy_nodes_world(2,nodes))+xy_nodes_world(1,nodes);
        if x > 40
            x =40;
            y = tan(theta)*(x-xy_nodes_world(1,nodes))+xy_nodes_world(2,nodes);            
        elseif x < -40
            x = -40;
            y = tan(theta)*(x-xy_nodes_world(1,nodes))+xy_nodes_world(2,nodes);            
        end       
    elseif theta <= pi
        x = -40; 
        y = tan(theta)*(x-xy_nodes_world(1,nodes))+xy_nodes_world(2,nodes);
        if y > 40
            y = 40;
            x = (1/tan(theta))*(y-xy_nodes_world(2,nodes))+xy_nodes_world(1,nodes);
        elseif y < -40
            y = -40;
            x = (1/tan(theta))*(y-xy_nodes_world(2,nodes))+xy_nodes_world(1,nodes);
        end        
     elseif theta <= 5*pi/4
        x = -40; 
        y = tan(theta)*(x-xy_nodes_world(1,nodes))+xy_nodes_world(2,nodes);
        if y > 40
            y = 40;
            x = (1/tan(theta))*(y-xy_nodes_world(2,nodes))+xy_nodes_world(1,nodes);
        elseif y < -40
            y = -40;
            x = (1/tan(theta))*(y-xy_nodes_world(2,nodes))+xy_nodes_world(1,nodes);
        end         
     elseif theta < 3*pi/2
        y = -40;  
        x = (1/tan(theta))*(y-xy_nodes_world(2,nodes))+xy_nodes_world(1,nodes);
        if x > 40
            x =40;
            y = tan(theta)*(x-xy_nodes_world(1,nodes))+xy_nodes_world(2,nodes);            
        elseif x < -40
            x = -40;
            y = tan(theta)*(x-xy_nodes_world(1,nodes))+xy_nodes_world(2,nodes);            
        end
     elseif theta == 3*pi/2
        x = xy_nodes_world(1,nodes);  
        y = -40; 
     elseif theta <= 7*pi/4
        y = -40;  
        x = (1/tan(theta))*(y-xy_nodes_world(2,nodes))+xy_nodes_world(1,nodes);
        if x > 40
            x =40;
            y = tan(theta)*(x-xy_nodes_world(1,nodes))+xy_nodes_world(2,nodes);            
        elseif x < -40
            x = -40;
            y = tan(theta)*(x-xy_nodes_world(1,nodes))+xy_nodes_world(2,nodes);            
        end
     elseif theta <= 2*pi
        x = 40; 
        y = tan(theta)*(x-xy_nodes_world(1,nodes))+xy_nodes_world(2,nodes);
        if y > 40
            y = 40;
            x = (1/tan(theta))*(y-xy_nodes_world(2,nodes))+xy_nodes_world(1,nodes);
        elseif y < -40
            y = -40;
            x = (1/tan(theta))*(y-xy_nodes_world(2,nodes))+xy_nodes_world(1,nodes);
        end          
    end
end

%%
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

end
