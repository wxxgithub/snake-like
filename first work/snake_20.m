function snake_20()
%����ʵ��0.2��
%��Ϸ�����ľ��봫����������Ӧ�Ŀ����㷨if-else����
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

global obstacle_num;
obstacle_num = 5;                       %��������ϰ�����Ŀ

global nodes;
global nodes_trajectory;
nodes_trajectory = [];                  %�켣����Ĵ���

global obstacle_xy;
obstacle_xy = zeros(2,obstacle_num);
obstacle_xy = [-20 20 0 -20 20;20 20 0 -20 -20;5 5 5 5 5];%����

global detect_t;
detect_t = 5;

[xy_nodes,coordinate_nodes,xy_nodes_world] = snake_init(25,0,0);
     %figure ;
for count = 1:350
    [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_forward(xy_nodes,coordinate_nodes);
    
    %disp('count = ');disp(count);
    disp(sprintf('\ncount = %d',count));
    display(xy_nodes,coordinate_nodes,xy_nodes_world);
    %����
    %if mod(count,30) == 0 
    %    [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_turn(xy_nodes,coordinate_nodes,xy_nodes_world,pi/4);
    %end
    [xy_nodes,coordinate_nodes,xy_nodes_world] = decision(xy_nodes,coordinate_nodes,xy_nodes_world);
    
    %[xy_nodes,coordinate_nodes,xy_nodes_world]=fun(xy_nodes,coordinate_nodes,xy_nodes_world);%���Ա߽����
    %%�����鿴

    %disp('xy_nodes_world = ');disp(xy_nodes_world);
    %disp('coordinate_nodes = ');disp(coordinate_nodes);
    %real_distance = detect_distance(xy_nodes,coordinate_nodes,xy_nodes_world);
    %disp('real_distance = ���ֿɷ�');disp(real_distance(1,:,:));
end

end

%%
%��ʼ��λ�ã�����������
%snake_init(ͷ���(����ϵԭ��)����-x,ͷ���(����ϵԭ��)����-y,ͷ�������ϵ��������ϵ��ƫת�Ƕ�theta,nodes)
%pi/4(��ʱ��ƫת45��)��ԭ����ϵ����������ϵ����һ��
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
end

%%
%����ǰ���˶�
function [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_forward(xy_nodes,coordinate_nodes)
       global nodes;
       global nodes_trajectory;
       global detect_t;
       
       for i = 1 : nodes-1
            xy_nodes(1:2,nodes+1-i) = [xy_nodes(1:2,nodes-i)];                  %��ͷ����⣬������һ�δ�������ֵ(...��2->3��1->2)
            coordinate_nodes(1:3,nodes+1-i) = coordinate_nodes(1:3,nodes-i);    %��ͷ����⣬�������Ӧ����ϵԭ�㼰�Ƕ�ֵ
       end
       xy_nodes(1:2,1) = [xy_nodes(1,1)-pi/4;sin(xy_nodes(1,1)-pi/4)];
       coordinate_nodes(1:3,1) = coordinate_nodes(1:3,1);
       for i = 1 : nodes                                                      %ת��Ϊ������ϵ��
            xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))] ...
                               * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
       end
       detect_t = detect_t - 1;
end

%%
%�˶�ת��
%Ctheta������pi/4��-��ʱ��ƫת�Ƕ�
%ƫת�Ƕ�ȡֵ��Χ��0<=Ctheta<=90
function [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_turn(xy_nodes,coordinate_nodes,xy_nodes_world,Ctheta)
        global nodes;
    
        for i = 1 : nodes-1
            xy_nodes(1:2,nodes+1-i) = [xy_nodes(1:2,nodes-i)];                  %��ͷ����⣬������һ�δ�������ֵ
            coordinate_nodes(1:3,nodes+1-i) = coordinate_nodes(1:3,nodes-i);    %��ͷ����⣬�������Ӧ����ϵԭ�㼰�Ƕ�ֵ
        end
        %��������ԭ������ϵ�У���һ�����ָ�������ϵ�е�ͷβ��㼰��仯���ڶ�����λ֮��ı仯��
        xy_nodes(1:2,1) = [0;0];
        coordinate_nodes(1:3,1) = [xy_nodes_world(1:2,1);mod(coordinate_nodes(3,1)+Ctheta,2*pi)];            %%����ϵ�ĽǶ���˳ʱ��
        for i = 1 : nodes                                                      %ת��Ϊ������ϵ��
            xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))] ...
                               * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
        end
end
    
%%
%�ƶ��켣��ʾ
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
%�������һ���ĵ���Ϊ�ϰ������ĵ����꣬��������״�ɹ�ѡ��
function obstacle(radius,x0,y0,shape,rotate)
    theta=linspace(0,2*pi,shape+1)+rotate;
    hold on;
    line(radius*cos(theta)+x0,radius*sin(theta)+y0);
    hold off;
end

%%
%����������ϰ���ľ���
function neardist = calculte(xy_nodes_world)
    global obstacle_num;
    global obstacle_xy;
        %obstacle_xy = rand(3,5)*25;
        obstacle = [xy_nodes_world(1:2,1) obstacle_xy(1:2,:)]';
        D = pdist(obstacle,'euclidean');
        D_obstacle = D(1,1:obstacle_num);
        [neardist(1,1),neardist(1,2)] = min(D_obstacle);                                %������С����
end

%%
%���������ת�ǵĹ�ϵ--�ؼ�
function [xy_nodes,coordinate_nodes,xy_nodes_world] = decision0(xy_nodes,coordinate_nodes,xy_nodes_world)
%����ľ�������
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
%���������������ϵ�̽��ľ�����Ϣ
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
    disp(sprintf('̽��ľ��룺ǰ�� = %f  �� = %f   �ҷ� = %f',min_distance(1,1),min_distance(1,2),min_distance(1,3)));
    if detect_t <= 0
       [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_turn(xy_nodes,coordinate_nodes,xy_nodes_world,angle);
       detect_t = 5;
    end
end

function [xy_nodes,coordinate_nodes,xy_nodes_world] = decision(xy_nodes,coordinate_nodes,xy_nodes_world)
%���������������ϵ�̽��ľ�����Ϣ
    global detect_t;
    [real_distance,min_distance]= distance_123(xy_nodes,coordinate_nodes,xy_nodes_world);
    forward_dis = min_distance(1,1);
    left_dis = min_distance(1,2);
    right_dis = min_distance(1,3);
    if forward_dis <= 15
        if left_dis <= 15
            if left_dis <= right_dis
                angle = 7*pi/4;%��ת
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
    disp(sprintf('̽��ľ��룺ǰ�� = %f  �� = %f   �ҷ� = %f',min_distance(1,1),min_distance(1,2),min_distance(1,3)));
    if detect_t <= 0 && angle~=0
       [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_turn(xy_nodes,coordinate_nodes,xy_nodes_world,angle);
       detect_t = 6;
    end
end
%%
%%������
%�����߽��ת��Ƕ�
function [xy_nodes,coordinate_nodes,xy_nodes_world] = fun(xy_nodes,coordinate_nodes,xy_nodes_world)
    if abs(xy_nodes_world(1,1)) > 40 || abs(xy_nodes_world(2,1)) > 40
        [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_turn(xy_nodes,coordinate_nodes,xy_nodes_world,pi/2);
    end
end

%%
function [x,y] = one_direction(xy_nodes,coordinate_nodes,xy_nodes_world,alpha)
%��ⷽ���ж�
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
        x = (1/tan(theta))*(y-xy_nodes_world(2,1))+xy_nodes_world(1,1);%k/1Ӧ���ǣ��뵱Ȼ�����ˣ��Ͻ���
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
%������������ϵľ�����Ϣ
    terminal_pos = three_direction(xy_nodes,coordinate_nodes,xy_nodes_world);
    real_distance = detect_distance(xy_nodes,coordinate_nodes,xy_nodes_world);
    [min_distance(1,1),min_distance(2,1)]= min(real_distance(1,:,3));%ǰ��̽���������������
    [min_distance(1,2),min_distance(2,2)]= min(real_distance(2,:,3));%��̽���������������
    [min_distance(1,3),min_distance(2,3)]= min(real_distance(3,:,3));%�ҷ�̽���������������
        %disp(sprintf('̽��ľ��룺ǰ�� = %f  �� = %f   �ҷ� = %f',min_distance(1,1),min_distance(1,2),min_distance(1,3)));
end

function dis_test(xy_nodes,coordinate_nodes,xy_nodes_world)
%���������ϵ�������ʾ
    [real_distance,min_distance]= distance_123(xy_nodes,coordinate_nodes,xy_nodes_world);
    line([xy_nodes_world(1,1) real_distance(1,min_distance(2,1),1)],[xy_nodes_world(2,1) real_distance(1,min_distance(2,1),2)],'LineStyle','-.','Color','r');
    line([xy_nodes_world(1,1) real_distance(2,min_distance(2,2),1)],[xy_nodes_world(2,1) real_distance(2,min_distance(2,2),2)],'LineStyle','-.','Color','r');
    line([xy_nodes_world(1,1) real_distance(3,min_distance(2,3),1)],[xy_nodes_world(2,1) real_distance(3,min_distance(2,3),2)],'LineStyle','-.','Color','r');  
        %disp('��Ӧ���꣺x y');disp(real_distance(1,min_distance(2,1),1:2));
        disp(sprintf('��Ӧ���꣺ x =     %f         %f          %f\n          y =     %f        %f         %f',...
                                real_distance(1,min_distance(2,1),1),real_distance(2,min_distance(2,2),1),real_distance(3,min_distance(2,3),1),...
                                real_distance(1,min_distance(2,1),2),real_distance(2,min_distance(2,2),2),real_distance(3,min_distance(2,3),2)));
end

function terminal_pos = three_direction(xy_nodes,coordinate_nodes,xy_nodes_world)
%ģ�����������ϵ�̽��
%�ó���������߽��
    [terminal_pos(1,1),terminal_pos(2,1)] = one_direction(xy_nodes,coordinate_nodes,xy_nodes_world,pi);     %ǰ
    [terminal_pos(1,2),terminal_pos(2,2)] = one_direction(xy_nodes,coordinate_nodes,xy_nodes_world,3*pi/2); %��
    [terminal_pos(1,3),terminal_pos(2,3)] = one_direction(xy_nodes,coordinate_nodes,xy_nodes_world,pi/2);   %��
end

function real_xyd = segmentAndcicle(xy1,xy2,xy3,r)
%һ���߶�һ��Բ
%�߶ζ˵�ֱ�Ϊ:xy1(x1��y1)--ͷ��,xy2(x2��y2)--�߽��,Բ��Բ��Ϊxy3(x3��y3),r
%����A,B,C
%���ཻ���꣬��������С�������֮��Ӧ������
    x1=xy1(1,1);x2=xy2(1,1);x3=xy3(1,1);
    y1=xy1(2,1);y2=xy2(2,1);y3=xy3(2,1);
    A = (x2-x1)^2+(y2-y1)^2;
    B = 2*((x2-x1)*(x1-x3)+(y2-y1)*(y1-y3));
    C = x3^2+y3^2+x1^2+y1^2-2*(x3*x1+y3*y1)-r^2;
    u1 = (-B+(B^2-4*A*C)^(1/2))/(2*A);
    u2 = (-B-(B^2-4*A*C)^(1/2))/(2*A);
    
    delta = B^2-4*A*C;
    %�߶�����ֱ�ߺ�Բ���ཻ���
    if delta < 0        %��ʾû�н���
        real_xyd(1:2,1) = [x2;y2];
        real_xyd(3,1) = sqrt((real_xyd(1,1)-x1)^2+(real_xyd(2,1)-y1)^2);
    elseif delta == 0   %��ʾ����
            if u1 < 1 
                real_xyd(1:2,1) = [x1+u1*(x2-x1);y1+u1*(y2-y1)];
                real_xyd(3,1) = sqrt((real_xyd(1,1)-x1)^2+(real_xyd(2,1)-y1)^2);
            elseif u1 > 1
                real_xyd(1:2,1) = [x2;y2];
                real_xyd(3,1) = sqrt((x2-x1)^2+(y2-y1)^2);
            end
    else                %��ʾ�ཻ����
            if (u1 < 0) && (u2 < 0)                                %�߶���Բ�⣬�䷴���ӳ����ཻ����Ч��
                real_xyd(1:2,1) = [x2;y2];
                real_xyd(3,1) = sqrt((x2-x1)^2+(y2-y1)^2);
            elseif (u1 > 1) && (u2 > 1)                            %�߶���Բ�⣬�������ӳ����ཻ����Ч��
                real_xyd(1:2,1) = [x2;y2];
                real_xyd(3,1) = sqrt((x2-x1)^2+(y2-y1)^2); 
            elseif (u1 > 0 && u1 < 1) && (u2 > 0 && u2 < 1)    %�߶���Բ�ཻ���㣨��Ч��
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
%�����߶������Բ-----[��Ч�����x,y,��Ч����]
    global obstacle_xy;   
    terminal_pos = three_direction(xy_nodes,coordinate_nodes,xy_nodes_world);
    for i = 1:3
        for j = 1:5
            real_distance(i,j,1:3) = segmentAndcicle(xy_nodes_world(1:2,1),terminal_pos(1:2,i),obstacle_xy(1:2,j),obstacle_xy(3,j));
        end
    end
end
