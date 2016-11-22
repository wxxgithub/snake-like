function snake_01()
%����ʵ��0.2��
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
obstacle_xy = [30 -20 -5 -25 25;-20 -20 -5 25 0;5 5 5 5 5];%����

global detect_t;
detect_t = 5;

[xy_nodes,coordinate_nodes,xy_nodes_world] = snake_init(rand(1)*25,rand(1)*25,pi/8);

for count = 1:60
    [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_forward(xy_nodes,coordinate_nodes);
    display(xy_nodes,coordinate_nodes,xy_nodes_world);
    %����
    %if mod(count,30) == 0 
    %    [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_turn(xy_nodes,coordinate_nodes,xy_nodes_world,pi/4);
    %end
    [xy_nodes,coordinate_nodes,xy_nodes_world] = decision(xy_nodes,coordinate_nodes,xy_nodes_world);
    
    [xy_nodes,coordinate_nodes,xy_nodes_world] = fun(xy_nodes,coordinate_nodes,xy_nodes_world);
    %%�����鿴
    disp('count = ');disp(count);
    disp('xy_nodes_world = ');disp(xy_nodes_world);
    disp('coordinate_nodes = ');disp(coordinate_nodes);
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
      nodes_trajectory = [nodes_trajectory xy_nodes_world(:,nodes)];
      plot(xy_nodes_world(1,:),xy_nodes_world(2,:),'-r.',nodes_trajectory(1,:),nodes_trajectory(2,:),'-k.');
      
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
      
      axis([-40 40 -40 40]);
      axis square;
      pause(0.2);
end

%%
%�������һ���ĵ���Ϊ�ϰ������ĵ����꣬��������״�ɹ�ѡ��
function obstacle(radius,x0,y0,shape,rotate)
    theta=linspace(0,2*pi,shape+1)+rotate;
    hold on;
    line(radius*cos(theta)+x0,radius*sin(theta)+y0);
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
function [xy_nodes,coordinate_nodes,xy_nodes_world] = decision(xy_nodes,coordinate_nodes,xy_nodes_world)
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

%%
%%������
%�����߽��ת��Ƕ�
function [xy_nodes,coordinate_nodes,xy_nodes_world] = fun(xy_nodes,coordinate_nodes,xy_nodes_world)
    if abs(xy_nodes_world(1,1)) > 40 || abs(xy_nodes_world(2,1)) > 40
        [xy_nodes,coordinate_nodes,xy_nodes_world] = snake_turn(xy_nodes,coordinate_nodes,xy_nodes_world,pi/2);
    end
end

%%
%���������ϵ�������ʾ
function dis_test(xy_nodes,coordinate_nodes,xy_nodes_world)
    terminal_pos = zeros(3,3);
    [terminal_pos(1,1),terminal_pos(2,1)] = three_direction(xy_nodes,coordinate_nodes,xy_nodes_world,pi);%ǰ
    [terminal_pos(1,2),terminal_pos(2,2)] = three_direction(xy_nodes,coordinate_nodes,xy_nodes_world,3*pi/2);%��
    [terminal_pos(1,3),terminal_pos(3,3)] = three_direction(xy_nodes,coordinate_nodes,xy_nodes_world,pi/2);%��
    line([xy_nodes_world(1,1) terminal_pos(1,1)],[xy_nodes_world(2,1) terminal_pos(2,1)],'LineStyle','-.','Color','r');
    line([xy_nodes_world(1,1) terminal_pos(1,2)],[xy_nodes_world(2,1) terminal_pos(2,2)],'LineStyle','-.','Color','r');
    line([xy_nodes_world(1,1) terminal_pos(1,3)],[xy_nodes_world(2,1) terminal_pos(3,3)],'LineStyle','-.','Color','r');  
end
%%
%ģ�����������ϵ�̽��
%�ó��߽��
function [x,y] = three_direction(xy_nodes,coordinate_nodes,xy_nodes_world,alpha)
    theta = coordinate_nodes(3,1)+alpha;
    k = tan(theta);
    if theta >= 0 && theta <= pi/2
        x = 40;
        y = tan(theta)*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1);
    else if theta <= pi
        x = -40;
        y = tan(theta)*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1);
    else if theta <= 3*pi/2
        x = -40; 
        y = tan(theta)*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1);
    else if theta 
        x = 40; 
        y = tan(theta)*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1);
         end
         end
         end    
    end
end


