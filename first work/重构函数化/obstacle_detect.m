function [three_dis,real_distance,min_distance]=obstacle_detect()
%�������ܣ������������ֵ
%����������
%           three_dis    :ǰ������̽�⵽�ľ���
%           real_distance�������������߶Σ�����ϰ���Բ���ཻ����;��루x;y;d����
%           min_distance ��[ǰ�����룬�󷽾��룬�ҷ����룻ǰ�������������������������ҷ���������]
%���������ϵ�������ʾ
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
    %disp('��Ӧ���꣺x y');disp(real_distance(1,min_distance(2,1),1:2));
    %     disp(sprintf('��Ӧ���꣺ x =     %f         %f          %f\n          y =     %f        %f         %f',...
    %                           real_distance(1,min_distance(2,1),1),real_distance(2,min_distance(2,2),1),real_distance(3,min_distance(2,3),1),...
    %                            real_distance(1,min_distance(2,1),2),real_distance(2,min_distance(2,2),2),real_distance(3,min_distance(2,3),2)));

%%
function [real_distance,min_distance]= distance_123()
%������������ϵľ�����Ϣ
    terminal_pos = three_direction();
    real_distance = detect_distance();
    [min_distance(1,1),min_distance(2,1)]= min(real_distance(1,:,3));%ǰ��̽���������������
    [min_distance(1,2),min_distance(2,2)]= min(real_distance(2,:,3));%��̽���������������
    [min_distance(1,3),min_distance(2,3)]= min(real_distance(3,:,3));%�ҷ�̽���������������
        %disp(sprintf('̽��ľ��룺ǰ�� = %f  �� = %f   �ҷ� = %f',min_distance(1,1),min_distance(1,2),min_distance(1,3)));
end
%%

function real_distance = detect_distance()
%
%�����߶������Բ-----[��Ч�����x,y,��Ч����]
    terminal_pos = three_direction();
    for i = 1:3
        for j = 1:5
            real_distance(i,j,1:3) = segmentAndcicle(xy_nodes_world(1:2,nodes),terminal_pos(1:2,i),obstacle_xy(1:2,j),obstacle_xy(3,j));
        end
    end
end
%%
function terminal_pos = three_direction()
%ģ�����������ϵ�̽��
%�ó���������߽��
    [terminal_pos(1,1),terminal_pos(2,1)] = one_direction(0);     %ǰ
    [terminal_pos(1,2),terminal_pos(2,2)] = one_direction(pi/2); %��
    [terminal_pos(1,3),terminal_pos(2,3)] = one_direction(-pi/2);   %��
end

function [x,y] = one_direction(alpha)
%����˵�������ĳ����ǰ�����ң�ͷ����߽�ľ���
%         �������߽߱������
%��ⷽ���ж�
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
        x = (1/tan(theta))*(y-xy_nodes_world(2,nodes))+xy_nodes_world(1,nodes);%k/1Ӧ���ǣ��뵱Ȼ�����ˣ��Ͻ���
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

end
