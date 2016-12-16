clc;
clear;

global nodes;
global xy_nodes;
global coordinate_nodes;
global xy_nodes_world;
global obstacle_num;
global obstacle_xy;
global nodes_trajectory;

	init();
	fisMat = readfis('fuzzy16');
	worm_init(10,-35,-160);
    [three_dis,real_distance,min_distance]=obstacle_detect();	  
              
     motion_display(real_distance,min_distance);
        
     for count =1:60 %1---nodes(8)����һ������
                     %9---ת����һ��
                     %10-16---��7��
       if mod(xy_nodes(1,nodes),2*pi) == 0
          angle=decision_Fuzzy(fisMat,three_dis);
          worm_turn(angle);%ת������һ��
          continue;
       end
       %if mod(count,7) == 0

         % motion_display(real_distance,min_distance);
        %  continue;
       %end
          worm_motion();
          [three_dis,real_distance,min_distance]=obstacle_detect(); 
          
          motion_display(real_distance,min_distance);
       
     end
