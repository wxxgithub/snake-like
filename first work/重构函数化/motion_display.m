function motion_display(real_distance,min_distance)
%用途说明：轨迹显示+障碍物显示
%参数说明：
%
global nodes;
global xy_nodes;
global coordinate_nodes;
global xy_nodes_world;
global obstacle_num;
global obstacle_xy;
global nodes_trajectory;
    axisnum = 50;
      nodes_trajectory = [nodes_trajectory xy_nodes_world(:,nodes)];

      %plot(xy_nodes_world(1,:),xy_nodes_world(2,:),'-b.',nodes_trajectory(1,:),nodes_trajectory(2,:),'-k');
      plot(xy_nodes_world(1,:),xy_nodes_world(2,:),'-b.');
        hold on;
      obstacle(obstacle_xy(3,1),obstacle_xy(1,1),obstacle_xy(2,1),300,-pi/6);
      obstacle(obstacle_xy(3,2),obstacle_xy(1,2),obstacle_xy(2,2),300,-pi/6);
      obstacle(obstacle_xy(3,3),obstacle_xy(1,3),obstacle_xy(2,3),300,-pi/6);
      obstacle(obstacle_xy(3,4),obstacle_xy(1,4),obstacle_xy(2,4),300,-pi/6);
      obstacle(obstacle_xy(3,5),obstacle_xy(1,5),obstacle_xy(2,5),300,-pi/6);
      %hold off;
            %x=-20:1:20;
            %plot(x,tan(coordinate_nodes(3,1))*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1),'g-.',...
            %x,tan(coordinate_nodes(3,1)-pi/2)*(x-xy_nodes_world(1,1))+xy_nodes_world(2,1),'r-.');
            %end
      %plot(x,xy_nodes_world(2,1),'r-.');
      %plot(x,(x-xy_nodes_world(1,1))+xy_nodes_world(2,1),'r-.');
      
%      dis_test(xy_nodes,coordinate_nodes,xy_nodes_world);
      %sense_dis(real_distance,min_distance);
 

      axis([-axisnum axisnum -axisnum axisnum]);
      axis square;
      pause(0.2);
      %hold on;


%%
function obstacle(radius,x0,y0,shape,rotate)
%随机生成一中心店作为障碍物中心点坐标，有三个形状可供选择
%函数示例：
%obstacle(obstacle_xy(3,1),obstacle_xy(1,1),obstacle_xy(2,1),300,-pi/6);--圆       
    theta=linspace(0,2*pi,shape+1)+rotate;
    %hold on;
    line(radius*cos(theta)+x0,radius*sin(theta)+y0);
end

function sense_dis(real_distance,min_distance)
    line([xy_nodes_world(1,nodes) real_distance(1,min_distance(2,1),1)],[xy_nodes_world(2,nodes) real_distance(1,min_distance(2,1),2)],'LineStyle','-.','Color','r');
    line([xy_nodes_world(1,nodes) real_distance(2,min_distance(2,2),1)],[xy_nodes_world(2,nodes) real_distance(2,min_distance(2,2),2)],'LineStyle','-.','Color','r');
    line([xy_nodes_world(1,nodes) real_distance(3,min_distance(2,3),1)],[xy_nodes_world(2,nodes) real_distance(3,min_distance(2,3),2)],'LineStyle','-.','Color','r');
end

end