function init()
%用途说明：初始化线虫参数
%参数说明：
%          nodes			：线虫结点个数
%          xy_nodes			：各结点实时坐标值
%          coordinate_nodes	：各结点坐标系
%          obstacle_num		：障碍物数量
%          obstacle_xy		：障碍物中心坐标值
%          nodes_trajectory	：轨迹坐标存储器
%日    期：2016-12-12     
%作    者：wx
%地    点：信科1902
global nodes;
global xy_nodes;
global coordinate_nodes;
global xy_nodes_world;
global obstacle_num;
global obstacle_xy;
global nodes_trajectory;

nodes = 9;                        
xy_nodes = zeros(2,nodes); 
xy_nodes_world = zeros(2,nodes);
coordinate_nodes = zeros(3,nodes);  
                                   
obstacle_num = 5;                  
nodes_trajectory = [];            

obstacle_xy = zeros(2,obstacle_num);
obstacle_xy = [30 -20 -5 -25 25;-20 -20 -5 25 0;5 5 5 5 5];

%detect_t = 5;

end