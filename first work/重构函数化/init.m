function init()
%��;˵������ʼ���߳����
%����˵����
%          nodes			���߳������
%          xy_nodes			�������ʵʱ����ֵ
%          coordinate_nodes	�����������ϵ
%          obstacle_num		���ϰ�������
%          obstacle_xy		���ϰ�����������ֵ
%          nodes_trajectory	���켣����洢��
%��    �ڣ�2016-12-12     
%��    �ߣ�wx
%��    �㣺�ſ�1902
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