function worm_init(x,y,theta)
%用途说明：初始化线虫各结点坐标，坐标系偏转值
%参数说明：
%worm_init(头结点(坐标系原点)坐标-x,头结点(坐标系原点)坐标-y,
%          头结点坐标系与主坐标系的偏转角度theta,nodes)
%pi/4(顺时针偏转45度)，原坐标系与世界坐标系方向一致 
global nodes;
global xy_nodes;
global coordinate_nodes;
global xy_nodes_world; 
    theta = -2*pi*theta/360;
    xy_nodes(1:2,1) = [x;y];                                               %随机生成一个头结点        
    coordinate_nodes(1:3,1) = [xy_nodes(1:2,1);theta];                     %初始化一个坐标系（坐标系原点+偏转角度）%头结点坐标系与主坐标系的偏转角度，前行方向
    coordinate_nodes(1:3,1:nodes) = repmat(coordinate_nodes(1:3,1),1,nodes);                                                     
    for i = 1 : nodes             %nodes=1为头结点的坐标,图中显示为-最右-结点
        xy_nodes(1:2,i) = [pi/4*(i-1);sin(pi/4*(i-1))];   
        xy_nodes_world(1:2,i) = [cos(coordinate_nodes(3,i)) -sin(coordinate_nodes(3,i));sin(coordinate_nodes(3,i)) cos(coordinate_nodes(3,i))]...
                                 * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
    end
end