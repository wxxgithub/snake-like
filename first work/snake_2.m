%%
%初始参数设置
bodys = 0 : 0.1 : 1;                    %身体长度0.1
nodes = size(bodys,2);                  %节点个数
xy_nodes = zeros(2,nodes);              %节点[x;y] 
coordinate_nodes = zeros(2,nodes);      %各个节点次坐标系原点(x0,y0)
head_position = zeros(2,1);             %每次偏转前的头结点坐标寄存
beta = 0;

%%
%初始化头部坐标系
%xy_nodes(1:2,nodes) = rand(2,1)*5;    %随机生成一个头结点
theta = rand(1)*pi;                   %头结点坐标系与主坐标系的偏转角度，前行方向
for i = 1 : nodes
    xy_nodes(1:2,i) = [-pi/4*(i-1);sin(-pi/4*(i-1)+pi/2)];
end
coordinate_nodes(:,:) = repmat(xy_nodes(1:2,1),1,11);
plot(xy_nodes(1,:),xy_nodes(2,:),'-*');
%hold on
for i = 1 : nodes
    xy_nodes1(1:2,i) = [cos(theta) -sin(theta);sin(theta) cos(theta)] * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
end
plot(xy_nodes1(1,:),xy_nodes1(2,:),'-*');

%%
%无偏转情况下前行 
%for count = 1:20
%for i = 1 : nodes-1
%    xy_nodes(1:2,nodes+1-i) = xy_nodes(1:2,nodes-i);
%end
%    xy_nodes(1:2,1) = [xy_nodes(1,1)+pi/4;sin(xy_nodes(1,1)+pi/4+pi/2)];    
%    for i = 1 : nodes
%        xy_nodes1(1:2,i) = [cos(theta) -sin(theta);sin(theta) cos(theta)] * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,1); 
%    end
%    plot(xy_nodes1(1,:),xy_nodes1(2,:),'-k.');
%    axis([-10 40 -10 40]);
%    pause(0.5);
%end

%%
%有偏转情况下前行
for count = 1 : 30
    for i = 1 : nodes-1
        xy_nodes(1:2,nodes+1-i) = xy_nodes(1:2,nodes-i);
        coordinate_nodes(1:2,nodes+1-i) = coordinate_nodes(1:2,nodes-i);
    end
        head_position(1:2,1) = xy_nodes(1:2,1);
        coordinate_nodes(1:2,2) = coordinate_nodes(1:2,1);
        xy_nodes(1:2,1) = [xy_nodes(1,1)+pi/4;sin(xy_nodes(1,1)+pi/4+pi/2)]; 
        
    if count == 15
        beta = 30 * pi/180;
        xy_nodes(1:2,1) =  [cos(beta) -sin(beta);cos(beta) sin(beta)] * (xy_nodes(1:2,1)-head_position(1:2,1)) + head_position(1:2,1);
        coordinate_nodes(1:2,1) = xy_nodes(1:2,1);
    end
    %主坐标系转换
    for i = 1 : nodes
        xy_nodes1(1:2,i) = [cos(theta-beta) -sin(theta-beta);sin(theta-beta) cos(theta-beta)] * [xy_nodes(1,i);xy_nodes(2,i)] + coordinate_nodes(1:2,i); 
    end
    plot(xy_nodes1(1,:),xy_nodes1(2,:),'-k.');
    axis([-10 40 -10 40]);
    pause(0.5);
end

