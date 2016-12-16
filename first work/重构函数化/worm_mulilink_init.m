%function worm_mulilink_init()

nodes = 9;
A=0.5;
B=4*pi/18;
l = 0.9; %模块长

n = nodes-1; %%模块数8
delta = linspace(0,0,n); 
sigma = linspace(0,0,n); 
xy_nodes = zeros(2,n+1);

%delta(1) = pi/4;%各关节间夹角
sigma(1) = 4.45*pi/18;%各关节与水平夹角

for i = 2:n %各关节间夹角
     delta(i) = A*sin((i-1)*B); 
end 
for j = 2:n %各关节与水平夹角
     sigma(j) = sigma(j-1)-delta(j); 
end 

for k = 2:n+1 %%各关节连接点的坐标
     xy_nodes(1:2,k)= [l*cos(sigma(k-1))+xy_nodes(1,k-1);...
                       l*sin(sigma(k-1))+xy_nodes(2,k-1)]; 
end 
%plot(xy_nodes(1,:),xy_nodes(2,:),'-b.'); 
%axis([-0.5 7 -0.5 7]);
%    xlabel('x');ylabel('y');
%    for i=1:1:nodes
%        text(xy_nodes(1,i),xy_nodes(2,i),...
%            ['\leftarrow' num2str(i)],'fontsize',15);
%    end
   
%hold on; 