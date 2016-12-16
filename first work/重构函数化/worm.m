A=0.5;
B=2*pi/9;
%function draw(A,B) 
%%模块数
n = 8; 

delta = linspace(0,0,n); %[0,0...,0] n个0
sigma = linspace(0,0,n); %[0,0...,0]
nextdot_y = zeros(1,n+1); %合在一起写
nextdot_x = zeros(1,n+1); 
%%模块长
l = 0.4; 

%%初始角
%delta(1) = pi/4;%delta(1) = (A/2)/sin(B/2); 
sigma(1) = pi/4;%sigma(1) = (A/2)/sin(B/2); 

%%各关节间夹角
for i = 2:n 
     delta(i) = A*sin((i-1)*B); 
end 

%%各关节与水平夹角
for j = 2:n 
     sigma(j) = sigma(j-1)-delta(j); 
end 

%%各关节连接点的坐标
nextdot_y(1) = 0; 
nextdot_x(1) = 0; 
for k = 2:n+1 
     nextdot_y(k) = l*sin(sigma(k-1))+nextdot_y(k-1); 
     nextdot_x(k) = l*cos(sigma(k-1))+nextdot_x(k-1); 
end 
plot(nextdot_x,nextdot_y); 
axis([-1.5 7 -1.5 1.5]);
hold on; 