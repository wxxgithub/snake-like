A=0.5;
B=2*pi/9;
%function draw(A,B) 
%%ģ����
n = 8; 

delta = linspace(0,0,n); %[0,0...,0] n��0
sigma = linspace(0,0,n); %[0,0...,0]
nextdot_y = zeros(1,n+1); %����һ��д
nextdot_x = zeros(1,n+1); 
%%ģ�鳤
l = 0.4; 

%%��ʼ��
%delta(1) = pi/4;%delta(1) = (A/2)/sin(B/2); 
sigma(1) = pi/4;%sigma(1) = (A/2)/sin(B/2); 

%%���ؽڼ�н�
for i = 2:n 
     delta(i) = A*sin((i-1)*B); 
end 

%%���ؽ���ˮƽ�н�
for j = 2:n 
     sigma(j) = sigma(j-1)-delta(j); 
end 

%%���ؽ����ӵ������
nextdot_y(1) = 0; 
nextdot_x(1) = 0; 
for k = 2:n+1 
     nextdot_y(k) = l*sin(sigma(k-1))+nextdot_y(k-1); 
     nextdot_x(k) = l*cos(sigma(k-1))+nextdot_x(k-1); 
end 
plot(nextdot_x,nextdot_y); 
axis([-1.5 7 -1.5 1.5]);
hold on; 