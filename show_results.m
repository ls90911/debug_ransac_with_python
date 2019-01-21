clear
clc
%close all


data = load('result.txt');

% problem1
%data = data(6700:7800,:);

% problem2
%data = data(8400:9000,:);
%data = data(8400:end-1000,:);

% Predicted
x = data(:,2);
y = data(:,3);

% Corrected
cx = data(:,4);
cy = data(:,5);

ransac_nr = data(:,10);

mx = data(:,11);
my = data(:,12);

gates = [3.8,0;0,2.0];

%%

figure(1);
hold off;
plot(x,y);%,'.');
hold on
plot(x(1),y(1),'O');
plot(cx,cy);%,'.');

for i=1:length(gates)
    gx = [0 ,   0  ,  0.1 , 0.1 ,  0]   + gates(1,i);
    gy = [-0.5, 0.5 , 0.5, -0.5 , -0.5] + gates(2,i);
    plot(gx,gy,'r', 'LineWidth', 2);
end

plot(mx,my,'X');
grid on;
axis equal;


