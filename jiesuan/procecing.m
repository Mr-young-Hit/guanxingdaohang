format long
clc
clear all
data1 = importdata('data9fu.txt');
data2 = importdata('data9zheng.txt');
total_x1 = sum(data2(:,5)) - sum(data1(:,5));
total_y1 = sum(data2(:,6)) - sum(data1(:,6));
total_z1 = sum(data2(:,7)) - sum(data1(:,7));
data1 = importdata('data5fu.txt');
data2 = importdata('data5zheng.txt');
total_x2 = sum(data2(:,5)) - sum(data1(:,5));
total_y2 = sum(data2(:,6)) - sum(data1(:,6));
total_z2 = sum(data2(:,7)) - sum(data1(:,7));
data1 = importdata('data1fu.txt');
data2 = importdata('data1zheng.txt');
total_x3 = sum(data2(:,5)) - sum(data1(:,5));
total_y3 = sum(data2(:,6)) - sum(data1(:,6));
total_z3 = sum(data2(:,7)) - sum(data1(:,7));
N= [total_x1 total_x2 total_x3;
    total_y1 total_y2 total_y3;
    total_z1 total_z2 total_z3];
N = N*2*pi/3600/360;% °/h转化为rad/h
oma = eye(3,3)*4*pi;% °转化为rad
KG = N\oma
data1 = importdata('datapiao1.txt');
data2 = importdata('datapiao2.txt');
data1_x_average = sum(data1(:,5))/(size(data1,1));
data2_x_average = sum(data2(:,5))/(size(data2,1));
data1_y_average = sum(data1(:,6))/(size(data1,1));
data2_y_average = sum(data2(:,6))/(size(data2,1));
data1_z_average = sum(data1(:,7))/(size(data1,1));
data2_z_average = sum(data2(:,7))/(size(data2,1));
N_average = [data1_x_average + data2_x_average;
           data1_y_average + data2_y_average;
           data1_z_average + data2_z_average];
e = 0.5*KG*N_average/3600/(1e-3)*3600-15.04107*[0;0;sin(45.73265/360*2*pi)] 
% °/h
e = e*2*pi/360/3600 %弧度/s
%到这里是一样的
tic
%老师给的实验数据
KA = [-0.0000923798 -0.00000036241 0.0000000447582;
      -0.000000384431 0.0000962791 -0.0000000989842;
      -0.000000060277 0.000000109072 0.0000944793]; 
KA = KA';
K = blkdiag(KA,KG)
delta_b = [0.000636843;-0.00093891;-0.00316339];  %g
b = [delta_b;e];
bias = repmat(b,1,1800000)*1e-3;
data1 = data1(:,2:7);
data2 = data2(:,2:7);%这里一开始写得是有错得！！！
data1 = [data1(:,1:3) data1(:,4:6)/3600*2*pi/360];
data2 = [data2(:,1:3) data2(:,4:6)/3600*2*pi/360];
data1 = data1';
data2 = data2';
data1_com = K * data1-bias;
data2_com = K * data2-bias;
data1_com = data1_com';
data2_com = data2_com';
toc;


save tuoluopiaoyi11.txt data1_com -ascii -double
save tuoluopiaoyi22.txt data2_com -ascii -double
