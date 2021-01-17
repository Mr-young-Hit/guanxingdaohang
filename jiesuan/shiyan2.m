clc
clear all
gyodata1 = importdata('data9fu.txt');
gyodata2 = importdata('data9zheng.txt');
gyodata1 = importdata('data1fu.txt');
gyodata2 = importdata('data1zheng.txt');
%初始条件
lat=45.73265;
lon=126.62859;
hei=149.842;
wie=7.2921151467e-5;
Re=6378137;
ts=0.001;
g0=9.7803267714;
omiga = eye(3,3)*4*pi;% °转化为rad
weizhi1_x3 = sum(gyodata2(:,5)) - sum(gyodata1(:,5));
weizhi1_y3 = sum(gyodata2(:,6)) - sum(gyodata1(:,6));
weizhi1_z3 = sum(gyodata2(:,7)) - sum(gyodata1(:,7));
gyodata1 = importdata('data5fu.txt');
gyodata2 = importdata('data5zheng.txt');
weizhi5_x2 = sum(gyodata2(:,5)) - sum(gyodata1(:,5));
weizhi5_y2 = sum(gyodata2(:,6)) - sum(gyodata1(:,6));
weizhi5_z2 = sum(gyodata2(:,7)) - sum(gyodata1(:,7));
gyodata1 = importdata('data9fu.txt');
gyodata2 = importdata('data9zheng.txt');
weizhi9_x1 = sum(gyodata2(:,5)) - sum(gyodata1(:,5));
weizhi9_y1 = sum(gyodata2(:,6)) - sum(gyodata1(:,6));
weizhi9_z1 = sum(gyodata2(:,7)) - sum(gyodata1(:,7));
N_sigma_g= [weizhi9_x1 weizhi5_x2 weizhi1_x3;
    weizhi9_y1 weizhi5_y2 weizhi1_y3;
    weizhi9_z1 weizhi5_z2 weizhi1_z3];
N_sigma_g = N_sigma_g*2*pi/3600/360;% °/h转化为rad/h
Kg = N_sigma_g\omiga
gyodata1 = importdata('datapiao1.txt');
weizhi1_xpingjun = sum(gyodata1(:,5))/(size(gyodata1,1));
weizhi1_ypingjun = sum(gyodata1(:,6))/(size(gyodata1,1));
weizhi1_zpingjun = sum(gyodata1(:,7))/(size(gyodata1,1));
gyodata2 = importdata('datapiao2.txt');
weizhi2_xpingjun = sum(gyodata2(:,5))/(size(gyodata2,1));
weizhi2_ypingjun = sum(gyodata2(:,6))/(size(gyodata2,1));
weizhi2_zpingjun = sum(gyodata2(:,7))/(size(gyodata2,1));
lianggeweizhi_he = [weizhi1_xpingjun + weizhi2_xpingjun;
           weizhi1_ypingjun + weizhi2_ypingjun;
           weizhi1_zpingjun + weizhi2_zpingjun];
delta_g = 0.5*Kg*lianggeweizhi_he/(1e-3)-15.04107*[0;0;sin(45.73265/360*2*pi)] 
% °/h
delta_g = delta_g*2*pi/360/3600 %弧度/s
tic
%老师给的实验数据算出来的结果
Ka = [-0.0000923798 -0.00000036241 0.0000000447582;
      -0.000000384431 0.0000962791 -0.0000000989842;
      -0.000000060277 0.000000109072 0.0000944793]; 
Ka = Ka';
K = blkdiag(Ka,Kg)
delta_b = [0.000636843;-0.00093891;-0.00316339];  %g
delta = [delta_b;delta_g];
zhengtipianzhi = repmat(delta,1,1800000)*1e-3;
gyodata1 = gyodata1(:,2:7);
gyodata2 = gyodata2(:,2:7);
gyodata1 = [gyodata1(:,1:3) gyodata1(:,4:6)/3600*2*pi/360];
gyodata2 = [gyodata2(:,1:3) gyodata2(:,4:6)/3600*2*pi/360];
gyodata1 = gyodata1';
gyodata2 = gyodata2';
weizhi1_processed = K * gyodata1-zhengtipianzhi;
weizhi2_processed = K * gyodata2-zhengtipianzhi;
weizhi1_processed = weizhi1_processed';
weizhi2_processed = weizhi2_processed';
data = weizhi1_processed;  
%data = weizhi2_processed;  如果用位置2就取消注释，注释上一行

g = g0+0.051799*sind(lat)*sind(lat)-0.00000094114*hei;
acc_pingjun = g*mean(data(:,1:3),1)'; 
gyo_pingjun = mean(data(:,4:6),1)';%加速度输出0.001g0
gn = [0;0;-g];
wnie=[0;wie*cosd(lat);wie*sind(lat)];
position=[lat;lon;hei];
T31=acc_pingjun(1)/(g*ts);
T32=acc_pingjun(2)/(g*ts);
T33=acc_pingjun(3)/(g*ts);
T11=-(-gyo_pingjun(3)*acc_pingjun(2)+gyo_pingjun(2)*acc_pingjun(3))/(ts*ts*g*wie*cosd(lat));
T12=-(-gyo_pingjun(1)*acc_pingjun(3)+gyo_pingjun(3)*acc_pingjun(1))/(ts*ts*g*wie*cosd(lat));
T13=-(-gyo_pingjun(2)*acc_pingjun(1)+gyo_pingjun(1)*acc_pingjun(2))/(ts*ts*g*wie*cosd(lat));
T21=-(gyo_pingjun(1)-ts*T31*wie*sind(lat))/(ts*wie*cosd(lat));
T22=-(gyo_pingjun(2)-ts*T32*wie*sind(lat))/(ts*wie*cosd(lat));
T23=-(gyo_pingjun(3)-ts*T33*wie*sind(lat))/(ts*wie*cosd(lat));
T_jielianjuzhen = [T11,T12,T13;T21,T22,T23;T31,T32,T33];
T_jielianjuzhen=normr(T_jielianjuzhen);
zitaijiao = [ asin(T32)*180/pi; 
        atan2(-T31,T33)*180/pi; 
        atan2(-T12,T22)*180/pi];
Qnb = [ 1.0 + T_jielianjuzhen(1,1) + T_jielianjuzhen(2,2) + T_jielianjuzhen(3,3);
        1.0 + T_jielianjuzhen(1,1) - T_jielianjuzhen(2,2) - T_jielianjuzhen(3,3);
        1.0 - T_jielianjuzhen(1,1) + T_jielianjuzhen(2,2) - T_jielianjuzhen(3,3);
        1.0 - T_jielianjuzhen(1,1) - T_jielianjuzhen(2,2) + T_jielianjuzhen(3,3)]; 
fuhaoxiangliang = sign([ 1;
         T32-T23; 
         T13-T31; 
         T21-T12]);
Qnb = fuhaoxiangliang.*sqrt(abs(Qnb))/2; 
Qnb = Qnb/norm(Qnb);
toc