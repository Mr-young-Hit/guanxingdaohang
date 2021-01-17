clear
clc
data = importdata('tuoluopiaoyi11.txt');
tic
lat=45.73265;
lon=126.62859;
hei=149.842;
wie=7.2921151467e-5;
Re=6378137;
ts=0.001;
g0=9.7803267714;
g = g0+0.051799*sind(lat)*sind(lat)-0.00000094114*hei;
v = g*mean(data(:,1:3),1)'; theta = mean(data(:,4:6),1)';%加速度输出0.001g0
gn = [0;0;-g];
wnie=[0;wie*cosd(lat);wie*sind(lat)];
pos=[lat;lon;hei];
T31=v(1)/(g*ts);
T32=v(2)/(g*ts);
T33=v(3)/(g*ts);
T21=(theta(1)-ts*T31*wie*sind(lat))/(ts*wie*cosd(lat));
T22=(theta(2)-ts*T32*wie*sind(lat))/(ts*wie*cosd(lat));
T23=(theta(3)-ts*T33*wie*sind(lat))/(ts*wie*cosd(lat));
T11=(-theta(3)*v(2)+theta(2)*v(3))/(ts*ts*g*wie*cosd(lat));
T12=(-theta(1)*v(3)+theta(3)*v(1))/(ts*ts*g*wie*cosd(lat));
T13=(-theta(2)*v(1)+theta(1)*v(2))/(ts*ts*g*wie*cosd(lat));
T = [T11,T12,T13;T21,T22,T23;T31,T32,T33];
qnb = [ 1.0 + T11 + T22 + T33;
        1.0 + T11 - T22 - T33;
        1.0 - T11 + T22 - T33;
        1.0 - T11 - T22 + T33]; 
s = sign([ 1;
         T32-T23; 
         T13-T31; 
         T21-T12]);
qnb = s.*sqrt(abs(qnb))/2; 
qnb = qnb/norm(qnb);
att = [ asin(T32)*180/pi; 
        atan2(-T31,T33)*180/pi; 
        atan2(-T12,T22)*180/pi];
toc
