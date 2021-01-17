clc
clear
data = importdata('weizhi1piaoyi.txt');%位置一漂移的数据，并经过预处理
AMM = data(1:10:end,1:3);
GMM=data(:,4:6);
R=6378137;
g0=9.7803267714;
Wie=7.2921151467e-5;
longitude(1)=126.62859;%经度
latitude(1)=45.73265;%维度
height(1)=149.842;
k=10;
K=180000;
%实验2解算的值
Q(1,:)=[0.999993,0.001492,0.0008676,-0.003323];
%Q(1,:)=[cos(0/2*pi/180),0,0,sin(0/2*pi/180)];
%[pitching rolling heading] = quat2angle(q,'XYZ');
%Gy=0.01*pi/3600/180;%陀螺仪,单位时rad/s 加速度计单位转换,预处理过，这里就不用了
Gy=1;
Gy=0.01;
A=g0*1e3;
Vr=zeros(180000,3);%初始化
Ag=zeros(180000,3);
time=zeros(1,1800);
Height=zeros(1,1800);
for N=1:K%加速度度计的输出数量
    q(1,:)=Q(N,:);
    for n=1:k%由陀螺仪更新姿态
        N1=10*(N-1)+n;%索引
        w=Gy*GMM(N1,:);
        wmod=norm(w);
        kc=1-wmod^2/8+wmod^4/384;%四阶
        ks=0.5-wmod^2/48;
        q(n+1,:)=quatmultiply(q(n,:),[kc ks*w]);
    end
    Q(N+1,:)=q(n+1,:);
    [heading,pitching,rolling] = quat2angle(Q(N,:),'ZXY');
    Pitching(N)=pitching*180/pi;
    Rolling(N)=rolling*180/pi;
    Heading(N)=heading*180/pi;
    %下面是导航坐标系的等效四元数
    Wig=[-Vr(N,2)/(R+height(N)),Vr(N,1)/(R+height(N))+Wie*cos(latitude(N)*pi/180),Vr(N,1)/(R+height(N))*tan(latitude(N)*pi/180)+Wie*sin(latitude(N)*pi/180)];%ppt30
    Wig0=Wig*0.01;    %得到转角，相当于乘以时间,转动的角度就是等效矢量 ppt51，30
    Wigmod=quatmod([Wig0,0]);
    Wig1=Wig0/Wigmod;            
    qq=[cos(Wigmod/2),sin(Wigmod/2)*Wig1];      
    Q(N+1,:)=quatmultiply(quatinv(qq),Q(N+1,:));
    [pitching,rolling,heading] = quat2angle(Q(N,:),'XYZ');
    Pitching(N)=pitching*180/pi;
    Rolling(N)=rolling*180/pi;
    Heading(N)=heading*180/pi;
    N2=N;%索引
    fb=AMM(N2,:)*A;
    fg1=quatmultiply(Q(N+1,:),[0,fb]);      
    fg=quatmultiply(fg1,quatinv(Q(N+1,:))); 
    %g = g0+0.051799*sind(lat)*sind(lat)-0.00000094114*hei
    g=g0*(1+0.051799*sin(latitude(N)*pi/180)^2)-0.00000094114*height(N);%加速度计转换到导航坐标系
    Ag(N+1,:)=fg(2:4)-cross([0;Wie*cos(latitude(N)*pi/180);Wie*sin(latitude(N)*pi/180)]+Wig',Vr(N,:))-[0,0,g]; %ppt4 
    Vr(N+1,:)=(Ag(N,:)+Ag(N+1,:))/2*0.01+Vr(N,:);   %平均加速度
    longitude(N+1)=0.01*(Vr(N,1)+Vr(N+1,1))/2/(R+height(N))/cos(latitude(N)*pi/180)/pi*180+longitude(N);
    latitude(N+1)=0.01*(Vr(N,2)+Vr(N+1,2))/2/(R+height(N))/pi*180+latitude(N);
    %height(N+1)=0.01*(Vr(N,3)+Vr(N+1,3))/2+height(N);%平均速度
    height(N+1)=height(N);

end
figure(1)
plot(longitude,latitude);
xlabel('经度/度');
ylabel('纬度/度');
grid on;
figure(2);
for i=1:1800
    time(i)=i;
    Height(i)=height(100*i);
    Pitching1(i)=Pitching(100*i);
    Rolling1(i)=Rolling(100*i);
    Heading1(i)=Heading(100*i);
end
plot(time,Height);
xlabel('时间/秒');
ylabel('高度/米');
figure(3);
plot(time,Pitching1);
xlabel('时间/秒');
ylabel('Pitching/度');
figure(4);
plot(time,Rolling1);
xlabel('时间/秒');
ylabel('Rolling/度');
figure(5);
plot(time,Heading1);
xlabel('时间/秒');
ylabel('Heading/度');

    
    
    
    
        
        


