clc
clear
data = importdata('weizhi1piaoyi.txt');%λ��һƯ�Ƶ����ݣ�������Ԥ����
AMM = data(1:2:end,1:3);
GMM=data(:,4:6);
R=6378137;
g0=9.7803267714;
Wie=7.2921151467e-5;
longitude(1)=126.62859;%����
latitude(1)=45.73265;%ά��
height(1)=149.842;
k=2;
K=900000;
%ʵ��2�����ֵ
Q(1,:)=[0.999993,0.001492,0.0008676,-0.003323];
%Q(1,:)=[cos(0/2*pi/180),0,0,sin(0/2*pi/180)];
%[pitching rolling heading] = quat2angle(q,'XYZ');
%Gy=0.01*pi/3600/180;%������,��λʱrad/s ���ٶȼƵ�λת��,Ԥ�����������Ͳ�����
Gy=1;
Gy=0.01;
A=g0*1e3;
Vr=zeros(900000,3);%��ʼ��
Ag=zeros(900000,3);
time=zeros(1,1800);
Height=zeros(1,1800);
for N=1:K%���ٶȶȼƵ��������
    for n=1:k%�������Ǹ�����̬
        N1=2*(N-1)+n;%����
        if n==1
            w1=Gy*GMM(N1,:);
        end
        if n==2
            w2=Gy*GMM(N1,:);
            w=w1+w2+cross(w1,w2);
            wmod=quatmod([w,0]);
            w1=w/wmod;
            q1=[cos(wmod/2),sin(wmod/2)*w1];
            q2=quatmultiply(Q(N,:),q1);
        end  
    end
    Q(N+1,:)=q2;
    [heading,pitching,rolling] = quat2angle(Q(N,:),'ZXY');
    Pitching(N)=pitching*180/pi;
    Rolling(N)=rolling*180/pi;
    Heading(N)=heading*180/pi;
    %�����ǵ�������ϵ�ĵ�Ч��Ԫ��
    Wig=[-Vr(N,2)/(R+height(N)),Vr(N,1)/(R+height(N))+Wie*cos(latitude(N)*pi/180),Vr(N,1)/(R+height(N))*tan(latitude(N)*pi/180)+Wie*sin(latitude(N)*pi/180)];%ppt30
    Wig0=Wig*0.002;    %�õ�ת�ǣ��൱�ڳ���ʱ��,ת���ĽǶȾ��ǵ�Чʸ�� ppt51��30
    Wigmod=quatmod([Wig0,0]);
    Wig1=Wig0/Wigmod;            
    qq=[cos(Wigmod/2),sin(Wigmod/2)*Wig1];      
    Q(N+1,:)=quatmultiply(quatinv(qq),Q(N+1,:));
    [pitching,rolling,heading] = quat2angle(Q(N,:),'XYZ');
    Pitching(N)=pitching*180/pi;
    Rolling(N)=rolling*180/pi;
    Heading(N)=heading*180/pi;
    N2=N;%����
    fb=AMM(N2,:)*A;
    fg1=quatmultiply(Q(N+1,:),[0,fb]);      
    fg=quatmultiply(fg1,quatinv(Q(N+1,:))); 
    %g = g0+0.051799*sind(lat)*sind(lat)-0.00000094114*hei
    g=g0*(1+0.051799*sin(latitude(N)*pi/180)^2)-0.00000094114*height(N);%���ٶȼ�ת������������ϵ
    Ag(N+1,:)=fg(2:4)-cross([0;Wie*cos(latitude(N)*pi/180);Wie*sin(latitude(N)*pi/180)]+Wig',Vr(N,:))-[0,0,g]; %ppt4 
    Vr(N+1,:)=(Ag(N,:)+Ag(N+1,:))/2*0.002+Vr(N,:);   %ƽ�����ٶ�
    longitude(N+1)=0.002*(Vr(N,1)+Vr(N+1,1))/2/(R+height(N))/cos(latitude(N)*pi/180)/pi*180+longitude(N);
    latitude(N+1)=0.002*(Vr(N,2)+Vr(N+1,2))/2/(R+height(N))/pi*180+latitude(N);
    %height(N+1)=0.01*(Vr(N,3)+Vr(N+1,3))/2+height(N);%ƽ���ٶ�
    height(N+1)=height(N);

end
figure(1)
plot(longitude,latitude);
xlabel('����/��');
ylabel('γ��/��');
grid on;
figure(2);
for i=1:1800
    time(i)=i;
    Vx(i)=Vr(500*i,1);
    Vy(i)=Vr(500*i,2);
    Pitching1(i)=Pitching(500*i);
    Rolling1(i)=Rolling(500*i);
    Heading1(i)=Heading(500*i);
end
plot(time,Vx);
xlabel('ʱ��/��');
ylabel('x�ٶ� m/s');
figure(3);
plot(time,Vy);
xlabel('ʱ��/��');
ylabel('y�ٶ� m/s');
figure(4);
plot(time,Pitching1);
xlabel('ʱ��/��');
ylabel('Pitching/��');
figure(5);
plot(time,Rolling1);
xlabel('ʱ��/��');
ylabel('Rolling/��');
figure(6);
plot(time,Heading1);
xlabel('ʱ��/��');
ylabel('Heading/��');

    
    
    
    
        
        


