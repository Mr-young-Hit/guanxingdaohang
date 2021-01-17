#include<iostream>
#include<fstream>
#include<vector>
#include<stdio.h>
#include<string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>
using namespace std;

vector<double> getAcc(string A)
{
    //ifstream fin ( "/home/mryoung/Algorithm-project/navigation/标定实验/12.Txt" );
    vector<double> result;
    ifstream fin ( A );
        if ( !fin )
        {
            cout<<"请配置好数据位置!"<<endl;
            return result;
        }
        //step1:读取每次实验的数据，求Nak和Yk(k=1,2...12)
        vector<vector<double>> a;
        vector<double> temp;
        while ( !fin.eof() )
        {
            string a1, a2, a3 ,a4;
            for(int i=0;i<8;i++)
            {
                if(i==1)
                    fin>>a1;
                else if(i==2)
                    fin>>a2;
                else if(i==3)
                    fin>>a3;
                else
                {
                    fin>>a4;
                }            
            }
            temp.push_back(atof ( a1.c_str() ));
            temp.push_back(atof ( a2.c_str() ));
            temp.push_back(atof ( a3.c_str() ));
            a.push_back(temp);
            if ( fin.good() == false )
                break;
            temp.clear();
        }
        //cout<<"数据的维度"<<a.size()-1<<'*'<<a[0].size()<<endl;
        double sum1=0,sum2=0,sum3=0;
        
        
        for(int i=0;i<a.size()-1;i++)
        {
            sum1+=a[i][0];
            sum2+=a[i][1];
            sum3+=a[i][2];
        }
        double result1=double(sum1)/120;
        double result2=double(sum2)/120;
        double result3=double(sum3)/120;
        result.push_back(result1);
        result.push_back(result2);
        result.push_back(result3);
        return result;
}

vector<double> getAcc1(string A)
{
    //ifstream fin ( "/home/mryoung/Algorithm-project/navigation/标定实验/12.Txt" );
    vector<double> result;
    double sum1=0.0,sum2=0.0,sum3=0.0;
    int count=0;
    ifstream fin ( A );
        if ( !fin )
        {
            cout<<"请配置好数据位置!"<<endl;
            return result;
        }
        //step1:读取每次实验的数据，求Nak(k=1,2...12)
        while ( !fin.eof() )
        {
            string a1, a2, a3 ,a4;
            for(int i=0;i<8;i++)
            {
                if(i==1)
                    {
                        fin>>a1;
                        sum1+=atof ( a1.c_str() );
                    }
                else if(i==2)
                    {
                        fin>>a2;
                        sum2+=atof ( a2.c_str() );
                    }
                else if(i==3)
                    {
                        fin>>a3;
                        sum3+=atof ( a3.c_str() );
                    }
                else
                {
                    fin>>a4;
                }            
            }
            count++;
            if ( fin.good() == false )
                break;
        }
        double result1=double(sum1)/count*1000;
        double result2=double(sum2)/count*1000;
        double result3=double(sum3)/count*1000;
        result.push_back(result1);
        result.push_back(result2);
        result.push_back(result3);
        return result;
}

Eigen::Vector3d getYk(double psi,double gamma,double theta)
{
    Eigen::Vector3d v_3d;
    v_3d<<0,0,1;//初始位置的比力输入
    const double pi = acos(-1.0);
    psi=double(psi)*pi/180;
    gamma=double(gamma)*pi/180;
    theta=double(theta)*pi/180;
    Eigen::Matrix<double, 3, 3> z,x,y;
    z<<cos(psi),sin(psi),0,
        -sin(psi),cos(psi),0,
        0,0,1;
    x<<1,0,0,
        0,cos(theta),sin(theta),
        0,-sin(theta),cos(theta);
    y<<cos(gamma),0,-sin(gamma),
        0,1,0,
        sin(gamma),0,cos(gamma);
    return y*x*z*v_3d;
}

int main()
{
    //step1:构造Na矩阵
    vector<vector<double>> Na;
    for(int i=1;i<=12;i++)
    {
        string a=to_string(i);
        string b="/home/mryoung/Algorithm-project/navigation/20201219第二组测漂试验/data1"+a+".Txt";
        //string b="/home/mryoung/Algorithm-project/navigation/标定实验/data1"+a+".Txt";
        vector<double> Nak=getAcc1(b);
        Nak.push_back(-1.0);
        Na.push_back(Nak);
    }
    cout<<"Na的维度"<<Na.size()<<'*'<<Na[0].size()<<endl;
    Eigen::Matrix<double, 12, 4> Matrix_Na;
    for(int i=0;i<Na.size();i++)
    {
        for(int j=0;j<Na[0].size();j++)
        {
            Matrix_Na(i,j)=Na[i][j];
        }
    }
    cout<<"H的结果："<<endl;
    cout<<Matrix_Na<<endl;
    //step2:构造Yk矩阵
    double psi1[]={0,180,180,0,0,0,0,180,90,270,90,270};
    double gamma1[]={0,0,180,180,90,270,90,90,270,270,90,90};
    double theta1[]={0,0,0,0,90,90,270,270,0,0,0,0};
    vector<Eigen::Vector3d> Y;
    for(int i=0;i<12;i++)
    {
        double psi=psi1[i],gamma=gamma1[i],theta=theta1[i];
        Eigen::Vector3d Yk=getYk(psi,gamma,theta);
        Y.push_back(Yk);
    }
    //step3:分别求解方程
    Eigen::Matrix<double,12,1> Y1;
    for(int i=0;i<12;i++)
    {
        Y1(i)=Y[i](0);
        //cout<<Y[i](0)<<endl;
    }

    Eigen::Matrix<double,12,1> Y2;
    for(int i=0;i<12;i++)
    {
        Y2(i)=Y[i](1);
        //cout<<Y[i](1)<<endl;
    }

    Eigen::Matrix<double,12,1> Y3;
    for(int i=0;i<12;i++)
    {
        Y3(i)=Y[i](2);
        //cout<<Y[i](2)<<endl;
    }
    Eigen::Matrix<double,4,1> result1=(Matrix_Na.transpose()*Matrix_Na).inverse()*Matrix_Na.transpose()*Y1;
    Eigen::Matrix<double,4,1> result2=(Matrix_Na.transpose()*Matrix_Na).inverse()*Matrix_Na.transpose()*Y2;
    Eigen::Matrix<double,4,1> result3=(Matrix_Na.transpose()*Matrix_Na).inverse()*Matrix_Na.transpose()*Y3;
    //step4:合并得到最终结果
    vector<vector<double>> Ka;
    vector<double> delta;
    vector<double> temp;
    for(int i=0;i<4;i++)
    {
        if(i<3)
            temp.push_back(result1(i,0));
        else
        {
            delta.push_back(result1(i,0));
        }        
    }
    Ka.push_back(temp);
    temp.clear();
    for(int i=0;i<4;i++)
    {
        if(i<3)
            temp.push_back(result2(i,0));
        else
        {
            delta.push_back(result2(i,0));
        }        
    }
    Ka.push_back(temp);
    temp.clear();
    for(int i=0;i<4;i++)
    {
        if(i<3)
            temp.push_back(result3(i,0));
        else
        {
            delta.push_back(result3(i,0));
        }        
    }
    Ka.push_back(temp);
    //step5:观察输出结果
    cout<<"Ka的计算结果为："<<endl;
    for(int i=0;i<Ka.size();i++)
    {
        for(int j=0;j<Ka[0].size();j++)
        {
            cout<<Ka[i][j]<<",";
        }
        cout<<endl;
    }
    cout<<"delta的计算结果为："<<endl;
    for(int i=0;i<delta.size();i++)
    {
        cout<<delta[i]<<" , ";
    }
    cout<<endl;
}