#include<iostream>
#include<fstream>
#include<vector>
#include<stdio.h>
#include<string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>
using namespace std;

vector<double> getsigma(string A,string B)
{
    //ifstream fin ( "/home/mryoung/Algorithm-project/navigation/标定实验/12.Txt" );
    vector<double> result;
    ifstream fin1 ( A );
        if ( !fin1 )
        {
            cout<<"请配置好数据位置!"<<endl;
            return result;
        }
        //step1:读取数据并求和
        double sum4=0.0,sum5=0.0,sum6=0.0;
        string a4, a5, a6 ,a7;
        while ( !fin1.eof() )
        {
            for(int i=0;i<8;i++)
            {
                if(i==4)
                    {
                        fin1>>a4;
                        sum4+=atof( a4.c_str());
                    }
                else if(i==5)
                    {
                        fin1>>a5;
                        sum5+=atof( a5.c_str());
                    }
                else if(i==6)
                    {
                        fin1>>a6;
                        sum6+=atof( a6.c_str());
                    }
                else
                {
                    fin1>>a7;
                }            
            }
            if ( fin1.good() == false )
                break;
        }
        double sum4_down=0.0,sum5_down=0.0,sum6_down=0.0;
        ifstream fin2 ( B );
        if ( !fin2 )
        {
            cout<<"请配置好数据位置!"<<endl;
            return result;
        }
        //step1:读取数据并求和
        while ( !fin2.eof() )
        {
            for(int i=0;i<8;i++)
            {
                if(i==4)
                    {
                        fin2>>a4;
                        sum4_down+=atof ( a4.c_str());
                    }
                else if(i==5)
                    {
                        fin2>>a5;
                        sum5_down+=atof ( a5.c_str());
                    }
                else if(i==6)
                    {
                        fin2>>a6;
                        sum6_down+=atof ( a6.c_str());
                    }
                else
                {
                    fin2>>a7;
                }            
            }
            if ( fin2.good() == false )
                break;
        }
        vector<double> zhengzhuan;
        zhengzhuan.push_back(double(sum4));
        zhengzhuan.push_back(double(sum5));
        zhengzhuan.push_back(double(sum6));
        cout<<"正转的数据和："<<endl;
        for(int i=0;i<3;i++)
        {
            cout<<zhengzhuan[i]<<" , ";
        }
        cout<<endl;
        vector<double> fanzhuan;
        fanzhuan.push_back(double(sum4_down));
        fanzhuan.push_back(double(sum5_down));
        fanzhuan.push_back(double(sum6_down));
        cout<<"反转的数据和："<<endl;
        for(int i=0;i<3;i++)
        {
            cout<<fanzhuan[i]<<" , ";
        }
        cout<<endl;
        double n_x=double(sum4-sum4_down)/3600;
        double n_y=double(sum5-sum5_down)/3600;
        double n_z=double(sum6-sum6_down)/3600;
        result.push_back(n_x);
        result.push_back(n_y);
        result.push_back(n_z);
        cout<<"作差的数据："<<endl;
        for(int i=0;i<3;i++)
        {
            cout<<result[i]*3600<<" , ";
        }
        cout<<endl;
        return result;
}

int main()
{
    //step1:构造旋转角度差值，单位为度
    Eigen::Matrix<double, 3, 3> Omiga;
    Omiga<<720,0,0,
            0,720,0,
            0,0,720;
    //step2:构造N_sigma_g，x,y,z轴正反转输出求和并求差的结果
    string a="/home/mryoung/Algorithm-project/navigation/20201219第二组测漂试验/data1zheng.Txt";
    string b="/home/mryoung/Algorithm-project/navigation/20201219第二组测漂试验/data1fu.Txt";
    vector<double> position1=getsigma(a,b);//z轴 第六章ppt30
    a="/home/mryoung/Algorithm-project/navigation/20201219第二组测漂试验/data5zheng.Txt";
    b="/home/mryoung/Algorithm-project/navigation/20201219第二组测漂试验/data5fu.Txt";
    vector<double> position2=getsigma(a,b);//y轴
    a="/home/mryoung/Algorithm-project/navigation/20201219第二组测漂试验/data9zheng.Txt";
    b="/home/mryoung/Algorithm-project/navigation/20201219第二组测漂试验/data9fu.Txt";
    vector<double> position3=getsigma(a,b);//x轴
    Eigen::Matrix<double, 3, 3> N_sigma_g;
    for(int i=0;i<position3.size();i++)
    {
        N_sigma_g(0,i)=position3[i];
    }
    for(int i=0;i<position2.size();i++)
    {
        N_sigma_g(1,i)=position2[i];
    }
    for(int i=0;i<position1.size();i++)
    {
        N_sigma_g(2,i)=position1[i];
    }
    //step3:计算输出结果Kg=Omiga*inv(N_sigma_g)
    cout<<"计算的N_sigma_g结果为："<<endl;
    cout<<N_sigma_g.transpose()<<endl;
    Eigen::Matrix<double, 3, 3> Kg=Omiga*N_sigma_g.transpose().inverse();
    cout<<"计算的Kg结果为："<<endl;
    cout<<Kg<<endl;
}