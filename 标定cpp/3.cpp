#include<iostream>
#include<fstream>
#include<vector>
#include<stdio.h>
#include<string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>
using namespace std;




vector<double> getaver(string A)
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
        int count=0;
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
            count++;
            if ( fin1.good() == false )
                break;
        }
        double n_x=double(sum4)/count;
        double n_y=double(sum5)/count;
        double n_z=double(sum6)/count;
        result.push_back(n_x);
        result.push_back(n_y);
        result.push_back(n_z);
        return result;
}

int main()
{
    //step1:实验2中计算出的Kg
    Eigen::Matrix<double, 3, 3> Kg;
    /*老师给的数据结果
    Kg<<-0.842775,-0.00319318,-0.00407784,
            -0.00465821,0.855458,-0.00289143,
            0.00100248,-0.000564525,0.842754;
    */
   //第二组实验结果
    Kg<<-0.842778,-0.00227324,0.00261341,
            -0.000831941,0.843843,-0.00256351,
            -0.00400072,0.00242357,2.17558;
   
    //step2:构造omega_ie项
    double omega_ie=15.04107;
    const double pi = acos(-1.0);
    Eigen::Matrix<double, 3, 1> L;
    L<<0,0,sin(45.73265*pi/180);
    //step3:正反两个位置取平均值
    string a="/home/mryoung/Algorithm-project/navigation/20201219第二组测漂试验/datapiao1.Txt";
    string b="/home/mryoung/Algorithm-project/navigation/20201219第二组测漂试验/datapiao2.Txt";
    vector<double> aver0=getaver(a);
    vector<double> aver180=getaver(b);
    Eigen::Matrix<double, 3, 1> N_g0;
    for(int i=0;i<aver0.size();i++)
    {
        N_g0(i,0)=aver0[i];
    }
    cout<<"N_g0:"<<endl;
    cout<<N_g0*1000<<endl;
    Eigen::Matrix<double, 3, 1> N_g180;
    for(int i=0;i<aver180.size();i++)
    {
        N_g180(i,0)=aver180[i];
    }
    cout<<"N_g180:"<<endl;
    cout<<N_g180*1000<<endl;

    //step4:计算输出结果epsilon_b
    cout<<"计算的epsilon_b结果为："<<endl;
    Eigen::Matrix<double, 3, 1> epsilon_b=double(1)/2*(Kg*N_g0+Kg*N_g180)*1000-omega_ie*L;
    cout<<epsilon_b<<endl;
}