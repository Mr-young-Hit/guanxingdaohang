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


int main()
{
    //step1:构造Ka矩阵
    Eigen::Matrix<double,3,3> Ka;
    /*老师的数据
    Ka<<-9.23798e-05,-3.6241e-07,4.47582e-08,
      -3.84431e-07,9.62791e-05,-9.89842e-08,
      -6.0277e-08,1.09072e-07,9.44793e-05;
    */
    //第二组的数据
    Ka<<-9.23507e-05,-9.57571e-08,4.49844e-08,
      -1.288e-07,9.62465e-05,-1.00444e-07,
      -5.93324e-08,1.0963e-07,9.44503e-05;
    //step2:delta矩阵
    Eigen::Matrix<double,3,1> delta;
    //老师的数据
    //delta<<0.000636843,-0.00093891, -0.00316339;
    //第二组的数据
    delta<<0.000557277,-0.00090878, -0.00304563;
    for(int i=1;i<=12;i++)
    {
        string a=to_string(i);
        string b="/home/mryoung/Algorithm-project/navigation/20201219第二组测漂试验/data1"+a+".Txt";
        vector<double> Nak=getAcc1(b);
        Eigen::Matrix<double,3,1> Na;
        for(int i=0;i<3;i++)
        {
            Na(i,0)=Nak[i];
        }
        Eigen::Matrix<double,3,1> fb;
        if(i==1 || i==2)
        {
            fb<<0,0,1;
        }
        if(i==3 || i==4)
        {
            fb<<0,0,-1;
        }
        if(i==5 || i==6)
        {
            fb<<0,1,0;
        }
        if(i==7 || i==8)
        {
            fb<<0,-1,0;
        }
        if(i==9 || i==10)
        {
            fb<<1,0,0;
        }
        if(i==11 || i==12)
        {
            fb<<-1,0,0;
        }
        cout<<"delta_f"<<i<<"="<<Ka*Na-delta-fb;
    }
}