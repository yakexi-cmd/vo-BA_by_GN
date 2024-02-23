/*
    第三讲-使用最小二乘求解ax^2+bx+c
    使用高斯牛顿法
*/
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;






int iterations=50;
double ar=1.0,br=2.0,cr=1.0;//真实的a,b,c值
double ae=2.0,be=-1.0,ce=5.0;//估计的a,b,c初始值
double cost,lastcost;
int main(int argc,char ** argv)
{
    ros::init(argc,argv,"GN_part3_node");
    ros::NodeHandle nh;

    cv::RNG rng; //opencv随机数产生

    // 由于待估计变量是a,b,c.因此x序列应该是已知的点
    vector<double> x_buf,y_buf;
    for(int i=0;i<100;i++)
    {
        x_buf.push_back(i/100.0);
        //y中是实际观测点的值
        y_buf.push_back(ar*x_buf[i]*x_buf[i]+br*x_buf[i]+cr+rng.gaussian(1.0*1.0));
    }
    
    double e;
    lastcost=0;
    for(int it=0;it<iterations;it++)
    {
        Matrix3d H=Matrix3d::Zero();
        Vector3d b=Vector3d::Zero();
        
        cost=0;
        // 
        for(int i=0;i<100;i++)
        {
            e=y_buf[i]-(ae*x_buf[i]*x_buf[i]+be*x_buf[i]+ce);
            Vector3d J;
            J<< x_buf[i]*x_buf[i],x_buf[i],1;  //J=de/dx=[da/ddx db/dx dc/dx]
            
            H+=J*J.transpose();
            b+=-J.transpose()*e; 
            cost+=e*e;         
        }
        Vector3d dx=H.ldlt().solve(b);//dx中存储的是a,b,c的变化量，即[da,db,dc]
        
        if(isnan(dx[0]))
        {
            break;
        }

        if(it>0 && cost>=lastcost)
        {
            break;
        }
        // if(dx<1e-6)
        //     break;
        ae=ae+dx[0];
        be=be+dx[1];
        ce=ce+dx[2];
        lastcost=cost;
        cout<<"total cost:"<<cost<<",\t\tupdate:"<<dx.transpose()<<"\t\testimated params:"<<ae<<","<<be<<","<<ce<<endl;

    }
    cout<<"estimated abc:"<<ae<<","<<be<<","<<ce<<endl;
    return 0;
}



