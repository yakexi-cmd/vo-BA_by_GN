/*slam实践一：
    1.首先读入多张图片  main函数中
    2.通过特征匹配获得前一帧、后一帧之间的像素坐标(u,v)
    3.通过空间三维点的投影获得投影点的像素坐标？
    4.利用两者的差值，进行最小二乘估计(Bundle Adjustment光束平差法)获得转换矩阵R和平移矩阵t信息
*/
#include <ros/ros.h>
#include <iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
// calib包含了相机校准和三维重建相关内容
// 基础的多视角几何算法，单个立体摄像头的标定，物体姿态估计，立体相似性算法，3D信息重建
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <chrono>//时钟相关

using namespace std;
using namespace cv;

typedef vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;


void find_feature_matches(const Mat &img_1,const Mat &img_2,
                        std::vector<KeyPoint> &keypoints_1,
                        std::vector<KeyPoint> &keypoints_2,
                        std::vector<DMatch> &matches)
{
    Mat descriptors_1,descriptors_2;
    Ptr<FeatureDetector> detector=ORB::create();
    Ptr<DescriptorExtractor> descriptor=ORB::create();
    Ptr<DescriptorMatcher> matcher=DescriptorMatcher::create("BruteForce-Hamming");

    detector->detect(img_1,keypoints_1);
    detector->detect(img_2,keypoints_2);

    descriptor->compute(img_1,keypoints_1,descriptors_1);
    descriptor->compute(img_2,keypoints_2,descriptors_2);

    // 对两帧图像的描述子 使用汉明距离进行匹配
    vector<DMatch> match;
    matcher->match(descriptors_1,descriptors_2,match);

    double min_dist=10000,max_dist=0;

    // 找出最大和最小距离，即最不相似和最相似的距离
    for(int i=0;i<descriptors_1.rows;i++)
    {
        double dist=match[i].distance;
        if(dist<min_dist)
            min_dist=dist;
        if(dist>max_dist)
            max_dist=dist;
    }

    for(int i=0;i<descriptors_1.rows;i++)
    {
        if(match[i].distance<=max(2*min_dist,30.0));
            matches.push_back(match[i]);
    }
}

Point2d pixel2cam(const Point2d &p,const Mat &K)
{
    return Point2d(
        (p.x-K.at<double>(0,2))/K.at<double>(0,0),
        (p.y-K.at<double>(0,2))/K.at<double>(1,1)
    );
}
void bundleAdjustment_gaussNewton(
    const VecVector3d &points_3d,const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &T)
{
    typedef Eigen::Matrix<double,6,1> Vector6d;//代表位姿R和t，矩阵T是位姿的李群
    double cost=0,lastCost=0;//代价，这个应该表示的是误差吧？
    const int iteration=100;
    double fx=K.at<double>(0,0);
    double fy=K.at<double>(1,1);
    double cx=K.at<double>(0,2);
    double cy=K.at<double>(1,2);

    for(int it=0;it<iteration;it++)
    {
        Eigen::Matrix<double,6,6> H=Eigen::Matrix<double,6,6>::Zero();
        Vector6d b=Vector6d::Zero();
        cost=0;

        for(int id=0;id<points_3d.size();id++)
        {
            //通过3d点计算对应的像素坐标(u,v)
            Eigen::Vector3d p_=T*points_3d[it];//对应课本上的P’=TP
            Eigen::Vector2d pose_u(fx*p_[0]/p_[2]+cx,fy*p_[1]/p_[2]+cy);//对应u=fx*x/z+cx,v=fy*y/z+cy
            Eigen::Vector2d e=points_2d[id]-pose_u;//求误差e=特征匹配得到的(u,v)-三维点投影对应的(u,v)

            cost+=e.squaredNorm();//利用平方范数构建最小二乘 对应课本求解argmin||e||^2
            Eigen::Matrix<double,2,6> J;
            double x=p_[0],y=p_[1],z=p_[2];
            double inv_z=1/z;
            double inv_z2=inv_z*inv_z;
            // J=[-fx/z 0 fx*x/z^2 fx*x*y/z^2 -fx-fx*x^2/z^2 fx*y/z
            //         0  -fy/z fy*y/z^2 fy+fy*y^2/z^2 -fy*x*y/z^2 -fy*x/z];

            J << -fx * inv_z,
            0,
            fx * x * inv_z2,
            fx * x * y * inv_z2,
            -fx - fx * x * x * inv_z2,
            fx * y * inv_z,
            0,
            -fy * inv_z,
            fy * y * inv_z2,
            fy + fy * y * y * inv_z2,
            -fy * x * y * inv_z2,
            -fy * x * inv_z;
   
            H += J.transpose()*J;
            b += -J.transpose()*e;

        }
        Vector6d dx;
        dx=H.ldlt().solve(b);
        // 去掉两种无解的情况:（1）dx无解  （2）当前损失值比上一次还大，这种情况下并未进行优化，因此跳出循环，不保留此次结果
        if(isnan(dx[0]))
        {
            break;
        }
        if(it>0 && cost>=lastCost)
        {
            break;
        }

        T=Sophus::SE3d::exp(dx)*T;
        lastCost=cost;

        if(dx.norm()<1e-6)
        {
            break;
        }
    }
    cout<<T.matrix()<<endl;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"BA_node");
    ros::NodeHandle nh("~");
    vector<string> image_msg;

    String img_path="./image/*.png";
    vector<String> fn;
    glob(img_path,fn,false);//glob将img_path中的数据存入fn图像文件集
    // 将image_msg中放入各张图像的路径
    // 注意这里的image_msg类型为std::string
    for(size_t i=0;i<fn.size();i++)
    {
        // Mat img_buf=imread(fn[i]);
        image_msg.push_back(fn[i]);
    }
    // 正常的opencv读取图像流程
    // imread读取，imshow显示
    Mat image_buf;
    vector<Mat> image_vec;
    for(int i=0;i<image_msg.size();i++)
    {
        image_buf=imread(image_msg[i]);
        image_vec.push_back(image_buf);
    }
    for(int i=0;i<image_vec.size();i++)
    {
        imshow("image_buf",image_vec[i]);
        if(i>=2)
        {
            Mat img_1=imread(image_msg[i]);
            Mat img_2=imread(image_msg[i-1]);
            vector<KeyPoint> keypoints_1,keypoints_2;
            vector<DMatch> matches;

            find_feature_matches(img_1,img_2,keypoints_1,keypoints_2,matches);

            // 利用双目相机的深度图，还原匹配的特征点的世界坐标
            Mat d1=imread(image_msg[i-2]);
            Mat K=(Mat_<double>(3,3)<<520.9,0,325.1,0,521.0,249.7,0,0,1);
            vector<Point3f> pts_3d;//存储特征点的世界坐标
            vector<Point2f> pts_2d;//存储特征点通过特征匹配得到的像素坐标
            // 遍历匹配好的特征点
            for(DMatch m:matches)
            {
                // 获取某个点的深度d=d1(y')[x']
                // 获得第y行x列的深度信息
                ushort d=d1.ptr<unsigned short>(int (keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
                if(d == 0)
                    continue;
                float dd=d/5000.0;
                Point2d p1=pixel2cam(keypoints_1[m.queryIdx].pt,K);
                // 通过乘以深度将相机坐标转换到世界坐标系，世界坐标系的深度来自于d1特征点的深度信息
                pts_3d.push_back(Point3f(p1.x*dd,p1.y*dd,dd));
                // 把第二帧的特征点直接插入pts_2d,作为构建重投影误差的像素坐标
                pts_2d.push_back(keypoints_2[m.trainIdx].pt);
            }
            Mat r,t;
            solvePnP(pts_3d,pts_2d,K,Mat(),r,t,false);//3d-2d特征匹配求解r,t
            Mat R;
            cv::Rodrigues(r,R);//把旋转向量变为旋转矩阵的形式

            cout<<"R="<<R<<endl;
            cout<<"t="<<t<<endl;

            VecVector3d pts_3d_eigen;
            VecVector2d pts_2d_eigen;
            for(size_t i=0;i<pts_3d.size();++i)
            {
                pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[i].x,pts_3d[i].y,pts_3d[i].z));
                pts_2d_eigen.push_back(Eigen::Vector2d(pts_2d[i].x,pts_2d[i].y));
            }
            //高斯牛顿求解位姿
            Sophus::SE3d pose_gn;
            bundleAdjustment_gaussNewton(pts_3d_eigen,pts_2d_eigen,K,pose_gn);

            
        }
        // waitKey(0);
    }
    waitKey(0);
    
    
    return 0;
}