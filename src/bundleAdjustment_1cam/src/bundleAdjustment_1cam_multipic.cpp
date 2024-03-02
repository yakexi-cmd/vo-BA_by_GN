/*单目相机进行位姿估计获得Rt
    区别：和双目及RGB相机的区别在于  单目没有深度信息
    1.首先进行初始化，通过前两帧图片获得第一个三维点的深度信息，从而得到P[x,y,z]三维坐标
    2.利用得到的Point3f和特征匹配得到的Point2f进行bundle Adjustment 最小二乘估计求解R，t
    3.步骤和前面实战1，2一样

    步骤：
    1.首先将5张图片存入img_msg数组中
    2.？？感觉这里是先执行了goodFeaturesToTrack获取到好的特征点，之后
    3.针对两张图进行光流法找到特征点
        cv::calcOpticalFlowPyLK(第一帧图，第二帧图，第一帧特征点，第二帧特征点，status存在点的id值，err，cv::Size(21,21),3)
        其中得到的cur_pts这类是Point3f，而不是KeyPoint
    4.这部分可以添加一个剔除操作，将不合适的关键点剔除
        （1）通过图像边界剔除outlier
        （2）通过对极约束
    5.利用获取到的像素坐标进行三角测量，得到3d点信息
*/
#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <cstdlib>

using namespace cv;
using namespace std;
using namespace Eigen;

typedef vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;
bool init_structure=true;

ros::Publisher pub_path,pub_odometry;
nav_msgs::Odometry odometry;
nav_msgs::Path path;

int COL,ROW;
// inBorder函数进行边界检测
bool inBorder(const Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x=cvRound(pt.x);//cvRound将点四舍五入变为整数
    int img_y=cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x<COL-BORDER_SIZE && BORDER_SIZE <= img_y && img_y<ROW-BORDER_SIZE;
}

void reduceVector(vector<Point2f> &v ,vector<uchar> status)
{
    int j=0;
    for(int i=0;i<int (v.size());i++)
    {
        if(status[i])
        {
            v[j++]=v[i];
        }
    }
    v.resize(j);
}

// 对极约束通过2d-2d图像帧获得Rt信息
void pose_estimation_2d2d(
    const vector<Point2f> &pts1,
    const vector<Point2f> &pts2,
    Mat &R,Mat &t)
{
    Mat K=(Mat_<double>(3,3)<<520.9,0,325.1,0,521.0,249.7,0,0,1);

    Point2f principal_point(325.1,249.7);//对应像素平面（单位为pixel）和成像平面[归一化平面，单位为m]的关系，二者之间是单位的区别
    int focal_length=521;//对应焦距
    // principal_point和focal_length整合可以得到相机的内参K=[fx 0 cx
                                                    //   0 fy cy
                                                    //   0  0  1]
    Mat essential_matrix;
    essential_matrix=findEssentialMat(pts1,pts2,focal_length,principal_point);
    recoverPose (essential_matrix,pts1,pts2,R,t,focal_length,principal_point);
}

Point2f pixel2cam(const Point2f &p,const Mat &K)
{
    return Point2f(
        (p.x-K.at<float>(0,2))/K.at<float>(0,0),
        (p.y-K.at<float>(1,2))/K.at<float>(1,1)
    );
}

Sophus::SE3d all_T;
// BA光束平差法(pnp的一种)利用3d-2d图像帧同时优化位姿和相机坐标，非线性优化得到更准确的R，t值
void bundleAdjustment_gaussNewton(
    const VecVector3d &points_3d,const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &T)//T对应位姿[R|t]
{
    typedef Matrix<double ,6,1> Vector6d;//代表位姿R和t，矩阵T是位姿的李群
    double cost=0,lastcost=0;//代价，这个应该表示的是误差吧？
    // 后面采用cost+=e  是因为在一张图像中不止一个特征点，因此要对一帧图中所有特征点的误差求和
    const int iteration=100;
    float fx=K.at<float>(0,0);
    float fy=K.at<float>(1,1);
    float cx=K.at<float>(0,2);
    float cy=K.at<float>(1,2);

    for(int it=0;it<iteration;it++)
    {
        Matrix<double,6,6> H=Matrix<double,6,6>::Zero();
        Vector6d b=Vector6d::Zero();
        cost=0;
        for(int id=0;id<points_3d.size();id++)
        {
            Vector3d p_=T*points_3d[id];
            Vector2d pose_u(fx*p_[0]/p_[2]+cx,fy*p_[1]/p_[2]+cy);
            Vector2d e=points_2d[id]-pose_u;

            cost+=e.squaredNorm();
            Matrix<double,2,6> J;
            double x=p_[0],y=p_[1],z=p_[2];
            double inv_z=1/z;
            double inv_z2=inv_z*inv_z;

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
        if(it>0 && cost>=lastcost)
        {
            break;
        }

        T=Sophus::SE3d::exp(dx)*T;
        lastcost=cost;

        if(dx.norm()<1e-6)
        {
            break;
        }
    }
    all_T=T;
    cout<<T.matrix()<<endl;
}
/*
1.首先通过goodFeaturesToTrack或者GFTTDetector特征检测获得特征点,
2.利用光流法获得下一帧特征点的位置
3.通过边界检测，筛选不符合条件的点
--------------------------------------------
-----通过上述操作能够得到两帧图像的2d信息---------
-----   接下来要通过三角化得到3d信息   ----------
--------------------------------------------
4.2d-2d特征匹配获得初始的R和t
5.基于2d-2d对极约束得到的R和t信息，使用三角测量获得位姿的深度，即3d信息
*/

//===============================================//
//======特征检测方法1：使用goodFeaturesToTrack======//
//===============================================//
VecVector3d all_points3d_init;//3d点初始化，存储最开始的空间点坐标
VecVector3d all_points3d;
Mat all_img_buf;
vector<Point2f> all_pts_buf;

void image_init(const Mat &_img1,const Mat &_img2)
{
    vector<uchar> status;
    vector<float> err;
    vector<Point2f> pts1,pts2;

    //1.获得特征点，利用角点检测
    vector<Point2f> n_pts1,n_pts2;//n_pts：存储角点的坐标集合，MAX_CNT-pts2.size()：检测角点数
    int MAX_CNT,MIN_DIST;
    Mat mask;
    mask=Mat(ROW,COL,CV_8UC1,cv::Scalar(255));
    cv::goodFeaturesToTrack(_img1,pts1,500-pts1.size(),0.01,20,mask);
 
    //2.通过LK光流法获取到特征点
    cv::calcOpticalFlowPyrLK(_img1,_img2,pts1,pts2,status,err,cv::Size(21,21),3);
    
    
    //3.剔除不相关的点
    for(int i=0;i< int(pts2.size());i++)
    {
        if(status[i] && !inBorder(pts2[i]))
        {
            status[i]=0;
        }
    }
    reduceVector(pts1,status);
    reduceVector(pts2,status);
    // reduceVector(ids,status);

    //4.2d-2d对极约束获得两帧二维图像间的坐标变换信息R(旋转矩阵)和t(平移矩阵)
    Mat R,t;
    pose_estimation_2d2d(pts1,pts2,R,t);
    // cout<<"R"<<R<<endl;
    // cout<<"t"<<t<<endl;

    //5.三角测量
    Mat T1=(Mat_<float>(3,4)<<
        1,0,0,0,
        0,1,0,0,
        0,0,1,0);
    Mat T2=(Mat_<float>(3,4)<<
        R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0,0),
        R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1,0),
        R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2,0)
    );
    Mat K = (Mat_<float>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    //把像素平面转换到相机平面
    
    vector<Point2f> pts1_cam,pts2_cam;
    for(int i=0;i<pts1.size();i++)
    {
        pts1_cam.push_back(pixel2cam(pts1[i],K));
        pts2_cam.push_back(pixel2cam(pts2[i],K));
        // cout<<pts1_cam;
    }
    
    
    Mat pts_4d;
    //通过三角化得到的是齐次坐标[x,y,z,1]
    triangulatePoints(T1,T2,pts1_cam,pts2_cam,pts_4d);
    vector<Point3f> points;
    //转换成非齐次坐标
    for(int i=0;i<pts_4d.cols;i++)
    {
        Mat x=pts_4d.col(i);
        x=x/x.at<float>(3,0);
        Point3f p(
            x.at<float>(0,0),
            x.at<float>(1,0),
            x.at<float>(2,0)
        );
        points.push_back(p);
    }
    // for(int i=0;i<points.size();i++)
    // {
    //     cout<<points[i]<<endl;
    // }

    //6.通过第5步得到了points 3d坐标信息，接下来为BA非线性优化问题
    VecVector3d pts_3d_eigen;
    VecVector2d pts_2d_eigen;
    for(size_t i=0;i<points.size();++i)
    {
        pts_3d_eigen.push_back(Eigen::Vector3d(points[i].x,points[i].y,points[i].z));
        pts_2d_eigen.push_back(Eigen::Vector2d(pts2[i].x,pts2[i].y));
    }
    Sophus::SE3d pose_gn;
    bundleAdjustment_gaussNewton(pts_3d_eigen,pts_2d_eigen,K,pose_gn);

    //初始化，存储最开始的3d点坐标
    for(int id=0;id<pts_3d_eigen.size();id++)
    {
        Vector3d pose=all_T*pts_3d_eigen[id];
        all_points3d_init.push_back(pose);

    }
    _img2.copyTo(all_img_buf);
    all_pts_buf=pts2;

    Mat img2_CV;
    cv::cvtColor(_img2, img2_CV, COLOR_GRAY2BGR);
    for (int i = 0; i < pts2.size(); i++) {
        if (status[i]) {
            cv::circle(img2_CV, pts2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_CV, pts1[i], pts2[i], cv::Scalar(0, 250, 0));
        }
    }
    imshow("img2_CV",img2_CV);
    waitKey(0);
    
}

void newImage_process(const Mat &_img1,const Mat &_img2)
{
    // imshow("img1:",_img1);
    // imshow("img2:",_img2);
    // waitKey(0);
    vector<uchar> status;
    vector<float> err;
    vector<Point2f> pts2;
    //2.通过LK光流法获取到特征点
    
    cv::calcOpticalFlowPyrLK(_img1,_img2,all_pts_buf,pts2,status,err,cv::Size(21,21),3);
    Mat K = (Mat_<float>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    cout<<"all_pts_buf:"<<all_pts_buf.size()<<endl;
    cout<<"pts2:"<<pts2.size()<<endl;
    // 6.通过第5步得到了points 3d坐标信息，接下来为BA非线性优化问题
    VecVector2d pts_2d_eigen;
    for(size_t i=0;i<all_points3d_init.size();++i)
    {
        pts_2d_eigen.push_back(Eigen::Vector2d(pts2[i].x,pts2[i].y));
    }
    // cout<<"pose_buf"<<pose_buf.matrix();
    Sophus::SE3d pose_gn;

    // //保存前一张图中的信息，用于和后一张图进行BA计算

    all_points3d=all_points3d_init;
    all_points3d_init.clear();
    
    bundleAdjustment_gaussNewton(all_points3d,pts_2d_eigen,K,pose_gn);
   
    
    //初始化，存储最开始的3d点坐标
    for(int id=0;id<all_points3d.size();id++)
    {
        Vector3d pose=all_T*all_points3d[id];
        all_points3d_init.push_back(pose);

    }

    Mat img2_CV;
    cv::cvtColor(_img2, img2_CV, COLOR_GRAY2BGR);
    for (int i = 0; i < pts2.size(); i++) {
        if (status[i]) {
            cv::circle(img2_CV, pts2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_CV, all_pts_buf[i], pts2[i], cv::Scalar(0, 250, 0));
        }
    }
    imshow("img2_CV",img2_CV);
    waitKey(0);
    _img2.copyTo(all_img_buf);
    all_pts_buf=pts2;
}
Sophus::SE3d pose_buf;
vector<Point2f> pts_buf;//存储前一张图对应的像素坐标S

Mat img_buf;

int main(int argc,char **argv)
{
    
    ros::init(argc,argv,"BA_1cam_node");
    ros::NodeHandle nh;

    ros::Publisher pub_img=nh.advertise<sensor_msgs::Image>("img_node",1000);
    ros::Publisher pub_cammsg=nh.advertise<nav_msgs::Odometry> ("cam_node",1000);
    
    pub_path=nh.advertise<nav_msgs::Path>("pub_path",1000);
    pub_odometry=nh.advertise<nav_msgs::Odometry> ("odometry",1000);

    srand(time(NULL));
    String folder_path="./image/*.png";
    vector<String> fn;
    glob(folder_path,fn,false);

    vector<Mat> img_msg;
    Mat img1,img2;
    for(int i=0;i<fn.size();i++)
    {
        string img_path=fn[i];
        img_msg.push_back(imread(img_path,0));
    }

    //     // cv_bridge::CvImage cv_bridge_image;
    //     // cv_bridge_image.header.stamp=ros::Time::now();
    //     // cv_bridge_image.header.frame_id="camera_frame";
    //     // cv_bridge_image.encoding=sensor_msgs::image_encodings::MONO8;
    //     // cv_bridge_image.image=all_img_buf;

    path.header.stamp=ros::Time::now();
        path.header.frame_id="odom";

    //首先去发布初始的位置信息[0,0,0],之后的位置是通过计算得到的Rt和初始位置[0,0,0]计算得到的
    double x_init=0.0,y_init=0.0,z_init=0.0;
    Vector3d pose_init(0.0,0.0,0.0);
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp=ros::Time::now();
    pose_stamped.header.frame_id = "odom";
    pose_stamped.pose.position.x=pose_init.x();
    pose_stamped.pose.position.y=pose_init.y();
    pose_stamped.pose.position.z=pose_init.z();
    path.poses.push_back(pose_stamped);
    pub_path.publish(path);

    for(int i=0;i<img_msg.size();i++)
    {
        

        if(i!=img_msg.size()-1)
        {
            img1=img_msg[i];
            img2=img_msg[i+1];
            COL=img2.cols;
            ROW=img2.rows;

            if(init_structure)
            {
                image_init(img1,img2);
                init_structure=false;
            }
            else
            {
                // 现在已知第一帧的空间位置为points3d_init[x,y,z]
                // 变量是插入的新一帧
                // image_init(img1,img2);
                newImage_process(all_img_buf,img2);
            }
        }
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp=ros::Time::now();
        pose_stamped.header.frame_id = "odom";
        // pose_stamped.pose.position.x=all_pts_buf[0].x;
        // pose_stamped.pose.position.y=all_pts_buf[0].y;

        // all_points3d_init为VecVector类型，其中包含了多个特征点的[x,y,z]
        Vector3d pose_new=all_T*pose_init;
        pose_stamped.pose.position.x=pose_new.x()/100;
        pose_stamped.pose.position.y=pose_new.y()/100;
        pose_stamped.pose.position.z=pose_new.z()/100;
        
        cout<<"all_points3d_init[0].x"<<pose_stamped.pose.position.x<<endl;
        cout<<"all_points3d_init[0].y"<<pose_stamped.pose.position.y<<endl;
        cout<<"all_points3d_init[0].z"<<pose_stamped.pose.position.z<<endl;

        path.poses.push_back(pose_stamped);
        pub_path.publish(path);

        pose_init=pose_new;
    }

    return 0;
}