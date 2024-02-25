/*单目相机进行位姿估计获得Rt
    区别：和双目及RGB相机的区别在于  单目没有深度信息
    1.首先进行初始化，通过前两帧图片获得第一个三维点的深度信息，从而得到P[x,y,z]三维坐标
    2.利用得到的Point3d和特征匹配得到的Point2d进行bundle Adjustment 最小二乘估计求解R，t
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


using namespace cv;
using namespace std;
using namespace Eigen;

typedef vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;


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

    Point2d principal_point(325.1,249.7);//对应像素平面（单位为pixel）和成像平面[归一化平面，单位为m]的关系，二者之间是单位的区别
    int focal_length=521;//对应焦距
    // principal_point和focal_length整合可以得到相机的内参K=[fx 0 cx
                                                    //   0 fy cy
                                                    //   0  0  1]
    Mat essential_matrix;
    essential_matrix=findEssentialMat(pts1,pts2,focal_length,principal_point);
    recoverPose (essential_matrix,pts1,pts2,R,t,focal_length,principal_point);
}

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
    double fx=K.at<double>(0,0);
    double fy=K.at<double>(1,1);
    double cx=K.at<double>(0,2);
    double cy=K.at<double>(1,2);

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
void image_process(const Mat &_img1,const Mat &_img2)
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
    cout<<"R"<<R<<endl;
    cout<<"t"<<t<<endl;

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
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    Mat pts_4d;
    //通过三角化得到的是齐次坐标[x,y,z,1]
    triangulatePoints(T1,T2,pts1,pts2,pts_4d);
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
    for(int i=0;i<points.size();i++)
    {
        cout<<points[i]<<endl;
    }

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
    
    
    Mat img2_CV;
    cv::cvtColor(_img2, img2_CV, COLOR_GRAY2BGR);
    for (int i = 0; i < pts2.size(); i++) {
        if (status[i]) {
            cv::circle(img2_CV, pts2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_CV, pts1[i], pts2[i], cv::Scalar(0, 250, 0));
        }
    }
    imshow("img2_CV",img2_CV);


}
//========================================//
//======特征检测方法2：使用GFTTDetector======//
//========================================//
// void image_process(const Mat &_img1,const Mat &_img2)
// {
//     //1.获得特征点，利用角点检测
//     vector<KeyPoint> kp1;
//     Ptr<GFTTDetector> detector = GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
//     detector->detect(_img1, kp1);

//     vector<uchar> status;
//     vector<float> err;
//     vector<Point2f> pts1,pts2;
//     for(auto &kp:kp1)
//         pts1.push_back(kp.pt);

//     //通过LK光流法获取到特征点
//     cv::calcOpticalFlowPyrLK(_img1,_img2,pts1,pts2,status,err,cv::Size(21,21),3);
    
//     Mat img2_CV;
//     cv::cvtColor(_img2, img2_CV, COLOR_GRAY2BGR);
//     for (int i = 0; i < pts2.size(); i++) {
//         if (status[i]) {
//             cv::circle(img2_CV, pts2[i], 2, cv::Scalar(0, 250, 0), 2);
//             cv::line(img2_CV, pts1[i], pts2[i], cv::Scalar(0, 250, 0));
//         }
//     }

//     imshow("img2_CV",img2_CV);
// }

int main(int argc,char **argv)
{
    ros::init(argc,argv,"BA_1cam_node");
    ros::NodeHandle nh;
    Mat img1=imread("./image/000001.png",0);
    Mat img2=imread("./image/000002.png",0);
    COL=img2.cols;
    ROW=img2.rows;
    // 正常的opencv读取图像流程
    // imread读取，imshow显示

    // cv::goodFeaturesToTrack(_img1,pts1,MAX_CNT-pts1.size(),0.01,MIN_DIST,mask);

    image_process(img1,img2);
    waitKey(0);

    return 0;
}

// int main(int argc,char **argv)
// {
//     ros::init(argc,argv,"BA_1cam_node");
//     ros::NodeHandle nh;
//     String folder_path="./image/*.png";
//     vector<String> fn;
//     glob(folder_path,fn,false);

//     // vector<Mat> img_msg;
//     // for(int i=0;i<fn.size();i++)
//     // {
//     //     string img_path=fn[i];
//     //     img_msg.push_back(imread(img_path));
//     // }
//     // for(int i=0;i<img_msg.size();i++)
//     // {
//     //     imshow("img_msg",img_msg[i]);
//     // }
//     vector<string> image_msg;
//     for(size_t i=0;i<fn.size();i++)
//     {
//         // Mat img_buf=imread(fn[i]);
//         image_msg.push_back(fn[i]);
//     }
//     // 正常的opencv读取图像流程
//     // imread读取，imshow显示
//     Mat image_buf;
//     vector<Mat> image_vec;
//     for(int i=0;i<image_msg.size();i++)
//     {
//         image_buf=imread(image_msg[i]);
//         image_vec.push_back(image_buf);
//     }
//     for(int i=0;i<image_vec.size();i++)
//     {
//         imshow("img_msg",image_vec[i]);
//     }

//     return 0;
// }