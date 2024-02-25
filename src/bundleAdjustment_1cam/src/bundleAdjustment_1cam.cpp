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

using namespace cv;
using namespace std;

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

/*
1.首先通过goodFeaturesToTrack或者GFTTDetector特征检测获得特征点,
2.通过边界检测，筛选不符合条件的点
3.利用光流法获得下一帧特征点的位置
--------------------------------------------
-----通过上述操作能够得到两帧图像的2d信息---------
-----   接下来要通过三角化得到3d信息   ----------
--------------------------------------------
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
 
    //通过LK光流法获取到特征点
    cv::calcOpticalFlowPyrLK(_img1,_img2,pts1,pts2,status,err,cv::Size(21,21),3);
    
    
    //剔除不相关的点
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