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


void image_process(const Mat &_img)
{
    
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"BA_1cam_node");
    ros::NodeHandle nh;
    String folder_path="/image/*.png";
    vector<String> fn;
    glob(folder_path,fn,false);

    vector<Mat> img_msg;
    for(int i=0;i<fn.size();i++)
    {
        string img_path=fn[i];
        img_msg.push_back(imread(img_path));
    }
    for(int i=0;i<img_msg.size();i++)
    {
        imshow("img_msg",img_msg[i]);
    }

    return 0;
}