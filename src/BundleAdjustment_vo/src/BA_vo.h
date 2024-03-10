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
#include <tf/transform_broadcaster.h>

using namespace cv;
using namespace std;
using namespace Eigen;


typedef vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;
extern bool init_structure;

extern int COL,ROW;
extern VecVector3d all_points3d_init;//3d点初始化，存储最开始的空间点坐标
extern VecVector3d all_points3d;
extern Mat all_img_buf;
extern vector<Point2f> all_pts_buf;

extern Mat cur_img,forw_img;
extern Point2f cur_pts,forw_pts;
extern bool init_img;
extern int i_;

extern Sophus::SE3d pose_buf;
extern vector<Point2f> pts_buf;//存储前一张图对应的像素坐标S

extern Mat img_buf;
extern geometry_msgs::PoseStamped pose_stamped;
extern Vector3d pose_init;

extern Sophus::SE3d all_T;
extern vector<uchar> state;

class visual_odometry
{
public:
    visual_odometry(string fn_kitti);
    void FetchIntrinsicParams();
    std::string AddZeroPadding(const int value, const unsigned precision);
    bool inBorder(const Point2f &pt);
    void reduceVector(vector<Point2f> &v ,vector<uchar> status);
    void reduceVector3d(VecVector3d &v ,vector<uchar> status);
    // 对极约束通过2d-2d图像帧获得Rt信息
    void pose_estimation_2d2d(const vector<Point2f> &pts1,const vector<Point2f> &pts2,Mat &R,Mat &t);
    Point2f pixel2cam(const Point2f &p,const Mat &K);
    // BA光束平差法(pnp的一种)利用3d-2d图像帧同时优化位姿和相机坐标，非线性优化得到更准确的R，t值
    void bundleAdjustment_gaussNewton(const VecVector3d &points_3d,const VecVector2d &points_2d,const Mat &K,Sophus::SE3d &T);//T对应位姿[R|t]
    //处理前两帧图像
    void image_init(const Mat &_img1,const Mat &_img2);
    //该函数用于每隔10次重新识别特征点，防止随着匹配的进行，特征点数越来越少
    void image_process(const Mat &_img1,const Mat &_img2);
    //用于处理其他图像，没有特殊情况均使用该函数
    void newImage_process(const Mat &_img1,const Mat &_img2);
    //用于测试特征匹配效果
    void feature_detect_lk(const Mat &_img1,const Mat &_img2);
    //读取各帧图片
    void readImage(const Mat &_img);
private:
    Point2f pp_;
    double f_;
    double scale_;
    string fn_kitti_,fn_calib_,fn_poses_,fn_images_;
    cv::Mat img_traj_;
    cv::Mat mask_;

};