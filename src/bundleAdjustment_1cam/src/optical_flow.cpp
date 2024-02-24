/*
光流法跟踪特征点
    最终能够得到从第一帧到第二帧的运行速度和像素坐标

光流法是直接法的重要内容
之后进行直接法似乎？可以进行位姿估计

*/
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>

using namespace std;
using namespace cv;

class opticalFlowTracker {
public:
    opticalFlowTracker(
        const Mat &img1_,
        const Mat &img2_,
        const vector<KeyPoint> &kp1_,
        vector<KeyPoint> &kp2_,
        vector<bool> &success_,
        bool inverse_ = true, bool has_initial_ = false) :
        img1(img1_), img2(img2_), kp1(kp1_), kp2(kp2_), success(success_), inverse(inverse_),
        has_initial(has_initial_) {}

    void calculateOpticalFlow(const Range &range);

private:
    const Mat &img1;
    const Mat &img2;
    const vector<KeyPoint> &kp1;
    vector<KeyPoint> &kp2;
    vector<bool> &success;
    bool inverse = true;
    bool has_initial = false;
};

void OpticalFlowSingleLevel(
    const Mat &img1,
    const Mat &img2,
    const vector<KeyPoint> &kp1,
    vector<KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse = false,
    bool has_initial_guess = false
);


inline float Getpixelvalue(const Mat &img,float x,float y)
{
    if(x<0) x=0;
    if(y<0) y=0;
    if(x>=img.cols-1) x=img.cols-2;
    if(y>=img.rows-1) y=img.rows-2;

    float xx=x-floor(x);
    float yy=y-floor(y);
    int x_a1=std::min(img.cols-1,int(x)+1);
    int y_a1=std::min(img.rows-1,int(y)+1);

    return (1-xx)*(1-yy)*img.at<uchar>(y,x)
    +xx*(1-yy)*img.at<uchar>(y,x_a1)
    +(1-xx)*yy*img.at<uchar>(y_a1,x)
    +xx*yy*img.at<uchar>(y_a1,x_a1);

}




int main(int argc,char ** argv)
{
    ros::init(argc,argv,"optical_node");
    ros::NodeHandle nh;

    Mat img1=imread("./image/000001.png",0);
    Mat img2=imread("./image/000002.png",0);

    // 找到图片中的特征点及其对应的描述子？？
    vector<KeyPoint> kp1;
    Ptr<GFTTDetector> detector=GFTTDetector::create(500,0.01,20);
    detector->detect(img1,kp1);

    vector<KeyPoint> kp2_single;
    vector<bool> success_single;
    OpticalFlowSingleLevel(img1, img2, kp1, kp2_single, success_single);

    // vector<bool> success_pts;//表示第一帧和第二帧是否匹配成功
    // kp2_single.resize(kp1.size());
    // success_pts.resize(kp1.size());
    // bool inverse=false;
    // bool has_initial=true;
    // opticalFlowTracker tracker(img1,img2,kp1,kp2_single,success_pts,inverse,has_initial);
    // // opencv的并行计算功能
    // parallel_for_(Range(0,kp1.size()),
    //             std::bind(&opticalFlowTracker::calculateOpticalFlow,&tracker,placeholders::_1));
    
    // 使用opencv中的光流法函数
    vector<Point2f> pt1,pt2;
    for(auto &kp:kp1) 
        pt1.push_back(kp.pt);
    vector<uchar> status;
    vector<float> error;
    cv::calcOpticalFlowPyrLK(img1,img2,pt1,pt2,status,error);

    // 绘制相关图像
    Mat img2_single;
    cv::cvtColor(img2,img2_single,COLOR_GRAY2BGR);
    for(int i=0;i<kp2_single.size();i++)
    {
        if(success_single[i])
        {
            cv::circle(img2_single,kp2_single[i].pt,2,cv::Scalar(0,250,0),2);
            cv::line(img2_single,kp1[i].pt,kp2_single[i].pt,cv::Scalar(0,250,0));

        }
    }

    Mat img2_CV;
    cv::cvtColor(img2, img2_CV, COLOR_GRAY2BGR);
    for (int i = 0; i < pt2.size(); i++) {
        if (status[i]) {
            cv::circle(img2_CV, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
        }
    }
    cv::imshow("tracked single level", img2_single);
    cv::imshow("tracked by opencv", img2_CV);
    cv::waitKey(0);

    return 0;
}



void OpticalFlowSingleLevel(
    const Mat &img1,
    const Mat &img2,
    const vector<KeyPoint> &kp1,
    vector<KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse, bool has_initial) {
    kp2.resize(kp1.size());
    success.resize(kp1.size());
    opticalFlowTracker tracker(img1, img2, kp1, kp2, success, inverse, has_initial);
    parallel_for_(Range(0, kp1.size()),
                  std::bind(&opticalFlowTracker::calculateOpticalFlow, &tracker, placeholders::_1));
}

void opticalFlowTracker::calculateOpticalFlow(const Range &range)
{
    int half_path_size=4;
    int iteration=10;
    for(size_t i=range.start;i<range.end;i++)
    {
        auto kp=kp1[i];
        double dx=0,dy=0;
        if(has_initial)
        {
            dx=kp2[i].pt.x-kp.pt.x;//对应x方向的像素偏差dx
            dy=kp2[i].pt.y-kp.pt.y;//对应y方向的像素偏差dy
        }//所以update对应的是x,y方向的速度[u,v]

        double cost=0,lastcost=0;
        bool succ=true;
        //定义高斯牛顿法求解所需要的H,b,J等数据，首先初始化为0
        Eigen::Matrix2d H=Eigen::Matrix2d::Zero();
        Eigen::Vector2d b=Eigen::Vector2d::Zero();
        Eigen::Vector2d J;
        for(int it=0;it<iteration;it++)
        {
            if(inverse == false)
            {
                H=Eigen::Matrix2d::Zero();
                b=Eigen::Vector2d::Zero();
            }
            else{
                b=Eigen::Vector2d::Zero();
            }
            cost=0;
            

            for(int x=-4;x<4;x++)
            {
                for(int y=-4;y<4;y++)
                {
                    // 计算e=I(x,y)-I(x+1,y+1)
                    double error=Getpixelvalue(img1,kp.pt.x+x,kp.pt.y+y)-
                                Getpixelvalue(img2,kp.pt.x+x+dx,kp.pt.y+y+dy);
                    // -1表示梯度是减小的
                    // J=[0.5*(I(x+1，y+1)-I(x-1,y))   0.5*(I(x，y+1)-I(x,y-1)) ]
                    if(inverse == false){
                        J = -1.0 * Eigen::Vector2d(
                            0.5 * (Getpixelvalue(img2, kp.pt.x + dx + x + 1, kp.pt.y + dy + y) -
                                   Getpixelvalue(img2, kp.pt.x + dx + x - 1, kp.pt.y + dy + y)),
                            0.5 * (Getpixelvalue(img2, kp.pt.x + dx + x, kp.pt.y + dy + y - 1))
                        );
                    }
                    
                    else if(it==0)
                    {
                        J = -1.0 * Eigen::Vector2d(
                            0.5 * (Getpixelvalue(img1, kp.pt.x + x + 1, kp.pt.y + y) -
                                   Getpixelvalue(img1, kp.pt.x + x - 1, kp.pt.y + y)),
                            0.5 * (Getpixelvalue(img1, kp.pt.x + x, kp.pt.y + y + 1) -
                                   Getpixelvalue(img1, kp.pt.x + x, kp.pt.y + y - 1))
                        );
                    }
                    b+=-error*J;
                    cost+=error*error;
                    if(inverse ==false || it ==0)
                    {
                        H+=J*J.transpose();
                    }

                }
            }
            Eigen::Vector2d update=H.ldlt().solve(b);
            if(isnan(update[0]))
            {
                succ=false;
                break;
            }
            if(it > 0 &&cost>lastcost)
            {
                break;
            }
            dx+=update[0];
            dy+=update[1];
            lastcost=cost;
            succ=true;

            if(update.norm()<1e-2)
            {
                break;
            }
        }
        success[i]=succ;
        kp2[i].pt=kp.pt+Point2f(dx,dy);
    }
}


// //视觉slam里面的方法，前面是我自己的代码
// void opticalFlowTracker::calculateOpticalFlow(const Range &range) {
//     // parameters
//     int half_patch_size = 4;
//     int iterations = 10;
//     for (size_t i = range.start; i < range.end; i++) {
//         auto kp = kp1[i];
//         double dx = 0, dy = 0; // dx,dy need to be estimated
//         if (has_initial) {
//             dx = kp2[i].pt.x - kp.pt.x;
//             dy = kp2[i].pt.y - kp.pt.y;
//         }

//         double cost = 0, lastCost = 0;
//         bool succ = true; // indicate if this point succeeded

//         // Gauss-Newton iterations
//         Eigen::Matrix2d H = Eigen::Matrix2d::Zero();    // hessian
//         Eigen::Vector2d b = Eigen::Vector2d::Zero();    // bias
//         Eigen::Vector2d J;  // jacobian
//         for (int iter = 0; iter < iterations; iter++) {
//             if (inverse == false) {
//                 H = Eigen::Matrix2d::Zero();
//                 b = Eigen::Vector2d::Zero();
//             } else {
//                 // only reset b
//                 b = Eigen::Vector2d::Zero();
//             }

//             cost = 0;

//             // compute cost and jacobian
//             for (int x = -half_patch_size; x < half_patch_size; x++)
//                 for (int y = -half_patch_size; y < half_patch_size; y++) {
//                     double error = Getpixelvalue(img1, kp.pt.x + x, kp.pt.y + y) -
//                                    Getpixelvalue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy);;  // Jacobian
//                     if (inverse == false) {
//                         J = -1.0 * Eigen::Vector2d(
//                             0.5 * (Getpixelvalue(img2, kp.pt.x + dx + x + 1, kp.pt.y + dy + y) -
//                                    Getpixelvalue(img2, kp.pt.x + dx + x - 1, kp.pt.y + dy + y)),
//                             0.5 * (Getpixelvalue(img2, kp.pt.x + dx + x, kp.pt.y + dy + y - 1))
//                         );
//                     } else if (iter == 0) {
//                         // in inverse mode, J keeps same for all iterations
//                         // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
//                         J = -1.0 * Eigen::Vector2d(
//                             0.5 * (Getpixelvalue(img1, kp.pt.x + x + 1, kp.pt.y + y) -
//                                    Getpixelvalue(img1, kp.pt.x + x - 1, kp.pt.y + y)),
//                             0.5 * (Getpixelvalue(img1, kp.pt.x + x, kp.pt.y + y + 1) -
//                                    Getpixelvalue(img1, kp.pt.x + x, kp.pt.y + y - 1))
//                         );
//                     }
//                     // compute H, b and set cost;
//                     b += -error * J;
//                     cost += error * error;
//                     if (inverse == false || iter == 0) {
//                         // also update H
//                         H += J * J.transpose();
//                     }
//                 }

//             // compute update
//             Eigen::Vector2d update = H.ldlt().solve(b);

//             if (std::isnan(update[0])) {
//                 // sometimes occurred when we have a black or white patch and H is irreversible
//                 cout << "update is nan" << endl;
//                 succ = false;
//                 break;
//             }

//             if (iter > 0 && cost > lastCost) {
//                 break;
//             }

//             // update dx, dy
//             dx += update[0];
//             dy += update[1];
//             lastCost = cost;
//             succ = true;

//             if (update.norm() < 1e-2) {
//                 // converge
//                 break;
//             }
//         }

//         success[i] = succ;

//         // set kp2
//         kp2[i].pt = kp.pt + Point2f(dx, dy);
//     }
// }


