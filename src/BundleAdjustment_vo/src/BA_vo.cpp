#include "BA_vo.h"

bool init_structure=true;
int COL,ROW;
Mat K=(Mat_<double>(3,3)<<461.6,0,363.0,0,460.3,248.1,0,0,1);

std::string visual_odometry::AddZeroPadding(const int value, const unsigned precision) {
  std::ostringstream oss;
  oss << std::setw(precision) << std::setfill('0') << value;
  return oss.str();
}

visual_odometry::visual_odometry(string fn_kitti):fn_kitti_(fn_kitti)
{
    fn_calib_ = fn_kitti_+"/calib.txt";
    fn_poses_ = fn_kitti_+"/times.txt";
    fn_images_ = fn_kitti_+"/image_1/";

    cout<<"fn_images_"<<fn_images_<<endl;


    FetchIntrinsicParams();//read the fx,fy,cx,cy ,which are the intrinsic parameters
    
    //initial
    scale_=1.0;
    
    std::string fn1 = fn_images_ + AddZeroPadding(0, 6) + ".png";
    std::string fn2 = fn_images_ + AddZeroPadding(1, 6) + ".png";
    // // Load the fist two images.
    cv::Mat img1 = cv::imread(fn1,cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread(fn2,cv::IMREAD_GRAYSCALE);
    
    image_init(img1,img2);

}

Point2f pp;
double f_;

void visual_odometry::FetchIntrinsicParams()
{
    std::ifstream fin(fn_calib_);
    std::string line;
    if(fin.is_open())
    {
        std::getline(fin,line);
        std::istringstream iss(line);

        for(int j=0;j<13;j++)
        {
            std::string token;//string to double
            iss >> token;
            if(j==1)
            {
                //fx and fy
                f_=std::stod(token);
            }
            if(j==3)
            {
                //cx
                pp_.x=std::stod(token);
            }
            if(j==7)
            {   //cy
                pp_.y=std::stod(token);
            }
        }
        fin.close();
    }
    else
    {
        f_=0;
        pp_=Point2f(0,0);
    }
}
// inBorder函数进行边界检测
bool visual_odometry::inBorder(const Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x=cvRound(pt.x);//cvRound将点四舍五入变为整数
    int img_y=cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x<COL-BORDER_SIZE && BORDER_SIZE <= img_y && img_y<ROW-BORDER_SIZE;
}

void visual_odometry::reduceVector(vector<Point2f> &v ,vector<uchar> status)
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

void visual_odometry::reduceVector3d(VecVector3d &v ,vector<uchar> status)
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
void visual_odometry::pose_estimation_2d2d(
    const vector<Point2f> &pts1,
    const vector<Point2f> &pts2,
    Mat &R,Mat &t)
{                                      //   0 fy cy                                                   //   0  0  1]
    Mat essential_matrix;
    essential_matrix=findEssentialMat(pts1,pts2,f_,pp_,cv::RANSAC,0.999,1.0,mask_);
    recoverPose (essential_matrix,pts1,pts2,R,t,f_,pp_,mask_);
}

Point2f visual_odometry::pixel2cam(const Point2f &p,const Mat &K)
{
    return Point2f(
        (p.x-K.at<double>(0,2))/K.at<double>(0,0),
        (p.y-K.at<double>(1,2))/K.at<double>(1,1)
    );
}

Sophus::SE3d all_T;
vector<uchar> state;
// BA光束平差法(pnp的一种)利用3d-2d图像帧同时优化位姿和相机坐标，非线性优化得到更准确的R，t值
void visual_odometry::bundleAdjustment_gaussNewton(
    const VecVector3d &points_3d,const VecVector2d &points_2d,
    const Mat &K,
    Sophus::SE3d &T)//T对应位姿[R|t]
{
    state.clear();
    typedef Matrix<double ,6,1> Vector6d;//代表位姿R和t，矩阵T是位姿的李群
    double cost=0,lastcost=0;//代价，这个应该表示的是误差吧？
    // 后面采用cost+=e  是因为在一张图像中不止一个特征点，因此要对一帧图中所有特征点的误差求和
    const int iteration=10;
    double fx=K.at<double>(0,0);
    double fy=K.at<double>(1,1);
    double cx=K.at<double>(0,2);
    double cy=K.at<double>(1,2);

    // cout<<"cx:"<<cx<<"cy"<<cy<<endl;
    // cout<<"fx:"<<fx<<"fy"<<fy<<endl;

    for(int it=0;it<iteration;it++)
    {
        // cout<<"the iteration is:"<<it<<endl;
        Matrix<double,6,6> H=Matrix<double,6,6>::Zero();
        Vector6d b=Vector6d::Zero();
        cost=0;
        for(int id=0;id<points_3d.size();id++)
        {
            Vector3d p_=T*points_3d[id];
            Vector2d pose_u(fx*p_[0]/p_[2]+cx,fy*p_[1]/p_[2]+cy);
            Vector2d e=points_2d[id]-pose_u;
            

            // state[id]=1;
            // cout<<"误差ok"<<endl;
            // cout<<endl<<"特征点"<<id<<"的误差值："<<e<<endl;

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
        // cout<<"it:"<<it<<"cost:"<<cost<<endl;
        // cout<<"it:"<<it<<"lastcost:"<<lastcost<<endl;

        Vector6d dx;
        dx=H.ldlt().solve(b);
        // cout<<"dx:"<<dx<<endl;
        // 去掉两种无解的情况:（1）dx无解  （2）当前损失值比上一次还大，这种情况下并未进行优化，因此跳出循环，不保留此次结果
        if(isnan(dx[0]))
        {
            cout<<"the dx is nan!"<<endl;
            break;
        }
        if(it>0 && cost>=lastcost)
        {
            cout<<"wrong!!!!!!!!!!!!!"<<endl;
            break;
        }

        T=Sophus::SE3d::exp(dx)*T;
        lastcost=cost;

        if(dx.norm()<1e-6)
        {
            cout<<"the dx<1e-6"<<endl;
            break;
        }
    }
    int count_=0;
    for(int id=0;id<points_3d.size();id++)
    {
        Vector3d points_=T*points_3d[id];
        Vector2d pose_u(fx*points_[0]/points_[2]+cx,fy*points_[1]/points_[2]+cy);
        Vector2d e=points_2d[id]-pose_u;
        // cout<<"id:"<<id<<"--e:"<<e.x()<<","<<e.y()<<endl;
        
        if(e.x()>-5 && e.x()<5 && e.y()>-5 && e.y()<5)
        {
            // cout<<"######################"<<endl;
            // cout<<"id:"<<id<<"--e:"<<e.x()<<","<<e.y()<<endl;
            state.push_back(1);

            // state[id]=1;
        }
        else
        {
            count_++;
            state.push_back(0);
        }
            
    }
    // cout<<"不符合的点的数量"<<count_<<endl;
    // cout<<"BA中的3d-size:"<<points_3d.size()<<endl;
    // cout<<"BA中的2d-size:"<<points_2d.size()<<endl;

    //  for(int id=0;id<points_3d.size();id++)
    //     {
    //         Vector3d p_=T*points_3d[id];
    //         Vector2d pose_u(fx*p_[0]/p_[2]+cx,fy*p_[1]/p_[2]+cy);
    //         Vector2d e=points_2d[id]-pose_u;
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
//处理前两帧图像
void visual_odometry::image_init(const Mat &_img1,const Mat &_img2)
{
    vector<uchar> status;
    vector<float> err;
    vector<Point2f> pts1,pts2;

    //1.获得特征点，利用角点检测
    Mat mask;
    mask=Mat(ROW,COL,CV_8UC1,cv::Scalar(255));
    cv::goodFeaturesToTrack(_img1,pts1,500-pts1.size(),0.01,20,mask);
 
    //2.通过LK光流法获取到特征点
    cv::calcOpticalFlowPyrLK(_img1,_img2,pts1,pts2,status,err,cv::Size(21,21),3);

    reduceVector(pts1,status);
    reduceVector(pts2,status);
    reduceVector3d(all_points3d_init,status);

    //4.2d-2d对极约束获得两帧二维图像间的坐标变换信息R(旋转矩阵)和t(平移矩阵)
    Mat R,t;
    pose_estimation_2d2d(pts1,pts2,R,t);
    cout<<"R"<<R<<endl;
    cout<<"t"<<t<<endl;

    //5.三角测量
    Mat T1=(Mat_<double>(3,4)<<
        1,0,0,0,
        0,1,0,0,
        0,0,1,0);
    Mat T2=(Mat_<double>(3,4)<<
        R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0,0),
        R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1,0),
        R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2,0)
    );
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

    // cout<<"###BA之前的pts2-2d点size####:"<<pts2.size()<<endl;
    // cout<<"###BA之前的pts1-2d点size####:"<<pts1.size()<<endl;


    reduceVector(pts1,state);//筛选平面中的2d点 ，误差较大的不保留
    reduceVector(pts2,state);//筛选平面中的2d点 ，误差较大的不保留
    reduceVector3d(pts_3d_eigen,state);//把经过BA筛选后的3d点进行位姿坐标变换，state=0的点不再进行3d点运算
    // cout<<"###BA之后的pts_3d_eigen-3d点size####:"<<pts_3d_eigen.size()<<endl;
    // cout<<"###BA之后的pts2-2d点size####:"<<pts2.size()<<endl;
    // cout<<"###BA之后的pts1-2d点size####:"<<pts1.size()<<endl;

    //初始化，存储最开始的3d点坐标
    for(int id=0;id<pts_3d_eigen.size();id++)
    {
        Vector3d pose=all_T*pts_3d_eigen[id];
        all_points3d_init.push_back(pose);
    }
    //此处all_points3d_init的size应该和pts_3d_eigen的size相同，只不过矩阵的值由于经过了BA优化，有所区别，即x'=Rx+t
    cout<<"process_all_points3d_init:"<<all_points3d_init.size()<<endl;
    // Mat img2_CV;
    // //生成随机颜色
    // cv::RNG rng(12345);
    // //将_img1,_img2拼接在一起，生成图像img2_CV,用于接下来的特征匹配
    // hconcat(_img1, _img2, img2_CV);
    // cv::cvtColor(img2_CV, img2_CV, COLOR_GRAY2BGR);
    // // imshow("img2_CV",img2_CV);
    // for (int i = 0; i < pts2.size(); i++) {
    //     if (status[i]) {
    //         //cv::Scalar用于随机生成图像颜色
    //         cv::Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    //         cv::line(img2_CV, pts1[i], pts2[i]+Point2f(_img1.cols,0), color,1);
    //         cv::circle(img2_CV, pts1[i], 2, color, -1);
    //         cv::circle(img2_CV, pts2[i]+Point2f(_img1.cols,0), 2, color, -1);
    //     }
    // }
    // imshow("img2_CV",img2_CV);
    // waitKey(0);

    Mat img_CV;
    cv::cvtColor(_img2, img_CV, COLOR_GRAY2BGR);
    for (int i = 0; i < pts2.size(); i++) {
        if (status[i]) {
            cv::circle(img_CV, pts2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img_CV, pts1[i], pts2[i], cv::Scalar(0, 250, 0));
        }
    }
    imshow("img_CV",img_CV);

    waitKey(0);

    _img2.copyTo(all_img_buf);
    all_pts_buf=pts2;
    
    
}
//该函数用于每隔10次重新识别特征点，防止随着匹配的进行，特征点数越来越少
void visual_odometry::image_process(const Mat &_img1,const Mat &_img2)
{
    vector<uchar> status;
    vector<float> err;
    vector<Point2f> pts1,pts2;

    //1.获得特征点，利用角点检测
    Mat mask;
    mask=Mat(ROW,COL,CV_8UC1,cv::Scalar(255));
    cv::goodFeaturesToTrack(_img1,pts1,500-pts1.size(),0.01,20,mask);
 
    //2.通过LK光流法获取到特征点
    cv::calcOpticalFlowPyrLK(_img1,_img2,pts1,pts2,status,err,cv::Size(21,21),3);

    reduceVector(pts1,status);
    reduceVector(pts2,status);
    reduceVector3d(all_points3d_init,status);
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

    //6.通过第5步得到了points 3d坐标信息，接下来为BA非线性优化问题
    VecVector3d pts_3d_eigen;
    VecVector2d pts_2d_eigen;
    for(size_t i=0;i<points.size();++i)
    {
        pts_3d_eigen.push_back(Eigen::Vector3d(points[i].x,points[i].y,points[i].z));
        pts_2d_eigen.push_back(Eigen::Vector2d(pts2[i].x,pts2[i].y));
    }

    all_points3d_init.clear();
    cout<<"image_init_pts_3d_eigen:"<<pts_3d_eigen.size()<<endl;
    cout<<"image_init_pts_2d_eigen:"<<pts_2d_eigen.size()<<endl;
    Sophus::SE3d pose_gn;
    bundleAdjustment_gaussNewton(pts_3d_eigen,pts_2d_eigen,K,pose_gn);

    reduceVector(pts1,state);//筛选平面中的2d点 ，误差较大的不保留
    reduceVector(pts2,state);//筛选平面中的2d点 ，误差较大的不保留
    reduceVector3d(pts_3d_eigen,state);//把经过BA筛选后的3d点进行位姿坐标变换，state=0的点不再进行3d点运算
    
    //初始化，存储最开始的3d点坐标
    for(int id=0;id<pts_3d_eigen.size();id++)
    {
        Vector3d pose=all_T*pts_3d_eigen[id];
        all_points3d_init.push_back(pose);

    }
    // Mat img2_CV;
    // //生成随机颜色
    // cv::RNG rng(12345);
    // //将_img1,_img2拼接在一起，生成图像img2_CV,用于接下来的特征匹配
    // hconcat(_img1, _img2, img2_CV);
    // cv::cvtColor(img2_CV, img2_CV, COLOR_GRAY2BGR);
    // // imshow("img2_CV",img2_CV);
    // for (int i = 0; i < pts2.size(); i++) {
    //     if (status[i]) {
    //         //cv::Scalar用于随机生成图像颜色
    //         cv::Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    //         cv::line(img2_CV, pts1[i], pts2[i]+Point2f(_img1.cols,0), color,1);
    //         cv::circle(img2_CV, pts1[i], 2, color, -1);
    //         cv::circle(img2_CV, pts2[i]+Point2f(_img1.cols,0), 2, color, -1);
    //     }
    // }
    // imshow("img2_CV",img2_CV);
    // waitKey(0);

    Mat img_CV;
    cv::cvtColor(_img2, img_CV, COLOR_GRAY2BGR);
    for (int i = 0; i < pts2.size(); i++) {
        if (status[i]) {
            cv::circle(img_CV, pts2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img_CV, pts1[i], pts2[i], cv::Scalar(0, 250, 0));
        }
    }
    imshow("img_CV",img_CV);

    waitKey(0);

    _img2.copyTo(all_img_buf);
    all_pts_buf=pts2;

    cout<<"image_init_all_pts_buf:"<<all_pts_buf.size()<<endl;
    cout<<"image_init_all_pts2:"<<pts2.size()<<endl;
    cout<<"image_init_all_points3d_init:"<<all_points3d_init.size()<<endl;
    
}
//用于处理其他图像，没有特殊情况均使用该函数
void visual_odometry::newImage_process(const Mat &_img1,const Mat &_img2)
{
    // imshow("img1:",_img1);
    // imshow("img2:",_img2);
    // waitKey(0);
    vector<uchar> status;
    vector<float> err;
    vector<Point2f> pts2;
    
    //2.通过LK光流法获取到特征点
    cv::calcOpticalFlowPyrLK(_img1,_img2,all_pts_buf,pts2,status,err,cv::Size(21,21),3);
    
    reduceVector(all_pts_buf,status);
    reduceVector(pts2,status);
    reduceVector3d(all_points3d_init,status);

    cout<<"all_pts_buf:"<<all_pts_buf.size()<<endl;
    cout<<"pts2:"<<pts2.size()<<endl;
    cout<<"all_points3d_init:"<<all_points3d_init.size()<<endl;
    
    // 6.通过第5步得到了points 3d坐标信息，接下来为BA非线性优化问题
    VecVector2d pts_2d_eigen;
    for(size_t i=0;i<pts2.size();++i)
    {
        pts_2d_eigen.push_back(Eigen::Vector2d(pts2[i].x,pts2[i].y));
    }
    // cout<<"pose_buf"<<pose_buf.matrix();
    Sophus::SE3d pose_gn;

    // //保存前一张图中的信息，用于和后一张图进行BA计算

    all_points3d=all_points3d_init;
    all_points3d_init.clear();
    
    bundleAdjustment_gaussNewton(all_points3d,pts_2d_eigen,K,pose_gn);
   
    reduceVector(all_pts_buf,state);//筛选平面中的2d点 ，误差较大的不保留
    reduceVector(pts2,state);//筛选平面中的2d点 ，误差较大的不保留
    reduceVector3d(all_points3d,state);//把经过BA筛选后的3d点进行位姿坐标变换，state=0的点不再进行3d点运算
    
    cout<<"###BA之后的pts_3d_eigen-3d点size####:"<<all_points3d.size()<<endl;
    cout<<"###BA之后的pts2-2d点size####:"<<pts2.size()<<endl;
    cout<<"###BA之后的pts1-2d点size####:"<<all_pts_buf.size()<<endl;

    //初始化，存储最开始的3d点坐标
    for(int id=0;id<all_points3d.size();id++)
    {
        Vector3d pose=all_T*all_points3d[id];
        all_points3d_init.push_back(pose);

    }
    //此处all_points3d_init的size应该和pts_3d_eigen的size相同，只不过矩阵的值由于经过了BA优化，有所区别，即x'=Rx+t
    cout<<"process_all_points3d_init:"<<all_points3d_init.size()<<endl;
    

    // Mat img_CV;
    // //生成随机颜色
    // cv::RNG rng(12345);
    // //将_img1,_img2拼接在一起，生成图像img2_CV,用于接下来的特征匹配
    // hconcat(_img1, _img2, img_CV);
    // cv::cvtColor(img_CV, img_CV, COLOR_GRAY2BGR);
    // // imshow("img2_CV",img2_CV);
    // for (int i = 0; i < pts2.size(); i++) {
    //     if (status[i]) {
    //         //cv::Scalar用于随机生成图像颜色
    //         cv::Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    //         cv::line(img_CV, all_pts_buf[i], pts2[i]+Point2f(_img1.cols,0), color,1);
    //         cv::circle(img_CV, all_pts_buf[i], 2, color, -1);
    //         cv::circle(img_CV, pts2[i]+Point2f(_img1.cols,0), 2, color, -1);
    //     }
    // }
    // imshow("img2_CV",img_CV);
    // waitKey(0);
    Mat img_CV;
    cv::cvtColor(_img2, img_CV, COLOR_GRAY2BGR);
    for (int i = 0; i < pts2.size(); i++) {
        if (status[i]) {
            cv::circle(img_CV, pts2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img_CV, all_pts_buf[i], pts2[i], cv::Scalar(0, 250, 0));
        }
    }
    imshow("img_CV",img_CV);

    waitKey(0);

    _img2.copyTo(all_img_buf);
    all_pts_buf=pts2;
}

//-------------------------------------------------------------------------------------//
//-------------------------------用于测试特征匹配效果-------------------------------------//
// 通过连线查看两张图之间的匹配结果，用于检测特征匹配的准确性
// 结果：部分差异较大的图片的特征匹配结果较差，需要采用一定的特征点筛选方法等
void visual_odometry::feature_detect_lk(const Mat &_img1,const Mat &_img2)
{
    vector<uchar> status;
    vector<float> err;
    vector<Point2f> pts1,pts2;
    //1.获得特征点，利用角点检测
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

    
    Mat img2_CV;
    
    //生成随机颜色
    cv::RNG rng(12345);
    //将_img1,_img2拼接在一起，生成图像img2_CV,用于接下来的特征匹配
    hconcat(_img1, _img2, img2_CV);
    cv::cvtColor(img2_CV, img2_CV, COLOR_GRAY2BGR);
    // imshow("img2_CV",img2_CV);
    for (int i = 0; i < pts2.size(); i++) {
        if (status[i]) {
            //cv::Scalar用于随机生成图像颜色
            cv::Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            cv::line(img2_CV, pts1[i], pts2[i]+Point2f(_img1.cols,0), color,1);
            cv::circle(img2_CV, pts1[i], 2, color, -1);
            cv::circle(img2_CV, pts2[i]+Point2f(_img1.cols,0), 2, color, -1);
        }
    }
    imshow("img2_CV",img2_CV);

    // imshow("all matches",img_match);
    // imshow("good matches",img_goodmatch);
    waitKey(0);
}
Mat cur_img,forw_img;
Point2f cur_pts,forw_pts;
bool init_img;
int i_=0;

//-------------------------------用于测试特征匹配效果-------------------------------------//
//------------------------------------------------------------------------------------//


Sophus::SE3d pose_buf;
vector<Point2f> pts_buf;//存储前一张图对应的像素坐标S

Mat img_buf;
geometry_msgs::PoseStamped pose_stamped;
Vector3d pose_init;


