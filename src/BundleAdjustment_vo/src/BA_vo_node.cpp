#include "BA_vo.h"


ros::Publisher pub_path,pub_odometry;
ros::Publisher pub_img;
nav_msgs::Odometry odometry;
nav_msgs::Path path;



// void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
// {
//     cout<<"111111111"<<endl;
//     cv_bridge::CvImageConstPtr ptr;
//     if(img_msg->encoding == "8UC1")
//     {
//         sensor_msgs::Image img;
//         img.header=img_msg->header;
//         img.height=img_msg->height;
//         img.width=img_msg->width;
//         img.is_bigendian=img_msg->is_bigendian;
//         img.step=img_msg->step;
//         img.data=img_msg->data;
//         img.encoding="mono8";
//         ptr=cv_bridge::toCvCopy(img,sensor_msgs::image_encodings::MONO8);
//     }
//     else
//         ptr=cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::MONO8);
    
//     vo_odom.readImage(ptr->image);
//     pub_img.publish(ptr);
    
//     pose_stamped.header.stamp=ros::Time::now();
//     pose_stamped.header.frame_id = "odom";
//     // pose_stamped.pose.position.x=all_pts_buf[0].x;
//     // pose_stamped.pose.position.y=all_pts_buf[0].y;

//     // all_points3d_init为VecVector类型，其中包含了多个特征点的[x,y,z]
//     Vector3d pose_new=all_T*pose_init;
//     pose_stamped.pose.position.x=pose_new.x();
//     pose_stamped.pose.position.y=pose_new.y();
//     pose_stamped.pose.position.z=pose_new.z();
    
//     // cout<<"all_points3d_init[0].x"<<pose_stamped.pose.position.x<<endl;
//     // cout<<"all_points3d_init[0].y"<<pose_stamped.pose.position.y<<endl;
//     // cout<<"all_points3d_init[0].z"<<pose_stamped.pose.position.z<<endl;
//     // cout<<"===================================================================="<<endl;
//     path.poses.push_back(pose_stamped);
//     pub_path.publish(path);

//     pose_init=pose_new;
// }

int main(int argc,char **argv)
{
    ros::init(argc,argv,"BA_1cam_node");
    ros::NodeHandle nh;
    
    //Set publishers
    ros::Publisher pub_cammsg=nh.advertise<nav_msgs::Odometry> ("cam_node",1000);
    pub_img=nh.advertise<sensor_msgs::Image>("img_node",1000);
    pub_path=nh.advertise<nav_msgs::Path>("pub_path",1000);
    pub_odometry=nh.advertise<nav_msgs::Odometry> ("odometry",1000);
    
    path.header.stamp=ros::Time::now();
    path.header.frame_id="odom";

    //首先去发布初始的位置信息[0,0,0],之后的位置是通过计算得到的Rt和初始位置[0,0,0]计算得到的
    double x_init=0.0,y_init=0.0,z_init=0.0;
    pose_init.x()=0.0;
    pose_init.y()=0.0;
    pose_init.z()=0.0;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp=ros::Time::now();
    pose_stamped.header.frame_id = "odom";
    pose_stamped.pose.position.x=pose_init.x();
    pose_stamped.pose.position.y=pose_init.y();
    pose_stamped.pose.position.z=pose_init.z();
    path.poses.push_back(pose_stamped);
    pub_path.publish(path);

    //tf transform
    tf::TransformBroadcaster tf_broadcaster;
    tf::StampedTransform transform;
    transform.frame_id_="world";
    transform.child_frame_id_="odom";

    string fn_kitti="/home/parallels/开源slam/catkin_vo_ws/00";
    visual_odometry* vo_odom = new visual_odometry(fn_kitti);

    int nframe=2;
    while(ros::ok())
    {
        if(nframe >= 4000)
        {
            break;
        }
        std::string fn = fn_kitti+"/image_1/" + vo_odom->AddZeroPadding(nframe, 6) + ".png";
        cv::Mat img_fn = cv::imread(fn,cv::IMREAD_GRAYSCALE);
        if(nframe%4==0)
        {
            vo_odom->image_process(all_img_buf,img_fn);
        }
            
        // 现在已知第一帧的空间位置为points3d_init[x,y,z]
        // 变量是插入的新一帧
        // image_init(img1,img2);
        // else
        vo_odom->newImage_process(all_img_buf,img_fn);

        pose_stamped.header.stamp=ros::Time::now();
        pose_stamped.header.frame_id = "odom";
        // pose_stamped.pose.position.x=all_pts_buf[0].x;
        // pose_stamped.pose.position.y=all_pts_buf[0].y;

        // all_points3d_init为VecVector类型，其中包含了多个特征点的[x,y,z]
        Vector3d pose_new=all_T*pose_init;
        pose_stamped.pose.position.x=pose_new.x();
        pose_stamped.pose.position.y=pose_new.y();
        pose_stamped.pose.position.z=pose_new.z();
        
        // cout<<"all_points3d_init[0].x"<<pose_stamped.pose.position.x<<endl;
        // cout<<"all_points3d_init[0].y"<<pose_stamped.pose.position.y<<endl;
        // cout<<"all_points3d_init[0].z"<<pose_stamped.pose.position.z<<endl;
        // cout<<"===================================================================="<<endl;
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);

        pose_init=pose_new;
        nframe++;


    }
    
    
    // ros::Subscriber sub_img=nh.subscribe("/cam0/image_raw",100,img_callback);  
    
    // ros::spin();
    return 0;
}