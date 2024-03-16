#include "my_and_demo.h"


ros::Publisher pub_path,pub_odometry;
ros::Publisher pub_img;
nav_msgs::Odometry odometry;
nav_msgs::Path path;

// int main(int argc,char **argv)
// {
//     ros::init(argc,argv,"BA_1cam_node");
//     ros::NodeHandle nh;
    
//     //Set publishers
//     ros::Publisher pub_cammsg=nh.advertise<nav_msgs::Odometry> ("cam_node",1000);
//     pub_img=nh.advertise<sensor_msgs::Image>("img_node",1000);
//     pub_path=nh.advertise<nav_msgs::Path>("pub_path",1000);
//     pub_odometry=nh.advertise<nav_msgs::Odometry> ("odometry",1000);
    
//     path.header.stamp=ros::Time::now();
//     path.header.frame_id="map";

//     //首先去发布初始的位置信息[0,0,0],之后的位置是通过计算得到的Rt和初始位置[0,0,0]计算得到的
//     double x_init=0.0,y_init=0.0,z_init=0.0;
//     pose_init.x()=0.0;
//     pose_init.y()=0.0;
//     pose_init.z()=0.0;
//     geometry_msgs::PoseStamped pose_stamped;
//     pose_stamped.header.stamp=ros::Time::now();
//     pose_stamped.header.frame_id = "map";
//     pose_stamped.pose.position.x=pose_init.x();
//     pose_stamped.pose.position.y=pose_init.y();
//     pose_stamped.pose.position.z=pose_init.z();
//     path.poses.push_back(pose_stamped);
//     pub_path.publish(path);

//     //tf transform
//     tf::TransformBroadcaster tf_broadcaster;
//     tf::StampedTransform transform;
//     transform.frame_id_="map";
//     transform.child_frame_id_="camera";

//     string fn_kitti="/home/parallels/开源slam/catkin_vo_ws/00";
//     visual_odometry* vo_odom = new visual_odometry(fn_kitti);

//     int nframe=2;
//     while(ros::ok())
//     {
//         if(nframe >= 4000)
//         {
//             break;
//         }
//         std::string fn = fn_kitti+"/image_1/" + vo_odom->AddZeroPadding(nframe, 6) + ".png";
//         cv::Mat img_fn = cv::imread(fn,cv::IMREAD_GRAYSCALE);
//         vo_odom->image_test(prev_img,img_fn);
        

//         Matrix3d rotation_matrix;
//         cv::cv2eigen(all_R,rotation_matrix);
//         Vector3d translation_vector(all_t.at<double>(0),all_t.at<double>(1),all_t.at<double>(2));
        
//         Vector3d pose_new=rotation_matrix*pose_init+translation_vector;
//         cout<<"===pose_init:"<<pose_init<<endl;

//         Eigen::Quaterniond quaternion(rotation_matrix);

//         tf::Quaternion quat(quaternion.x(),quaternion.y(),quaternion.z(),quaternion.w());
        
//         tf::Vector3 _t(translation_vector.x(),translation_vector.y(),translation_vector.z());

//         transform.stamp_=ros::Time::now();
//         transform.setRotation(quat);
//         transform.setOrigin(_t);
//         tf_broadcaster.sendTransform(transform);

//         // if(prev_pts.size()<20)
//         // {
//         //    vo_odom->image_process(prev_img,img_fn);
//         // }
//         // else
//         // {
//         //     vo_odom->newImage_process(prev_img,nframe);
//         // }
//         // cout<<"===pose_init:"<<pose_init<<endl;
//         // Vector3d pose_new = all_T*pose_init;
//         // Matrix3d rotation_matrix=all_T.rotationMatrix();
//         // Vector3d translation_vector=all_T.translation();
//         // Eigen::Quaterniond quaternion(rotation_matrix);

//         // tf::Quaternion quat(quaternion.x(),quaternion.y(),quaternion.z(),quaternion.w());
        
//         // tf::Vector3 _t(translation_vector.x(),translation_vector.y(),translation_vector.z());

//         // transform.stamp_=ros::Time::now();
//         // transform.setRotation(quat);
//         // transform.setOrigin(_t);
//         // tf_broadcaster.sendTransform(transform);

//         //--------------pub odometry---------------------------//
//         nav_msgs::Odometry odom;
//         odom.header.stamp = ros::Time::now();
//         odom.header.frame_id = "/map";
//         odom.child_frame_id = "/camera";
        
//         geometry_msgs::Quaternion odom_quat;
        
//         odom_quat.x = quat[0];
//         odom_quat.y = quat[1];
//         odom_quat.z = quat[2];
//         odom_quat.w = quat[3];
//         // Set the position and rotation.
//         odom.pose.pose.position.x = pose_new.x();
//         odom.pose.pose.position.y = pose_new.y();
//         odom.pose.pose.position.z = pose_new.z();
//         odom.pose.pose.orientation = odom_quat;
//         pub_odometry.publish(odom);

//         //--------------pub path---------------------------//
//         pose_stamped.header.stamp=ros::Time::now();
//         pose_stamped.header.frame_id = "map";

//         pose_stamped.pose.position.x=pose_new.x();
//         pose_stamped.pose.position.y=pose_new.y();
//         pose_stamped.pose.position.z=pose_new.z();
        
//         cout<<"all_points3d_init[0].x"<<pose_stamped.pose.position.x<<endl;
//         cout<<"all_points3d_init[0].y"<<pose_stamped.pose.position.y<<endl;
//         cout<<"all_points3d_init[0].z"<<pose_stamped.pose.position.z<<endl;
//         cout<<"===================================================================="<<endl;
//         path.poses.push_back(pose_stamped);
//         pub_path.publish(path);
//         pose_init=pose_new;

//         //--------------pub image---------------------------//
//         Mat img_copy=img_fn;
//         cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
//         cv_ptr->encoding = "mono8";
//         cv_ptr->header.stamp = ros::Time::now();
//         cv_ptr->header.frame_id = "/map";
//         cv_ptr->image = img_copy;

//         // publish to /image.
//         imshow("pub_img",img_copy);
//         pub_img.publish(cv_ptr->toImageMsg());

//         nframe++;


//     }
    
    
//     // ros::Subscriber sub_img=nh.subscribe("/cam0/image_raw",100,img_callback);  
    
//     // ros::spin();
//     return 0;
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
    
    //tf transform
    tf::TransformBroadcaster tf_broadcaster;
    tf::StampedTransform transform;
    transform.frame_id_="map";
    transform.child_frame_id_="camera";

    string fn_kitti="/home/parallels/开源slam/catkin_vo_ws/00";
    visual_odometry* vo_odom = new visual_odometry(fn_kitti);

    //--------------pub path---------------------------//
    path.header.stamp=ros::Time::now();
    path.header.frame_id="map";

    //首先去发布初始的位置信息[0,0,0],之后的位置是通过计算得到的Rt和初始位置[0,0,0]计算得到的
    double x_init=0.0,y_init=0.0,z_init=0.0;
    pose_init.x()=0.0;
    pose_init.y()=0.0;
    pose_init.z()=0.0;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp=ros::Time::now();
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose.position.x=pose_init.x();
    pose_stamped.pose.position.y=pose_init.y();
    pose_stamped.pose.position.z=pose_init.z();
    path.poses.push_back(pose_stamped);
    pub_path.publish(path);

    int nframe=2;
    while(ros::ok())
    {
        if(nframe >= 4000)
        {
            break;
        }
        std::string fn = fn_kitti+"/image_1/" + vo_odom->AddZeroPadding(nframe, 6) + ".png";
        cv::Mat img_fn = cv::imread(fn,cv::IMREAD_GRAYSCALE);

        vo_odom->image_test(prev_img,img_fn);
        

        // Matrix3d rotation_matrix;
        // cv::cv2eigen(all_R,rotation_matrix);
        
        // Vector3d translation_vector(all_t.at<double>(0),all_t.at<double>(1),all_t.at<double>(2));
        // cout<<"===rotation_matrix:"<<rotation_matrix<<endl;
        // cout<<"===translation_vector:"<<translation_vector<<endl;
        
        // // Vector3d pose_new=rotation_matrix*pose_init+translation_vector;
        // cout<<"===pose_init:"<<pose_init<<endl;

        // Eigen::Quaterniond quaternion(rotation_matrix);

        // tf::Quaternion quat(quaternion.x(),quaternion.y(),quaternion.z(),quaternion.w());
        
        // tf::Vector3 _t(translation_vector.x(),translation_vector.y(),translation_vector.z());

        // transform.stamp_=ros::Time::now();
        // transform.setRotation(quat);
        // transform.setOrigin(_t);
        // tf_broadcaster.sendTransform(transform);
        // Vec3f euler=vo_odom->RotMatToEuler(all_R);
        tf::Matrix3x3 _R(all_R.at<double>(0,0),all_R.at<double>(0,1),all_R.at<double>(0,2),
                        all_R.at<double>(1,0),all_R.at<double>(1,1),all_R.at<double>(1,2),
                        all_R.at<double>(2,0),all_R.at<double>(2,1),all_R.at<double>(2,2));
        tf::Quaternion quat;
        _R.getRotation(quat);
        transform.stamp_=ros::Time::now();
        transform.setRotation(tf::Quaternion(quat[0], quat[1], quat[2], quat[3]));
        transform.setOrigin(tf::Vector3(all_t.at<double>(0), all_t.at<double>(1), all_t.at<double>(2)));

        // Braodcast the transform between /world and /camera.
        tf_broadcaster.sendTransform(transform);


        // if(prev_pts.size()<20)
        // {
        //    vo_odom->image_process(prev_img,img_fn);
        // }
        // else
        // {
        //     vo_odom->newImage_process(prev_img,nframe);
        // }
        // cout<<"===pose_init:"<<pose_init<<endl;
        // Vector3d pose_new = all_T*pose_init;
        // Matrix3d rotation_matrix=all_T.rotationMatrix();
        // Vector3d translation_vector=all_T.translation();
        // Eigen::Quaterniond quaternion(rotation_matrix);

        // tf::Quaternion quat(quaternion.x(),quaternion.y(),quaternion.z(),quaternion.w());
        
        // tf::Vector3 _t(translation_vector.x(),translation_vector.y(),translation_vector.z());

        // transform.stamp_=ros::Time::now();
        // transform.setRotation(quat);
        // transform.setOrigin(_t);
        // tf_broadcaster.sendTransform(transform);

        //--------------pub odometry---------------------------//
        
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "/map";
        odom.child_frame_id = "/camera";
        
        geometry_msgs::Quaternion odom_quat;
        
        odom_quat.x = quat[0];
        odom_quat.y = quat[1];
        odom_quat.z = quat[2];
        odom_quat.w = quat[3];
        // Set the position and rotation.
        odom.pose.pose.position.x = all_t.at<double>(0);
        odom.pose.pose.position.y = all_t.at<double>(1);
        odom.pose.pose.position.z = all_t.at<double>(2);
        odom.pose.pose.orientation = odom_quat;
        pub_odometry.publish(odom);
        // // Set the position and rotation.
        // odom.pose.pose.position.x = pose_new.x();
        // odom.pose.pose.position.y = pose_new.y();
        // odom.pose.pose.position.z = 1;//pose_new.z();
        // odom.pose.pose.orientation = odom_quat;
        // pub_odometry.publish(odom);

        //--------------pub path---------------------------//
        pose_stamped.header.stamp=ros::Time::now();
        pose_stamped.header.frame_id = "map";
    
        pose_stamped.pose.position.x=all_t.at<double>(0);
        pose_stamped.pose.position.y=all_t.at<double>(1);
        pose_stamped.pose.position.z=all_t.at<double>(2);

        // pose_stamped.pose.position.x=pose_new.x();
        // pose_stamped.pose.position.y=pose_new.y();
        // pose_stamped.pose.position.z=1;//pose_new.z();
        
        cout<<"all_points3d_init[0].x"<<pose_stamped.pose.position.x<<endl;
        cout<<"all_points3d_init[0].y"<<pose_stamped.pose.position.y<<endl;
        cout<<"all_points3d_init[0].z"<<pose_stamped.pose.position.z<<endl;
        cout<<"===================================================================="<<endl;
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);
        // pose_init=pose_new;

        //--------------pub image---------------------------//
        Mat img_copy=img_fn;
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        cv_ptr->encoding = "mono8";
        cv_ptr->header.stamp = ros::Time::now();
        cv_ptr->header.frame_id = "/map";
        cv_ptr->image = img_copy;

        // publish to /image.
        imshow("pub_img",img_copy);
        pub_img.publish(cv_ptr->toImageMsg());

        nframe++;


    }
    
    
    // ros::Subscriber sub_img=nh.subscribe("/cam0/image_raw",100,img_callback);  
    
    // ros::spin();
    return 0;
}