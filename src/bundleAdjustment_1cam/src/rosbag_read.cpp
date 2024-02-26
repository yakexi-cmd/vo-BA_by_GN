#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

ros::Publisher pub_img;
bool init_pub=true;
int count_;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    
    // if(init_pub)
    // {
    //     count=0;
    //     init_pub=false;
    // }
    
    cv_bridge::CvImageConstPtr ptr;
    if(img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header=img_msg->header;
        img.height=img_msg->height;
        img.width=img_msg->width;
        img.is_bigendian=img_msg->is_bigendian;
        img.step=img_msg->step;
        img.data=img_msg->data;
        img.encoding="mono8";
        ptr=cv_bridge::toCvCopy(img,sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr=cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::MONO8);
    Mat show_img=ptr->image;

    string filename="image/"+to_string(count_++)+".png";
    cv::imwrite(filename,show_img);
    
}

int main(int argc,char **argv)
{
    
    ros::init(argc,argv,"readImage_node");
    ros::NodeHandle nh("~");

    ros::Subscriber sub_img=nh.subscribe("/cam0/image_raw",100,img_callback);
    ros::spin();
    
    return 0;
}
