#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include<vector>
using namespace std;
using namespace cv;
Mat img_src;
Mat gray_img;
vector<Mat> channels;
vector<vector<Point>> counters;
vector<Vec4i>hierarchy;
class aim{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
public:
    aim():it_(nh_){
        image_sub_ = it_.subscribe("/cam_img", 1,  &aim::Image_cb, this);
    }
    void Image_cb(const sensor_msgs::ImageConstPtr& msg)
    {
        //convert ROS image msg to opencv Mat
        try
        {
            img_src = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        imshow("img",img_src);
        //cvtColor(img_src,gray_img,CV_BGR2GRAY);
        split(img_src,channels);
        absdiff(channels[0],channels[1],gray_img);
        find_led();
        waitKey(1);
    }
    bool find_led(){
        threshold(gray_img,gray_img,50,255,THRESH_BINARY);
        findContours(gray_img,counters,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_NONE);
        cvtColor(gray_img,gray_img,CV_GRAY2BGR);
        imshow("B",gray_img);
    }
};
int main(int argc, char** argv){
    ros::init(argc, argv, "aim_node");
    aim start;
    ros::spin();
    return 0;
}