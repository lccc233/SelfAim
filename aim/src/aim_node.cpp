#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include<vector>
#include<std_msgs/Int8.h>
#include<math.h>
#include"aim/serial_info.h"
using namespace std;
using namespace cv;
Mat img_src;
Mat gray_img;
vector<Mat> channels;
vector<vector<Point> > counters;
vector<Vec4i>hierarchy;
Point2f points[4];
std_msgs::Int8 fps_msg;
vector<pair <Point,Point> >led;
vector<Point3d> small={Point3d(-0.0675,0.03,0),Point3d(0.0675,0.03,0),Point3d(0.0675,-0.03,0),Point3d(-0.0675,-0.03,0)};
vector<Point2d> armor_points;
Mat cameraMatrix,distCoeffs;
Mat tvec,rvec;
aim::serial_info armor_p;
struct armor{
    Point lu;
    Point ld;
    Point ru;
    Point rd;
}aim_armor;
int dis=100000;
Point last_px(640,240);
int lost_num=-1;
class aim_{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher fps_pub;
    ros::Publisher info_pub;
public:
    aim_():it_(nh_){
        image_sub_ = it_.subscribe("/cam_img", 1,  &aim_::Image_cb, this);
        fps_pub=nh_.advertise<std_msgs::Int8>("/fps",1);
        info_pub=nh_.advertise<aim::serial_info>("/result",10);
        cameraMatrix = (cv::Mat_<double>(3,3) << 858.4395, 0, 343.996, 0, 858.3275, 181.230, 0, 0, 1);
        distCoeffs=(cv::Mat_<double>(1,4)<< -0.0687,0.1047,0,0);
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
        //cvtColor(img_src,gray_img,CV_BGR2GRAY);
        split(img_src,channels);
        absdiff(channels[0],channels[1],gray_img);
        find_led();
        find_armor();
        imshow("img",img_src);
        waitKey(1);
        fps_msg.data=1;
        fps_pub.publish(fps_msg);
    }
    bool find_led(){
        led.clear();
        threshold(gray_img,gray_img,30,255,THRESH_BINARY);
        findContours(gray_img,counters,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_NONE);
        for(int n=0;n<counters.size();++n){
            RotatedRect led_rect=minAreaRect(counters[n]);
            int miny=0,mminy=1,else_p[2];
            else_p[0]=2,else_p[1]=3;
            if(points[0].y<points[1].y)miny=1,mminy=0;
            led_rect.points(points);
            for(int i=2;i<=3;++i){
                if(points[i].y<points[miny].y){
                    else_p[i-2]=miny;
                    miny=i;
                }
                if(points[miny].y<points[mminy].y){
                    int tmp=mminy;
                    mminy=miny;
                    miny=tmp;
                }
            }
            Point st=Point((points[mminy].x+points[miny].x)/2,(points[mminy].y+points[miny].y)/2);
            Point en=Point((points[else_p[0]].x+points[else_p[1]].x)/2,(points[else_p[0]].y+points[else_p[1]].y)/2);
            led.push_back(make_pair(st,en));
            //circle(img_src,st,10,Scalar(255,20,20),3);
            //circle(img_src,en,10,Scalar(30,200,20),3);
            /*for(int i=0;i<4;++i){
                if(i==3)line(img_src,points[i],points[0],Scalar(0,255,0),2,8,0);
                else line(img_src,points[i],points[i+1],Scalar(0,255,0),2,8,0);
                //cout<<"Point "<<i<<" "<<points[i].x<<" "<<points[i].y<<endl;
            }*/
        }
        //cout<<"LED num: "<<led.size()<<endl;
        cvtColor(gray_img,gray_img,CV_GRAY2BGR);
        imshow("B",gray_img);
    }
    void find_armor(){
        circle(img_src,last_px,10,Scalar(0,255,255),3);
        dis=1000000;
        int flag[led.size()];
        for(int i=0;i<led.size();++i)flag[i]=0;
        for(int i=0;(i<led.size()-1)&&(!flag[i]);++i){
            for(int j=i+1;j<led.size()&&(!flag[j]);++j){
                Point v1=led[i].first-led[i].second;
                Point v2=led[j].first-led[j].second;
                Point v3=(led[i].first+led[i].second)/2-(led[j].first+led[j].second)/2;
                double l1=sqrt(pow(v1.x,2)+pow(v1.y,2));
                double l2=sqrt(pow(v2.x,2)+pow(v2.y,2));
                double l3=sqrt(pow(v3.x,2)+pow(v3.y,2));
                //cout<<l1<<" "<<l2<<" "<<l3<<endl;
                if(max(l1,l2)/min(l1,l2)<1.5
                    &&(l3/min(l1,l2)<5)&&(l3/min(l1,l2)>1.25)
                    &&((v1.x*v2.x)+(v1.y*v2.y))/(l1*l2)>0.9945
                    &&((v1.x*v3.x)+(v1.y*v3.y))/(l1*l3)<0.35
                    &&abs(v3.x)>abs(v3.y)
                    &&min(l1,l2)>10
                    ){
                        circle(img_src,(led[i].first+led[j].second)/2,10,Scalar(0,0,255),3);
                        if(
                            sqrt(((led[i].first.x+led[j].second.x)/2-last_px.x)*((led[i].first.x+led[j].second.x)/2-last_px.x)+
                            ((led[i].first.y+led[j].second.y)/2-last_px.y)*((led[i].first.y+led[j].second.y)/2-last_px.y))<dis
                            ){
                                dis=sqrt(((led[i].first.x+led[j].second.x)/2-last_px.x)*((led[i].first.x+led[j].second.x)/2-last_px.x)+
                                                    ((led[i].first.y+led[j].second.y)/2-last_px.y)*((led[i].first.y+led[j].second.y)/2-last_px.y));
                                if(dis<40||lost_num==-1){
                                if(led[i].first.x<led[j].first.x){
                                    aim_armor.lu=led[i].first;
                                    aim_armor.ld=led[i].second;
                                    aim_armor.ru=led[j].first;
                                    aim_armor.rd=led[j].second;
                                }
                                else{
                                    aim_armor.lu=led[j].first;
                                    aim_armor.ld=led[j].second;
                                    aim_armor.ru=led[i].first;
                                    aim_armor.rd=led[i].second;
                                }}
                            }
                     }
                    //armor_points={led[i].first,led[j].first,led[j].second,led[i].second};
                    //solvePnP(small,armor_points,cameraMatrix,distCoeffs,rvec,tvec,false,SOLVEPNP_ITERATIVE);
                    //cout<<tvec.at<double>(0,0)<<" "<<tvec.at<double>(0,1)<<" "<<tvec.at<double>(0,2)<<endl;
                    //armor_p.x_p=tvec.at<double>(0,0);
                    //armor_p.y_p=tvec.at<double>(0,1);
                    //armor_p.z_p=tvec.at<double>(0,2);
                    //info_pub.publish(armor_p);
                    //circle(img_src,(led[i].first+led[j].second)/2,10,Scalar(0,0,255),3);
            }
        }
        if((lost_num==-1 && dis<1000000)||dis<40){
                        cout<<dis<<endl;
                        armor_points={aim_armor.lu,aim_armor.ru,aim_armor.rd,aim_armor.ld};
                        solvePnP(small,armor_points,cameraMatrix,distCoeffs,rvec,tvec,false,SOLVEPNP_P3P);
                        circle(img_src,(aim_armor.lu+aim_armor.rd)/2,10,Scalar(0,0,255),3);
                        circle(img_src,aim_armor.lu,10,Scalar(100,0,0),3);
                        circle(img_src,aim_armor.ld,10,Scalar(0,100,0),3);
                        circle(img_src,aim_armor.ru,10,Scalar(20,50,100),3);
                        circle(img_src,aim_armor.rd,10,Scalar(100,0,100),3);
                        lost_num=0;
                        last_px=(aim_armor.lu+aim_armor.rd)/2;
                    }
        else{
            lost_num++;
            if(lost_num==10){
                lost_num=-1;
                last_px.x=640;
                last_px.y=240;
            }
        }
    }
};
int main(int argc, char** argv){
    ros::init(argc, argv, "aim_node");
    aim_ start;
    ros::spin();
    return 0;
}