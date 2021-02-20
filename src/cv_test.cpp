#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include "std_msgs/Int32.h"

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>
#include <vector>
using namespace std;
static const std::string OPENCV_WINDOW = "Image window";
cv::RNG rng(12345);


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ros::Publisher pub = nh_.advertise<std_msgs::String>("cup", 1);
  std_msgs::String str;
  int xPosition;
  int yPosition;
  int width;
  int height;
  int CupDistance;
  int machineState; 
  double green_lowH;
  double green_lowS;
  double green_lowV;
  double green_highH;
  double green_highS;
  double green_highV;
  double red_lowH;
  double red_lowS;
  double red_lowV;
  double red_highH;
  double red_highS;
  double red_highV;
  double white_lowR;
  double white_lowG;
  double white_lowB;
  double white_highR;
  double white_highG;
  double white_highB;
  double black_lowR;
  double black_lowG;
  double black_lowB;
  double black_highR;
  double black_highG;
  double black_highB;
  int Cupdetected;
  int circleSize;
  int checkFinished;
  ImageConverter()
      : it_(nh_){
    // Subscrive to input video feed and publish output video feed
    // image_sub_ = it_.subscribe("camera/color/image_raw", 1, &ImageConverter::imageCb, this);
    image_sub_ = it_.subscribe("image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    nh_.getParam("width",width);
    nh_.getParam("height",height);
    xPosition = 0;
    yPosition = 0;
    checkFinished = 0;
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  int checkRow(int posRow ,int posCol,int circle,cv_bridge::CvImagePtr final){
    int whiteCount = 0;
    int blackCount = 0;
    for(int now = posRow; now < posRow + circle ; now +=2)
      if(now > 0 && now < height){
        vector<int> tmp;
        tmp = final->image.row(now).col(posCol);
        if(tmp[0] == 255)
          whiteCount ++;
        else
          blackCount ++;
      }
    if(whiteCount >= blackCount)
      return 1;
    return 0;  
  }

  int checkCol(int posRow ,int posCol,int circle,cv_bridge::CvImagePtr final){
    int whiteCount = 0;
        int blackCount = 0;
        for(int now = posCol; now < posCol + circle ; now +=2)
          if(now > 0 && now < width){
            vector<int> tmp;
            tmp = final->image.row(posRow).col(now);
            if(tmp[0] == 255)
              whiteCount ++;
            else
              blackCount ++; 
          }
        if(whiteCount >= blackCount)
          return 1;
        return 0;  
  }

  int checkCupPos(int circle ,cv_bridge::CvImagePtr final){
    for(int row = 0; row < height; row += 0.5 * circle )
      for(int col = 0; col < width; col += 0.5 * circle)
        if(checkCol(row,col,circle,final) && checkRow(row,col,circle,final)){
          xPosition = col + 0.5 * circle ;
          yPosition = row  + 0.5 * circle;
          return 1;
        }
    return 0;    
  }

  void findCupPosition(cv_bridge::CvImagePtr final){
    
    // if(checkFinished != 1)
    checkFinished = checkCupPos(circleSize, final);
    // cout << checkFinished <<endl;
    cv::circle(final->image, cv::Point(xPosition, yPosition), circleSize, CV_RGB(255, 255, 255));
    cv::imshow("Final Window",final->image);  
  }

  void drawContours(cv_bridge::CvImagePtr final,string name, int r, int g, int b){
    cv::Mat canny_output;
    cv::Canny( final->image, canny_output, 100, 200);
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours( canny_output, contours, hierarchy, 3, 2);
    cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( r,g,b);
        cv::drawContours( drawing, contours, (int)i, color, -1, 8, hierarchy, 0 );
    }
    cv::imshow(name, drawing );
  }

  void drawContoursAndCenter(cv_bridge::CvImagePtr final,string name, int r, int g, int b){
    cv::Mat canny_output;
    cv::Canny( final->image, canny_output, 100, 200);
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours( canny_output, contours, hierarchy, 3, 2);
    vector<cv::Moments>mu(contours.size());
    for (int i = 0; i < contours.size(); i++ )
    {
    mu[i] = moments(contours[i], false);
    }
    //計算影象的質心
    vector<cv::Point2f>mc(contours.size());
    for (int i = 0; i < contours.size(); i++ )
    {
    mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
    }
    cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( r,g,b);
        cv::drawContours( drawing, contours, (int)i, color, -1, 8, hierarchy, 0 );
        cv::circle(drawing, mc[i], 4, color, -1, 8, 0);
    }
    cv::imshow(name, drawing );
  }

  void drawContoursRedGreen(cv_bridge::CvImagePtr red, cv_bridge::CvImagePtr green){
    cv::Mat red_canny_output;
    cv::Canny( red->image, red_canny_output, 100, 200);
    cv::Mat green_canny_output;
    cv::Canny( green->image, green_canny_output, 100, 200);
    vector<vector<cv::Point>> red_contours;
    vector<cv::Vec4i> red_hierarchy;
    cv::findContours( red_canny_output, red_contours, red_hierarchy, 3, 2);
    cv::Mat red_drawing = cv::Mat::zeros( red_canny_output.size(), CV_8UC3 );
    for( size_t i = 0; i< red_contours.size(); i++ )
    {
        cv::Scalar red_color = cv::Scalar(0,0,255);
        cv::drawContours( red_drawing, red_contours, (int)i, red_color, -1, 8, red_hierarchy, 0 );
    }
    vector<vector<cv::Point>> green_contours;
    vector<cv::Vec4i> green_hierarchy;
    cv::findContours( green_canny_output, green_contours, green_hierarchy, 3, 2);
    cv::Mat green_drawing = cv::Mat::zeros( green_canny_output.size(), CV_8UC3 );
    for( size_t i = 0; i< green_contours.size(); i++ )
    {
        cv::Scalar green_color = cv::Scalar(0,255,0);
        cv::drawContours( green_drawing, green_contours, (int)i, green_color, -1, 8, green_hierarchy, 0 );
    }
    cv::addWeighted(red_drawing,1,green_drawing,1,0.0,red_drawing);

    //===================
    vector<cv::Moments>mu(green_contours.size());
    for (int i = 0; i < green_contours.size(); i++)
    {
      mu[i] = moments(green_contours[i], false);
    }
    vector<cv::Point2f>mc(green_contours.size());
    for(int i = 0; i < green_contours.size(); i++){
      mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
    }
    cv::Scalar blue_color = cv::Scalar(255,0,0);
    for(int i = 0; i < green_contours.size(); i++){
      cv::circle(red_drawing, mc[i], 4, blue_color, -1, 8, 0);
    }
    //===================

    cv::imshow("contour", red_drawing);
  }

  void imageCb(const sensor_msgs::ImageConstPtr &msg){
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr_green_;
    cv_bridge::CvImagePtr cv_ptr_red_;
    cv_bridge::CvImagePtr cv_final;
    cv_bridge::CvImagePtr cv_ptr_white_;
    cv_bridge::CvImagePtr cv_ptr_black_;
    cv_bridge::CvImagePtr cv_ptr_red_bgr_;
    cv_bridge::CvImagePtr cv_ptr_red_hsv_;

    cv_bridge::CvImagePtr cv_ptr_red_colored_;
    try{
      cv_ptr =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_ptr_green_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_ptr_red_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_ptr_red_bgr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_ptr_red_hsv_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_final = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_ptr_white_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_ptr_black_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    nh_.getParam("/green_lowH", green_lowH);
    nh_.getParam("/green_lowS", green_lowS);
    nh_.getParam("/green_lowV", green_lowV);
    nh_.getParam("/green_highH",green_highH);
    nh_.getParam("/green_highS",green_highS);
    nh_.getParam("/green_highV",green_highV);

    nh_.getParam("/red_lowH", red_lowH);
    nh_.getParam("/red_lowS", red_lowS);
    nh_.getParam("/red_lowV", red_lowV);
    nh_.getParam("/red_highH",red_highH);
    nh_.getParam("/red_highS",red_highS);
    nh_.getParam("/red_highV",red_highV);


    nh_.getParam("/white_lowR", white_lowR);
    nh_.getParam("/white_lowG", white_lowG);
    nh_.getParam("/white_lowB", white_lowB);
    nh_.getParam("/white_highR",white_highR);
    nh_.getParam("/white_highG",white_highG);
    nh_.getParam("/white_highB",white_highB);

    nh_.getParam("/black_lowR", black_lowR);
    nh_.getParam("/black_lowG", black_lowG);
    nh_.getParam("/black_lowB", black_lowB);
    nh_.getParam("/black_highR",black_highR);
    nh_.getParam("/black_highG",black_highG);
    nh_.getParam("/black_highB",black_highB);

    // nh_.getParam("/xPose",xPosition);
    // nh_.getParam("/yPose",yPosition);
    nh_.getParam("/circleSize",circleSize);


    
    cv::GaussianBlur(cv_ptr->image,cv_ptr->image,cv::Size(301, 101),2);
    // cv::circle(cv_ptr->image, cv::Point(xPosition, yPosition), circleSize, CV_RGB(0, 255, 0));
    cv::cvtColor(cv_ptr->image,cv_ptr_green_->image,cv::COLOR_BGR2HSV);
    cv::cvtColor(cv_ptr->image,cv_ptr_red_->image,cv::COLOR_BGR2HSV);
    cv::cvtColor(cv_ptr->image,cv_ptr_red_hsv_->image,cv::COLOR_BGR2HSV);
    
    // Update GUI Window
    // cout<<cv_ptr->image.col(xPosition).row(yPosition)<<endl;
    // cout<<"green_lowH , "<<green_lowH<<", green_lowS , "<<green_lowS<<", green_lowV , "<<green_lowV<<endl;
    // cout<<"green_highH , "<<green_highH<<", green_highS , "<<green_highS<<", green_highV , "<<green_highV<<endl;
    // cout<<"red_lowH , "<<red_lowH<<", red_lowS , "<<red_lowS<<", red_lowV , "<<red_lowV<<endl;
    // cout<<"red_highH , "<<red_highH<<", red_highS , "<<red_highS<<", red_highV , "<<red_highV<<endl;
    // cout<<"white_lowR , "<<white_lowR<<", white_lowG , "<<white_lowG<<", white_lowB , "<<white_lowB<<endl;
    // cout<<"white_highR , "<<white_highR<<", white_highG , "<<white_highG<<", white_highB , "<<white_highB<<endl;    
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    cv::inRange(cv_ptr_green_->image,cv::Scalar(green_lowH,green_lowS,green_lowV),cv::Scalar(green_highH,green_highS,green_highV),cv_ptr_green_->image);
    cv::inRange(cv_ptr_red_->image,cv::Scalar(red_lowH,red_lowS,red_lowV),cv::Scalar(red_highH,red_highS,red_highV),cv_ptr_red_->image);
    cv::inRange(cv_ptr_white_->image,cv::Scalar(white_lowR,white_lowG,white_lowB),cv::Scalar(white_highR,white_highG,white_highB),cv_ptr_white_->image);
    cv::inRange(cv_ptr_black_->image,cv::Scalar(black_lowR,black_lowG,black_lowB),cv::Scalar(black_highR,black_highG,black_highB),cv_ptr_black_->image);


    cv::threshold(cv_ptr_red_->image,cv_ptr_red_->image,254,255,1);

    // cv::inRange(cv_ptr_red_bgr_->image,cv::Scalar(0,0,100),cv::Scalar(255,80,255),cv_ptr_red_bgr_->image);
    // cv::imshow("Red bgr Window",cv_ptr_red_bgr_->image);

    cv::inRange(cv_ptr_red_hsv_->image,cv::Scalar(0,110,110),cv::Scalar(255,255,255),cv_ptr_red_hsv_->image);
    

    cv::addWeighted(cv_ptr_red_->image,0.5,cv_ptr_red_hsv_->image,0.5,0.0,cv_ptr_red_->image);
    cv::threshold(cv_ptr_red_->image,cv_ptr_red_->image,254,255,0);

    cv::Mat a = cv_ptr_red_->image;
    cv::cvtColor(a, a, CV_GRAY2BGR);
    // cv::cvtColor(a, a, CV_BGR2GRAY);

    // cv::Mat b;
    // cv::Mat b;
    // cv::cvtColor(a,b,CV_BGR2GRAY);
    // cv::Mat mask;
    // double grayThres = cv::threshold(a, mask, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
    // cv::cvtColor(a, a, CV_GRAY2BGR);
    // a.setTo(cv::Scalar(0,0,255), a);

    cv::imshow("Red Window",cv_ptr_red_->image);
    // cv::imshow("Reddd Window",a);
    // cv::cvtColor(cv_ptr_red_->image, cv_ptr_red_colored_->image, CV_GRAY2BGR);
    // cv::Mat a = cv_ptr_red_colored_->image;
    // a.setTo(cv::Scalar(0,0,255), cv_ptr_red_->image);
    // cv::imshow("Red Window",cv_ptr_red_colored_->image);
    // cv::Mat gray;
    // cv::Mat mask;
    // double grayThres = cv::threshold(gray, mask, 0, 255, 0);
    // cv_ptr_red_->image.setTo(cv::Scalar(0,0,255), mask);
    // cv::imshow("Red Window",cv_ptr_red_->image);

    cv::waitKey(3);
    // cv::imshow("Red inverted Window",cv_ptr_red_->image);
    cv::imshow("Green Window",cv_ptr_green_->image);
    // cv::waitKey(3);
    // cv::imshow("Red Window",cv_ptr_red_->image);
    // cv::waitKey(3);
    cv::addWeighted(cv_ptr_red_->image,1,cv_ptr_green_->image,1,0.0,cv_final->image);
    cv::threshold(cv_final->image,cv_final->image,254,255,0);
    // cv::subtract(cv_final->image, cv_ptr_white_->image ,cv_final->image);
    // cv::subtract(cv_final->image, cv_ptr_black_->image ,cv_final->image);

    // Output modified video stream
    // image_pub_.publish(cv_ptr->toImageMsg());

    // drawContoursAndCenter(cv_ptr_green_,"gc",0,255,0);
    // drawContours(cv_ptr_red_,"rc",0,0,255);
    // drawContours(cv_final,"fnal",0,255,0);
    drawContoursRedGreen(cv_ptr_red_,cv_ptr_green_);

    findCupPosition(cv_final);
    cv::circle(cv_ptr->image, cv::Point(xPosition, yPosition), circleSize, CV_RGB(255, 255, 255));
    cv::imshow(OPENCV_WINDOW,cv_ptr->image);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cvTest");
  ImageConverter ic;
  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
