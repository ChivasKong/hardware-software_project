#include <ros/ros.h>                           //ros 的头文件
#include <image_transport/image_transport.h>   //image_transport
#include <cv_bridge/cv_bridge.h>              //cv_bridge
#include <sensor_msgs/image_encodings.h>    //图像编码格式
#include <opencv2/imgproc/imgproc.hpp>      //图像处理
#include <opencv2/highgui/highgui.hpp>       //opencv GUI
#include <iostream>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";   //申明一个GUI 的显示的字符串

class ImageConverter    //申明一个图像转换的类
{
  ros::NodeHandle nh_;        //实例化一个节点
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;     //订阅节点
  image_transport::Publisher image_pub_;      //发布节点
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/dvs_msgs/event_pic", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)   //回调函数
  {
    cv_bridge::CvImagePtr cv_ptr;  //申明一个CvImagePtr
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
//转化为opencv的格式之后就可以对图像进行操作了
    // Draw an example circle on the video stream
    /*if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));  //画圆
    */
    // Update GUI Window

    vector<Vec3f> circles;
    cv::Mat img = cv_ptr->image;
    blur(img, img, cv::Size(3, 3));//opencv自带的均值滤波函数
    Mat rgb;
    cvtColor(img,rgb,COLOR_GRAY2BGR);
    if (img.rows > 60 && img.cols > 60){
      GaussianBlur( img, img, Size(9, 9), 2, 2 );

      HoughCircles( img, circles, CV_HOUGH_GRADIENT,2, img.rows/10, 200, 100, 0, 0 );
      // cout << "circle" << endl;

      //cv::circle(img, cv::Point(50, 50), 10, CV_RGB(255,0,0));  //画圆
      //circle(img,Point(10,10),100,CV_RGB(255,0,0),10);
    }

    for( size_t i = 0; i < circles.size(); i++ )
    {
      //参数定义
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      //绘制圆心
      circle( rgb, center, 3, Scalar(0,255,0), -1, 8, 0 );
      //绘制圆轮廓
      circle( rgb, center, radius, Scalar(155,50,255), 3, 8, 0 );
      //打印圆心坐标
      printf("x = %d,y = %d\n",cvRound(circles[i][0]),cvRound(circles[i][1]));
    }
   
    cv::imshow(OPENCV_WINDOW, rgb);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dvs_get_bag");
  ImageConverter ic;
  ros::spin();
  return 0;
}
