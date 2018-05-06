#include <ros/ros.h>

#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/highgui/highgui.hpp>  
using namespace std;
using namespace cv;
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{

    
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "listener_");

	ros::NodeHandle n;

	ros::Subscriber dvs_subscriber = n.subscribe("/dvs_msgs/event_pic", 1000, imageCallback);

	ros::spin();

}
