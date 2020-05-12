#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <iomanip>
#include <ros/console.h>
using namespace cv;
using namespace std;


std_msgs::String output;                        //declaration of global variable "output" to publish the coordinates on the topic - "coordinate_topic"

//callback function "locate()"
void locate(const sensor_msgs::ImageConstPtr& msg){
 	cv_bridge::CvImagePtr srcptr;
    srcptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);    // conversion of ros image to cv image
       
    if(srcptr==NULL){
      cout<<"null";
      return;
    }

    
    Mat src_gray;                                                            
    cvtColor( srcptr->image, src_gray, COLOR_BGR2GRAY );                     //convert rgb image to grayscale image
 
    blur( src_gray, src_gray, Size(7,7) );                                   // homogenous blur (apply filter to remove the noise)

    Mat thresholded;                                                        // segmentation(thresholding) of the image to find contours
    threshold( src_gray, thresholded, 65, 255,THRESH_BINARY );
	
    vector<vector<Point> > contours;                                           //find and store contours in the vector "contours"
    findContours( thresholded, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );
      
    vector<Moments> mu(contours.size() );                                   //calculate central moments of all the contours detected in the above step
    for( size_t i = 0; i < contours.size(); i++ )
    {
        mu[i] = moments( contours[i] );
    }
     

    for( size_t i = 0; i < contours.size(); i++ )                           //print the coordinates of the center on the screen as well as on the topic
    {
        output.data= "(" + std::to_string(mu[i].m10/(mu[i].m00 + 1e-5)) + "," + std::to_string(mu[i].m01/(mu[i].m00 + 1e-5)) + ")";
        cout<<"("<<mu[i].m10/(mu[i].m00 + 1e-5)<<","<<mu[i].m01/(mu[i].m00 + 1e-5)<<")    "<<endl;
        
    }
    return;
}
           
int main(int argc, char** argv){
ros::init(argc, argv, "coordinate");
ros::NodeHandle nh;
ros::Subscriber sub = nh.subscribe("magnus/camera/image_raw",10, locate);
ros::Publisher pub = nh.advertise<std_msgs::String>("coordinate_topic",10);
ros::Rate loopRate(30);

while(ros::ok()){
ros::spinOnce();
pub.publish(output);
loopRate.sleep();
}
return 0;}

