#include "ros/ros.h"
#include "std_msgs/String.h"
#include "facedetector/Detection.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

static const std::string OPENCV_WINDOW = "Image window";

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

int p_x = 0;
int p_y = 0;
int p_h = 0;
int p_w = 0;

int cam_h = 360;
int cam_w = 640;

void imageCallback(const ImageConstPtr& cam_msg){
  // just image display
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(cam_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv:imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);
}

/*void facesCallback(const facedetector::Detection::ConstPtr& det_msg){

}*/

void callback(const facedetector::Detection::ConstPtr& det_msg, const ImageConstPtr& cam_msg){
  // syncronised image & detection
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(cam_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //init for temp. variables
  int x,y,w,h,fu,fv,fd;

  for (int i = 0; i < det_msg->image.size();i++){
    x = det_msg->x[i];y = det_msg->y[i];h = det_msg->height[i];w = det_msg->width[i];

    if (i==0){ // assumes one face
      if (p_x != 0 && p_y != 0){
        // if previous state is not 0, draw line between detection centers and previous position rectangle
        cv::rectangle(cv_ptr->image, cv::Rect(p_x,p_y,p_h,p_w), Scalar(0,255,255),2);
        cv::line(cv_ptr->image, cv::Point(p_x+(p_w/2), p_y+(p_h/2)),cv::Point(det_msg->x[i]+(det_msg->width[i]/2),det_msg->y[i]+(det_msg->height[i]/2)),Scalar(255,0,0));
      }

      /*ROS_INFO("%d, %d", p_x, p_y);
      ROS_INFO("%d, %d", det_msg->x[i], det_msg->y[i]);
      ROS_INFO("\n");*/

      //save current state
      p_x = det_msg->x[i]; p_y = det_msg->y[i]; p_h = det_msg->height[i]; p_w = det_msg->width[i];
    }

    cv::Rect r = cv::Rect(det_msg->x[i],det_msg->y[i],det_msg->height[i],det_msg->width[i]);
    cv::rectangle(cv_ptr->image, r, Scalar(0,0,255),2);
  }

  cv:imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "faces");

  ros::NodeHandle n;

  ros::Subscriber sub_image = n.subscribe("/ardrone/image_raw", 1, imageCallback);

  message_filters::Subscriber<facedetector::Detection> sub_faces(n, "/facedetector/faces", 1);
  message_filters::Subscriber<Image> sub_camera(n, "/ardrone/image_raw", 1);

  typedef sync_policies::ApproximateTime<facedetector::Detection, Image> MySyncPolicy;

  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_faces, sub_camera);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  cv::namedWindow(OPENCV_WINDOW);

  ros::spin();

  return 0;
}