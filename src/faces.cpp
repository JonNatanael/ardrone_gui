#include "ros/ros.h"
#include "std_msgs/String.h"
#include "facedetector/Detection.h"
#include <ardrone_autonomy/Navdata.h>
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
double p_fu = 0;
double p_fv = 0;
double p_fd = 0;
double p_fz = 0;

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

void callback(const facedetector::Detection::ConstPtr& det_msg, const ImageConstPtr& cam_msg, const ardrone_autonomy::Navdata::ConstPtr& nav_msg){
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
  int x,y,w,h;
  double fu,fv,fd,fz;

  for (int i = 0; i < det_msg->image.size();i++){
    x = det_msg->x[i];y = det_msg->y[i];h = det_msg->height[i];w = det_msg->width[i];

    if (i==0){ // assumes one face
      if (p_x != 0 && p_y != 0){
        // if previous state is not 0, draw line between detection centers and previous position rectangle
        cv::rectangle(cv_ptr->image, cv::Rect(p_x,p_y,p_h,p_w), Scalar(0,255,255),2);
        cv::line(cv_ptr->image, cv::Point(p_x+(p_w/2), p_y+(p_h/2)),cv::Point(det_msg->x[i]+(det_msg->width[i]/2),det_msg->y[i]+(det_msg->height[i]/2)),Scalar(255,0,0));
      }

      fu = (x+(double)(w/2))/cam_w;
      fv = (y+(double)(h/2))/cam_h;
      fd = sqrt((cam_w*cam_h)/(w*h));
      fz = abs(p_fu-fu)-(double)((-nav_msg->vy)/35); // 35 je FOV po y

      //ROS_INFO("fu = %f, fv = %f, fd = %f", fu, fv, fd);
      //ROS_INFO("p_fu = %f, p_fv = %f, p_fd = %f", p_fu, p_fv, p_fd);
      ROS_INFO("delta fu = %f, delta fv = %f, delta fd = %f", fu-p_fu, fv-p_fv, fd-p_fd);
      //ROS_INFO("delta z = %f, delta x = %f", fz, fd-p_fd);
      ROS_INFO("\n");

      //save current state
      p_x = det_msg->x[i]; p_y = det_msg->y[i]; p_h = det_msg->height[i]; p_w = det_msg->width[i];
      p_fu = fu; p_fv = fv; p_fd = fd;
    }

    cv::Rect r = cv::Rect(det_msg->x[i],det_msg->y[i],det_msg->height[i],det_msg->width[i]);
    cv::rectangle(cv_ptr->image, r, Scalar(0,0,255),2);
  }

  cv:imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(200);

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "faces");

  ros::NodeHandle n;

  ros::Subscriber sub_image = n.subscribe("/ardrone/image_raw", 1, imageCallback);

  message_filters::Subscriber<facedetector::Detection> sub_faces(n, "/facedetector/faces", 1);
  message_filters::Subscriber<Image> sub_camera(n, "/ardrone/image_raw", 1);
  message_filters::Subscriber<ardrone_autonomy::Navdata> sub_navdata(n, "/ardrone/navdata",1);
  //navdata_sub = n_.subscribe("/ardrone/navdata", 1, &MyPlugin::navdata_callback, this);

  typedef sync_policies::ApproximateTime<facedetector::Detection, Image, ardrone_autonomy::Navdata> MySyncPolicy;

  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_faces, sub_camera, sub_navdata);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  cv::namedWindow(OPENCV_WINDOW);

  ros::spin();

  return 0;
}