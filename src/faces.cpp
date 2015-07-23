#include "ros/ros.h"
#include "std_msgs/String.h"
#include "facedetector/Detection.h"
#include <ardrone_autonomy/Navdata.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include "opencv2/core/mat.hpp"
#include <vector>


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

static const std::string OPENCV_WINDOW = "Image window";

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

int cam_h = 360;
int cam_w = 640;
double fov_v = 38;
double fov_u = 70;

double A_exp = .4*.3; // pričakovana velikost objekta
double d_exp = 3; //pričakovana razdalja do objekta
double psi_ref = 0;
double diff_yaw = 0;

double alpha_x = 460;
double alpha_y = 530;

double x_ref = 0.5;
double y_ref = 0.5;
double d_ref = 6;

int v_lim = 50;



ros::Publisher cmd_pub;

geometry_msgs::Twist msg;
std::vector<cv::Point> v;
std::vector<cv::Point> v_kf;
KalmanFilter KF(4, 2, 0);
Mat_<float> measurement(2,1);

float distanceToTarget( const float fD) {

    float depth;
    depth = sqrt(alpha_x*alpha_y*A_exp/(cam_w*cam_h)) * fD;

    return depth;

}

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
  cv::Point p = cv::Point((double)cam_w/2, (double)cam_h/2);
  cv::circle(cv_ptr->image, p, 5, Scalar(0,0,0), -1);
  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);
}

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
  double fu,fv,fd;
  double pitch, yaw, roll, z;
  double pitch_err, yaw_err, roll_err, z_err;
  double pitch_corr, yaw_corr, roll_corr, z_corr;
  double psi, theta, sign_diff_yaw;

  double delta_fu, delta_fv, delta_fd;

  for (int i = 0; i < det_msg->image.size();i++){
    x = det_msg->x[i];y = det_msg->y[i];h = det_msg->height[i];w = det_msg->width[i];

    // push point to list
    v.push_back(cv::Point(x+w/2,y+h/2));
    if (v.size()>v_lim){
    	v.erase(v.begin());
    }
    // display trajectory
    for (vector<cv::Point>::iterator it = v.begin(); it!=v.end(); ++it){
  		cv::Point cur = *it;  		
  		//cv::circle(cv_ptr->image, cur, 2, Scalar(0,0,255), -1);
  		if (it!=v.begin()){ // for displaying the trajectory of past detections
  			cv::Point cur2 = *(--it);
  			++it;
			cv::line(cv_ptr->image, cur, cur2, Scalar(0,255,0), 2);
  		}
  	}

  	// update KF
  	Mat prediction = KF.predict();
	Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
	measurement(0) = x+w/2;
	measurement(1) = y+h/2;
	Mat estimated = KF.correct(measurement);
	Point statePt(estimated.at<float>(0),estimated.at<float>(1));
	cv::circle(cv_ptr->image, statePt, 5, Scalar(255,255,255), -1);

	//push predicted point to another list
	v_kf.push_back(statePt);
	if (v_kf.size()>v_lim){
    	v_kf.erase(v_kf.begin());
    }
    for (vector<cv::Point>::iterator it = v_kf.begin(); it!=v_kf.end(); ++it){
  		cv::Point cur = *it;  		
  		if (it!=v_kf.begin()){ // for displaying the trajectory of past detections
  			cv::Point cur2 = *(--it);
  			++it;
			cv::line(cv_ptr->image, cur, cur2, Scalar(0,0,255), 2);
  		}
  	}


    cv::line(cv_ptr->image, cv::Point(cam_w/2, cam_h/2),cv::Point(x+w/2,y+h/2),Scalar(0,255,255),2);

    if (i==0){ // assumes one face
      theta = nav_msg->vy;
      psi = nav_msg->rotZ;
      psi_ref = psi;

      sign_diff_yaw = (cos(psi)*sin(psi_ref)-cos(psi_ref)*sin(psi))>0 ? 1 : -1;
      diff_yaw = sign_diff_yaw*acos(cos(psi)*cos(psi_ref)+sin(psi)*sin(psi_ref));
      if (fabs(diff_yaw)>25.0){
      	psi_ref = psi;
      	diff_yaw = 0;
      }

      // featurji iz detektorja
      fu = (x+(double)(w/2))/cam_w;
      fv = (y+(double)(h/2))/cam_h;
      fd = sqrt((cam_w*cam_h)/(w*h));
      // razdalja med referenco in trenutno meritvijo
      delta_fu = x_ref-fu;
      delta_fv = y_ref-fv;
      delta_fd = d_ref-fd;
      // računanje napak po formulah
      yaw_err = delta_fu;
      roll_err = delta_fu-(double)((psi_ref-psi)/fov_u);
      z_err = delta_fv-(double)((-theta)/fov_v);
      pitch_err = delta_fd;
      // upoštevanje popravkov
      yaw_corr = x_ref-yaw_err;
      roll_corr = x_ref-roll_err;
      z_corr = y_ref-z_err;
      pitch_corr = d_ref-pitch_err;

      //izračunamo ukaze
      yaw = x_ref-yaw_corr;
      roll = x_ref-roll_corr;
      z = y_ref-z_corr;
      pitch = d_ref-pitch_corr;


      //pitch = delta_fd;
      //pitch = 0.3*(-(6-fd));
      //yaw = delta_fu;
      //yaw = 0.5-fu; // treba je gledat oddaljenost od središča, torej minimiziramo fu-0.5 = 0
      //roll = delta_fu-(double)((psi_ref-psi)/fov_u);
      //roll = 0.5-(fu-(double)((psi_ref-psi)/fov_u));
      //z = delta_fv-(double)((-theta)/fov_v);
      //z = (0.5-fv)*0.7;

      //get distance
      /*double Dxs, Dys, Dzs, DYs;
      double x_con,y_con, Y_con, z_con;
      x_con = sqrt((alpha_x*alpha_y*A_exp)/(cam_h*cam_w));
      y_con = d_exp*cam_w/alpha_x;
      z_con = d_exp*cam_h/alpha_y;
      Y_con = fov_u*(M_PI/180.0);
      Dxs = x_con*fd;
      Dys = y_con*(fu-0.5);
      Dzs = z_con*(fv-0.5);
      DYs = Y_con*(fu-0.5);*/

      //ROS_INFO("fu = %f, fv = %f, fd = %f", fu, fv, fd);
      //ROS_INFO("fu = %f, fv = %f, fd = %f, depth = %f", fu, fv, fd,distanceToTarget(fd));
      //ROS_INFO("pitch = %f, roll = %f, yaw = %f, z = %f", pitch, roll, yaw, z);
      //ROS_INFO("pitch = %f, roll = %f, yaw = %f, z = %f", pitch_corr, roll_corr, yaw_corr, z_corr);
      //ROS_INFO("Dxs = %f, Dys = %f, Dzs = %f, DYs = %f", Dxs, Dys, Dzs, DYs);
      //ROS_INFO("psi = %f, psi_ref = %f, diff_yaw = %f", psi, psi_ref, diff_yaw);
      //ROS_INFO("delta fu = %f, delta fv = %f, delta fd = %f", delta_fu, delta_fv, delta_fd);
      
      //ROS_INFO("delta z = %f, delta x = %f", z, fd-p_fd);
      //ROS_INFO("\n");
    }

    msg.linear.x = pitch;
    msg.linear.y = roll;
    msg.linear.z = z;
    msg.angular.z = yaw;

    //cmd_pub.publish(msg);

    cv::Rect r = cv::Rect(det_msg->x[i],det_msg->y[i],det_msg->height[i],det_msg->width[i]);
    cv::rectangle(cv_ptr->image, r, Scalar(0,0,255),2);
  }

  cv::Point p = cv::Point((double)cam_w/2, (double)cam_h/2);
  cv::circle(cv_ptr->image, p, 5, Scalar(0,0,0), -1);

  cv:imshow(OPENCV_WINDOW, cv_ptr->image);
  //cv::waitKey(25);
  cv::waitKey(3);

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

  cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  // KF init
  	KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,2,0,   0,1,0,2,  0,0,1,0,  0,0,0,1);
	measurement.setTo(Scalar(0));
	KF.statePre.at<float>(0) = cam_w/2;
	KF.statePre.at<float>(1) = cam_h/2;
	KF.statePre.at<float>(2) = 0;
	KF.statePre.at<float>(3) = 0;
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(KF.errorCovPost, Scalar::all(.1));
	

  cv::namedWindow(OPENCV_WINDOW);

  ros::spin();

  return 0;
}