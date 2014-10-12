#ifndef my_namespace__my_plugin_H
#define my_namespace__my_plugin_H

//**** C++ ****
#include <string>


//**** ROS ****
#include <ros/ros.h>
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

//**** QT and RQT ****
#include <rqt_gui_cpp/plugin.h>
//#include <my_namespace/ui_my_plugin.h>
#include <ui_my_plugin.h>
//#include <QWidget> //po "navodilih"
#include <QtGui/QWidget>

#include <QBasicTimer>
#include <QString>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <QTextStream>
#include <QButtonGroup>

//**** DRONE ****
#include "ardrone_autonomy/Navdata.h"
#include <ardrone_autonomy/CamSelect.h>

namespace my_namespace {

class batterySignal : public QObject
 {
     Q_OBJECT

 public:
     batterySignal() { m_value = 100; }

     int value() const { return m_value; }

 public slots:
 	void setValue(int value);

 signals:
     void valueChanged(int newValue);

 private:
     int m_value;

 };

class MyPlugin : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
  
public:
  MyPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
  
protected slots:
  //**** KRMILJENJE:
  virtual void click_take_off_button();
  virtual void click_land_button();
  virtual void click_emergency_off_button();
  virtual void spinbox_changed(double vrednost);
  virtual void spinbox2_changed(double vrednost);
  virtual void clickCameraFront();
  virtual void clickCameraBottom();
  virtual void clickAutoLand();
  // + checkbox
  
  
  //****  TUM AR.Drone:
  virtual void clickAddButton();  // narejeno splošno->lahko kličeju tudi druge funkcije
  virtual void clickClearButton(); // narejeno splošno->lahko kličeju tudi druge funkcije
  virtual void clickSendButton(); // narejeno splošno->lahko kličeju tudi druge funkcije
  virtual void clickResetButton();// narejeno splošno->lahko kličeju tudi druge funkcije
  virtual void comboBoxCommand(QString); // + vrednost
  virtual void openFileButton();
  virtual void radioB_joy();
  virtual void radioB_autopilot();
  virtual void radioB_joyAndAuto();
  
protected:
  virtual void drone_take_off();
  virtual void drone_land();
  virtual void drone_emergency();
  void camera_select(char cam);
  
  //virtual void test(std::string niz="jupej");
  //virtual void test(QString niz="jupej");
  virtual void test(QString niz);
  
  void timerEvent(QTimerEvent *event);
  void publish_vel();
  
  //navadata
  void navdata_callback(const ardrone_autonomy::Navdata& msg);
  
  // joy
  void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
//  virtual void clickoff();
  
//**** 

  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();
private:
//  Ui::MyPluginWidget ui_;
//  Ui::Ui_Form ui_;
  Ui::Drone ui_;
  QWidget* widget_;
  
  ros::Publisher pub_take_off;	// publish take off
  ros::Publisher pub_land;		// publish land
  ros::Publisher pub_emergency;	// publish emergency
  ros::Subscriber navdata_sub;	// navigation data
  //ros::ServiceClient cam_client;	// za preklop kamere-izberemo kamero
  //ardrone_autonomy::CamSelectRequest cam_client;
  ros::NodeHandle n_;
  
  ros::Publisher pub_tum_commands; // publisher for tum commands
//  ros::Subscriber sub_tum_commands; // subscriber for tum commands
  
  ros::Subscriber joy_sub_;		// joy subscriber
  ros::Publisher joy_vel_;		// publish vel for controling drone
  ros::NodeHandle n_joy;
  geometry_msgs::Twist last_send_vel;
  
  ardrone_autonomy::Navdata navdata_msg;
  
  float axes_max;
  float axes_scale;
  float pitch_y;
  float roll_x;
  float yaw_z;
  float hight_z;
  int battery;
  batterySignal battUpdate;
  
  char cam_sel;		// selected camera [0,1]
  int topics_ok;
  
  QStringList TumList; // vsebuje stringe/imena ukazov
  
};


} // namespace
#endif // my_namespace__my_plugin_H
