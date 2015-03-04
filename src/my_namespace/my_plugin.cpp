#include <my_namespace/my_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QObject>
#include <QMetaObject>
#include <Qt>
//#include <ros/publisher.h>
//#include <ros/subscriber.h>
#include <ros/ros.h>
#include <ros/service_client.h>


namespace my_namespace {

MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("MyPlugin");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);
  
  ros::start();
  
//  clickoff();
  // poveži funkcije (callback) s tipkami
  //****  krmiljenje:
  connect( ui_.take_off_b, 		SIGNAL(pressed()), 				this, SLOT(click_take_off_button())		);
  connect( ui_.land_b, 			SIGNAL(pressed()), 				this, SLOT(click_land_button())			);
  connect( ui_.emergency_off_b, SIGNAL(toggled(bool)), 				this, SLOT(click_emergency_off_button()));
  connect( ui_.doubleSpinBox, 	SIGNAL(valueChanged(double)), 	this, SLOT(spinbox_changed(double)) 	);
  connect( ui_.doubleSpinBox_2, SIGNAL(valueChanged(double)), 	this, SLOT(spinbox2_changed(double)) 	);
  connect( ui_.set_front_cam_b, SIGNAL(pressed()),				this, SLOT(clickCameraFront())			);
  connect( ui_.set_bottom_cam_b, SIGNAL(pressed()),				this, SLOT(clickCameraBottom())			);
  connect( ui_.Button_autoLand, SIGNAL(pressed()),				this, SLOT(clickAutoLand())				);
  connect (ui_.imu_calib_btn, SIGNAL(pressed()), this, SLOT(clickIMUCalib()));
  connect (ui_.flat_trim_btn, SIGNAL(pressed()), this, SLOT(clickFlatTrim()));
  connect (ui_.usb_record_start_btn, SIGNAL(pressed()), this, SLOT(clickUSBRecordStart()));
  connect (ui_.usb_record_stop_btn, SIGNAL(pressed()), this, SLOT(clickUSBRecordStop()));
    
  //****  tum ar.drone
  connect( ui_.Button_add,		SIGNAL(pressed()),				this, SLOT(clickAddButton())			);
  connect( ui_.Button_clear,	SIGNAL(pressed()),				this, SLOT(clickClearButton())			);
  connect( ui_.Button_send, 	SIGNAL(pressed()),				this, SLOT(clickSendButton())			);
  connect( ui_.Button_reset,	SIGNAL(pressed()),				this, SLOT(clickResetButton())			);
  connect( ui_.comboBox_commands, SIGNAL(currentIndexChanged(QString)),			this, SLOT(comboBoxCommand(QString))	);
  connect( ui_.Button_openFile, SIGNAL(pressed()),			this, SLOT(openFileButton())			);
  connect( ui_.radioB_onlyJ,    SIGNAL(pressed()),			this, SLOT(radioB_joy())				);
  connect( ui_.radioB_tumAP,    SIGNAL(pressed()),			this, SLOT(radioB_autopilot())			);
  connect( ui_.radioB_jAndAuto, SIGNAL(pressed()),			this, SLOT(radioB_joyAndAuto())			);
  
//  connect(ui_.emergency_off_b, SIGNAL(pressed()), this, SLOT(clickoff()));

//**** battery status update
connect(&battUpdate, SIGNAL(valueChanged(int)), ui_.progressBar, SLOT(setValue(int)));
  
  // publishers:
  // 
  pub_take_off 	= n_.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  pub_land 		= n_.advertise<std_msgs::Empty>("/ardrone/land", 1);
  pub_emergency = n_.advertise<std_msgs::Empty>("/ardrone/reset", 1);
  
//  cam_client = n_.serviceClient<ardrone_>("ardrone/setcamchannel");
//	cam_client = n_.serviceClient("ardrone/setcamchannel");
  
  //navdata sub:
	//navdata_sub = n_.subscribe<ardrone_autonomy::Navdata,MyPlugin>("/ardrone/navdata", 1, &MyPlugin::navdata_callback, this);
	navdata_sub = n_.subscribe("/ardrone/navdata", 1, &MyPlugin::navdata_callback, this);
  
  //joy:
  joy_sub_ = n_joy.subscribe<sensor_msgs::Joy,MyPlugin>("/joy", 1, &MyPlugin::joy_callback, this);
  joy_vel_ = n_joy.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  
  // tum drone commands
  pub_tum_commands = n_.advertise<std_msgs::String>("/tum_ardrone/com", 1);
//  sub_tum_commands = n_.subscribe<std_msgs::String>("/tum_ardrone/com", 1, &MyPlugin::subTumCommands, this);
  
//  start();
	
	axes_max	= (float)ui_.doubleSpinBox->value();
	axes_scale	= (float)ui_.doubleSpinBox_2->value();
	
	// call service?
	camera_select(0);
	
	topics_ok = 1;

	// command list
	buttonListSingle
	<< "button 0"
	<< "button 1"
	<< "button 2"
	<< "button 3"
	<< "l-trigger 1"
	<< "l-trigger 2"
	<< "r-trigger 1"
	<< "r-trigger 2"
	<< "cross up"
	<< "cross down"
	<< "cross left"
	<< "cross right";

	buttonListAxis
	<< "cross up-down"
	<< "cross left-right"
	<< "l-stick up-down"
	<< "r-stick left-right"
	<< "l-stick up-down"
	<< "r-stick left-right";

	ui_.comboBox_mapping_0->addItems(buttonListSingle);
	ui_.comboBox_mapping_1->addItems(buttonListSingle);
	ui_.comboBox_mapping_2->addItems(buttonListSingle);
	ui_.comboBox_mapping_3->addItems(buttonListSingle);
	ui_.comboBox_mapping_4->addItems(buttonListAxis);
	ui_.comboBox_mapping_5->addItems(buttonListAxis);
	ui_.comboBox_mapping_6->addItems(buttonListAxis);
	ui_.comboBox_mapping_7->addItems(buttonListAxis);
	
	//Tum combobox:
	TumList
	<< "c autoInit"
	<< "c autoTakeover"
	<< "c clearCommands"
	<< "c goto"
	<< "c land"
	<< "c lockScaleFP"
	<< "c moveBy"
	<< "c moveByRel"
	<< "f reset"
	<< "c setInitialReachDist"
	<< "c setMaxControl"
	<< "c setReference"
	<< "c setStayWithinDist"
	<< "c setStayTime"
	<< "c start"
	<< "c stop";

	ui_.comboBox_commands->addItems(TumList);
	
	battery = 100;
	startTimer(25);
	state = 2;
}

void MyPlugin::shutdownPlugin()
{
  // TODO unregister all publishers here
  pub_take_off.shutdown();
  pub_land.shutdown();
  pub_emergency.shutdown();
  navdata_sub.shutdown();
  n_.shutdown();
  
  joy_sub_.shutdown();
  joy_vel_.shutdown();
  n_joy.shutdown();
  
  topics_ok = 0;
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

// POVRATNI KLICI:

//**** krmiljenje:

void MyPlugin::click_take_off_button(){
	MyPlugin::drone_take_off();
	test("Taking off");
}

void MyPlugin::click_land_button(){
	MyPlugin::drone_land();
	test("Landing");
	//std::cout<<"test\n";
}

void MyPlugin::click_emergency_off_button(){
	MyPlugin::drone_emergency();
	//MyPlugin::test("pristanek v sili");
}

void MyPlugin::clickCameraFront(){
	MyPlugin::camera_select(0);
	// set message: selected bottom camera
}

void MyPlugin::clickCameraBottom(){
	MyPlugin::camera_select(1);
	// set mesage: selected bottom camera
}

void MyPlugin::clickAutoLand(){
	test("Ni se implementirano.");
}

void MyPlugin::clickIMUCalib(){
	IMU_calibrate();
}

void MyPlugin::clickFlatTrim(){
	flat_trim();
}

void MyPlugin::clickUSBRecordStart(){
	USB_record_start();
}

void MyPlugin::clickUSBRecordStop(){
	USB_record_stop();
}
  
//-- radio buttons
void MyPlugin::radioB_joy(){
	test("Only joystick.");
}

void MyPlugin::radioB_autopilot(){
	test("Tum AutoPilot.");
}
void MyPlugin::radioB_joyAndAuto(){
	test("Joystick and AutoPilot. ()");
}

  
//****  TUM AR.Drone:
void MyPlugin::clickAddButton(){
	// kopiraj besedilo iz "lineEdit_command" in ga dodaj na seznam "Send commands"
	ui_.textEdit_sendC->append( ui_.lineEdit_command->text() );
}

void MyPlugin::clickClearButton(){
	// počisti buffer ukazov v vrsti autopilot
	std_msgs::String msg;
	msg.data="c clearCommands";
	pub_tum_commands.publish(msg);
	// tumClearC();
	
}

void MyPlugin::clickSendButton(){
	// pošlji ukaz/e iz seznama "Send commands"
	// ukaze se pošilja kot string...
	
	// tvorjenje sporočil:
	std_msgs::String msg;
	QString str = ui_.textEdit_sendC->toPlainText();
	QStringList list = str.split("\n");
	for(int q=0; q< list.size(); q++){
		msg.data = list.at(q).toUtf8().constData();
		pub_tum_commands.publish(msg);
	}
	msg.data = "c start";
	pub_tum_commands.publish(msg);
	//tumSendC();
}

void MyPlugin::clickResetButton(){
	// resetiraj PTAM.....
	std_msgs::String msg;
	msg.data="c stop";
	pub_tum_commands.publish(msg);
	msg.data="c clearCommands";
	pub_tum_commands.publish(msg);
	msg.data="f reset";
	pub_tum_commands.publish(msg);
	//tumResetC();
}

void MyPlugin::comboBoxCommand(QString str){
	ui_.lineEdit_command->setText(str);
	test(str);
	//odpri datoteko in prikaži še info o izbranem ukazu...
	str.replace(QString(" "), QString("_"));

	// predpostavlja, da se GUI zažene v direktoriju catkin_ws
	QString fileName = QString(QDir::currentPath()+"/src/ardrone_gui/TUM_ukazi/" + str + QString(".txt"));
	if(!fileName.isEmpty()){
		QFile file(fileName);
		if(!file.open(QIODevice::ReadOnly)){
			QMessageBox::critical(0, QString("Error"), QString("Could not open file: ")+fileName);
			ui_.label_zgradba_ukaza->setText(QString());
			ui_.textEdit_opis->setPlainText(QString());
			return;
		}
		QTextStream in(&file);
		ui_.label_zgradba_ukaza->setText(in.readLine());
		in.seek(0);
		ui_.textEdit_opis->setPlainText(in.readAll());
		//ui_.label_zgradba_ukaza->setText(in.readLine());
		file.close();
	}
}

void MyPlugin::openFileButton(){
	QString fileName = QFileDialog::getOpenFileName(0, tr("Open File"), QString(), tr("Text Files (*.txt);;All Files ()"));
	//QString fileName = QFileDialog::getOpenFileName(0, "Open File", "", "Text Files (*.txt)");
	
	if(!fileName.isEmpty()){
		QFile file(fileName);
		if(!file.open(QIODevice::ReadOnly)){
			QMessageBox::critical(0, "Error", "Could not open file");
			return;
		}
		QTextStream in(&file);
		ui_.textEdit_sendC->append(in.readAll());
		file.close();
	}
}

//-------------

void MyPlugin::test(QString niz){
	ui_.label_joy->setText(niz);
}

// navdata:
void MyPlugin::navdata_callback(const ardrone_autonomy::Navdata& nav_msg){  
	//KRMILJENJE:
	battUpdate.setValue(nav_msg.batteryPercent);
	state = nav_msg.state;

	switch(state){
		case 1: QMetaObject::invokeMethod(ui_.label_drone_state, "setText", Qt::DirectConnection, Q_ARG(QString, "Inited")); break;
		case 2: QMetaObject::invokeMethod(ui_.label_drone_state, "setText", Qt::DirectConnection, Q_ARG(QString, "Landed")); break;
		case 3:
		case 7: QMetaObject::invokeMethod(ui_.label_drone_state, "setText", Qt::DirectConnection, Q_ARG(QString, "Flying")); break;
		case 4: QMetaObject::invokeMethod(ui_.label_drone_state, "setText", Qt::DirectConnection, Q_ARG(QString, "Hovering")); break;
		case 5: QMetaObject::invokeMethod(ui_.label_drone_state, "setText", Qt::DirectConnection, Q_ARG(QString, "Test")); break;
		case 6: QMetaObject::invokeMethod(ui_.label_drone_state, "setText", Qt::DirectConnection, Q_ARG(QString, "Taking off")); break;
		case 8: QMetaObject::invokeMethod(ui_.label_drone_state, "setText", Qt::DirectConnection, Q_ARG(QString, "Landing")); break;
		case 9: QMetaObject::invokeMethod(ui_.label_drone_state, "setText", Qt::DirectConnection, Q_ARG(QString, "Looping")); break;
		case 0: QMetaObject::invokeMethod(ui_.label_drone_state, "setText", Qt::DirectConnection, Q_ARG(QString, "Emergency")); break;
		default : QMetaObject::invokeMethod(ui_.label_drone_state, "setText", Qt::DirectConnection, Q_ARG(QString, "Unknown")); break;
	}
	
	// NAVIGATION DATA:
	QMetaObject::invokeMethod(ui_.label_drone_altitude, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.altd)+" mm"));

	// zaradi različnih threadov ne moremo direktno spreminjati label
	//sprememba vsebine posamezne labele
	//QMetaObject::invokeMethod(ui_.label_, "setText", Qt::QueuedConnection, Q_ARG(QString, <željeni niz>);
	//std::cout << nav_msg << endl;
	QMetaObject::invokeMethod(ui_.label_bat, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.batteryPercent)+"%"));
	QMetaObject::invokeMethod(ui_.label_state, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.state)));
	QMetaObject::invokeMethod(ui_.label_alt, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.altd)+" mm"));	
	QMetaObject::invokeMethod(ui_.label_rotate_x, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.rotX, 'f', 4)));
	QMetaObject::invokeMethod(ui_.label_rotate_y, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.rotY, 'f', 4)));
	QMetaObject::invokeMethod(ui_.label_rotate_z, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.rotZ, 'f', 4)));
	QMetaObject::invokeMethod(ui_.label_magnet_x, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.magX, 'f', 4)));
	QMetaObject::invokeMethod(ui_.label_magnet_y, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.magY, 'f', 4)));
	QMetaObject::invokeMethod(ui_.label_magnet_z, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.magZ, 'f', 4)));
	QMetaObject::invokeMethod(ui_.label_pressure, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.pressure)));
	QMetaObject::invokeMethod(ui_.label_pressure, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.pressure)));
	QMetaObject::invokeMethod(ui_.label_temp, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.temp)));
	QMetaObject::invokeMethod(ui_.label_wind_speed, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.wind_speed, 'f', 4)));
	QMetaObject::invokeMethod(ui_.label_wind_angle, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.wind_angle, 'f', 4)));
	QMetaObject::invokeMethod(ui_.label_wind_comp, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.wind_comp_angle, 'f', 4)));
	QMetaObject::invokeMethod(ui_.label_linvel_x, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.vx, 'f', 4)));
	QMetaObject::invokeMethod(ui_.label_linvel_y, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.vy, 'f', 4)));
	QMetaObject::invokeMethod(ui_.label_linvel_z, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.vz, 'f', 4)));
	QMetaObject::invokeMethod(ui_.label_linacc_x, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.ax, 'f', 4)));
	QMetaObject::invokeMethod(ui_.label_linacc_y, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.ay, 'f', 4)));
	QMetaObject::invokeMethod(ui_.label_linacc_z, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.az, 'f', 4)));
	QMetaObject::invokeMethod(ui_.label_motor_1, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.motor1)));
	QMetaObject::invokeMethod(ui_.label_motor_2, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.motor2)));
	QMetaObject::invokeMethod(ui_.label_motor_3, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.motor3)));
	QMetaObject::invokeMethod(ui_.label_motor_4, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(nav_msg.motor4)));
}

// joy:

void MyPlugin::joy_callback(const sensor_msgs::Joy::ConstPtr& joy){
	// call back funkcija za joy-stik
	//	MyPlugin::test("joy");
	//MyPlugin::drone_emergency();
	
	if(ui_.radioB_tumAP->isChecked()){
		clickResetButton();
		ui_.radioB_onlyJ->setChecked(true);
		radioB_joy();
	}
	
	// scale and limit axes commands
	pitch_y = axes_scale * (float)joy->axes[1];
	roll_x = axes_scale * (float)joy->axes[0];
	yaw_z = axes_scale * (float)joy->axes[3];
	height_z = axes_scale * (float)joy->axes[4];
	
	if(pitch_y > axes_max) pitch_y = axes_max;
	else if(pitch_y < -axes_max) pitch_y = -axes_max;
	if(roll_x > axes_max) roll_x = axes_max;
	else if(roll_x < -axes_max) roll_x = -axes_max;
	if(yaw_z > axes_max) yaw_z = axes_max;
	else if(yaw_z < -axes_max) yaw_z = -axes_max;
	if(height_z > axes_max) height_z = axes_max;
	else if(height_z < -axes_max) height_z = -axes_max;
	
	// send axes commands
	last_send_vel.linear.x = pitch_y;
	last_send_vel.linear.y = roll_x;
	last_send_vel.linear.z = height_z;
	last_send_vel.angular.z = yaw_z;
	
	last_send_vel.angular.x = 0.0;
	last_send_vel.angular.y = 0.0;
	
	/*ui_.label_pitch_y->setText( QString::number(pitch_y,'f', 4) );
	ui_.label_roll_x->setText( QString::number(roll_x,'f', 4) );
	ui_.label_yaw_z->setText( QString::number(yaw_z,'f', 4) );
	ui_.label_hight_z->setText( QString::number(hight_z,'f', 4) );*/

	QMetaObject::invokeMethod(ui_.label_pitch_y, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(pitch_y, 'f', 4)));
	QMetaObject::invokeMethod(ui_.label_roll_x, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(roll_x, 'f', 4)));
	QMetaObject::invokeMethod(ui_.label_yaw_z, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(yaw_z, 'f', 4)));
	QMetaObject::invokeMethod(ui_.label_height_z, "setText", Qt::QueuedConnection, Q_ARG(QString, QString::number(height_z, 'f', 4)));
	
	// gamepad f510

	// 0
	if(joy->buttons[0] == 1){
		// gumb: A
		// efekt: emergency
		ui_.btn_0->setText("On");
		drone_emergency();
	}
	else if(joy->buttons[0] == 0){
		ui_.btn_0->setText("Off");
	}

	// 1
	if(joy->buttons[1] == 1){
		// prednja kamera
		ui_.btn_1->setText("On");
		//camera_select(0);
	}
	else if(joy->buttons[1] == 0){
		ui_.btn_1->setText("Off");
	}

	// 2
	if(joy->buttons[2] == 1){
		// spodnja kamera
		ui_.btn_2->setText("On");
		camera_select(1);
	}
	else if(joy->buttons[2] == 0){
		ui_.btn_2->setText("Off");
	}

	// 3
	if(joy->buttons[3] == 1){
		ui_.btn_3->setText("On");
	}
	else if(joy->buttons[3] == 0){
		ui_.btn_3->setText("Off");
	}

	// 4
	if(joy->buttons[4] == 1){
		ui_.lt_1->setText("On");
	}
	else if(joy->buttons[4] == 0){
		ui_.lt_1->setText("Off");
	}

	// 5
	if(joy->buttons[5] == 1){
		ui_.rt_1->setText("On");
	}
	else if(joy->buttons[5] == 0){
		ui_.rt_1->setText("Off");
	}

	// 6
	if(joy->buttons[6] == 1){
		// gumb: back
		// efekt: land
		//drone_land();
		ui_.lt_2->setText("On");
	}
	else if(joy->buttons[6] == 0){
		ui_.lt_2->setText("Off");
	}

	// 7
	if(joy->buttons[7] == 1){
		// gumb: start
		// efekt: take off
		//drone_take_off();
		ui_.rt_2->setText("On");
	}
	else if(joy->buttons[7] == 0){
		ui_.rt_2->setText("Off");
	}

	// 8
	if(joy->buttons[8] == 1){
		//ui_.lt_2->setText("On");
	}
	else if(joy->buttons[8] == 0){
		//ui_.lt_2->setText("Off");
	}

	// 9
	if(joy->buttons[9] == 1){
		//ui_.lt_2->setText("On");
	}
	else if(joy->buttons[9] == 0){
		//ui_.lt_2->setText("Off");
	}	
	
	// krizec
	// left-right
	if(joy->axes[0] == 1){
		ui_.btn_left->setText("On");
	}
	else if(joy->axes[0] == -1){
		ui_.btn_right->setText("On");
	}
	else {
		ui_.btn_left->setText("Off");
		ui_.btn_right->setText("Off");
	}

	// up-down
	if(joy->axes[1] == 1){
		ui_.btn_up->setText("On");
	}
	else if(joy->axes[1] == -1){
		ui_.btn_down->setText("On");
	}
	else {
		ui_.btn_up->setText("Off");
		ui_.btn_down->setText("Off");
	}

	// left analog
	// left-right
	if(joy->axes[4] == 1){
		ui_.al_left->setText("On");
	}
	else if(joy->axes[4] == -1){
		ui_.al_right->setText("On");
	}
	else {
		ui_.al_left->setText("Off");
		ui_.al_right->setText("Off");
	}
	// up-down
	// left-right
	if(joy->axes[5] == 1){
		ui_.al_up->setText("On");
	}
	else if(joy->axes[5] == -1){
		ui_.al_down->setText("On");
	}
	else {
		ui_.al_up->setText("Off");
		ui_.al_down->setText("Off");
	}

	// right analog
	// left-right
	if(joy->axes[2] == 1){
		ui_.ar_left->setText("On");
	}
	else if(joy->axes[2] == -1){
		ui_.ar_right->setText("On");
	}
	else {
		ui_.ar_left->setText("Off");
		ui_.ar_right->setText("Off");
	}
	// up-down
	if(joy->axes[3] == 1){
		ui_.ar_up->setText("On");
	}
	else if(joy->axes[3] == -1){
		ui_.ar_down->setText("On");
	}
	else {
		ui_.ar_up->setText("Off");
		ui_.ar_down->setText("Off");
	}
}

//-------------

void MyPlugin::drone_take_off(){
	// koda za vzlet: pošlje se sporočilo tipa std_msgs/Empty
	std_msgs::Empty msg;
	pub_take_off.publish(msg);
	test("Take off drone.");
}

void MyPlugin::drone_land(){
	//koda za pristanek: pošlje se sporočilo tipa std_msgs/Empty
	std_msgs::Empty msg;
	pub_land.publish(msg);
	test("Land drone.");
}

void MyPlugin::IMU_calibrate(){
	std_srvs::Empty req;
	//pošlje zahtevo tipa std_srvs/Empty
	if (ros::service::call("/ardrone/imu_recalib", req)){
		test("Calibrating IMU sensors.");
	}
	else{
		test("IMU calibration failed.");
	}
}

void MyPlugin::flat_trim(){
	std_srvs::Empty req;
	//pošlje zahtevo tipa std_srvs/Empty
	if (ros::service::call("/ardrone/flattrim", req)){
		test("Flat trim successful.");
	}
	else{
		test("Flat trim unsuccessful.");
	}
}

void MyPlugin::USB_record_start(){
	ardrone_autonomy::RecordEnable rec;
	rec.request.enable = 1;
	if (ros::service::call("/ardrone/setrecord", rec)){
		test("USB recording started");
	}
	else {
		test("USB record start failed.");
	}
}

void MyPlugin::USB_record_stop(){
	ardrone_autonomy::RecordEnable rec;
	rec.request.enable = 0;
	if (ros::service::call("/ardrone/setrecord", rec)){
		test("USB recording stopped");
	}
	else {
		test("USB record stop failed.");
	}
}

void MyPlugin::drone_emergency(){
	// koda za pristanek v sili (motors off): pošlje se sporočilo tipa std_msgs/Empty
	std_msgs::Empty msg;
	pub_emergency.publish(msg);
}

void MyPlugin::publish_vel(){
	joy_vel_.publish(last_send_vel);
}

void MyPlugin::camera_select(char cam){
	//if(cam != cam_sel){
		ardrone_autonomy::CamSelect new_cam;
		new_cam.request.channel = cam;
/*		if( cam_client.call(new_cam) ) cam_sel=cam;
		*/
		if( ros::service::call("ardrone/setcamchannel", new_cam) ) cam_sel=cam;
	//}
	switch(cam){
		case 0: test("Selected front camera."); break;
		case 1: test("Selected bottom camera."); break;
	}
}

//------------------------------------------------------------------------------

void MyPlugin::timerEvent(QTimerEvent *event){
	//ui_.label_joy->setText("ros spin...");
	ros::spinOnce();
	//ui_.label_joy->setText("timer");
	if(topics_ok == 1) publish_vel();
	//ui_.progressBar->setValue(battery);
}

void MyPlugin::spinbox_changed(double vrednost){
	//ui_.label_joy->setText("box_1");
	axes_max = (float)vrednost;
}

void MyPlugin::spinbox2_changed(double vrednost){
	//ui_.label_joy->setText("box_2");
	axes_scale = (float)vrednost;
}

// battery state signal emitter
void batterySignal::setValue(int value)
{
	emit valueChanged(value);
}

/*
void MyPlugin::clickoff(){
	//test
}
*/

//****

//****

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

} // namespace
//PLUGINLIB_DECLARE_CLASS(my_namespace, MyPlugin, my_namespace::MyPlugin, rqt_gui_cpp::Plugin)
PLUGINLIB_EXPORT_CLASS(my_namespace::MyPlugin, rqt_gui_cpp::Plugin)
