/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/erica_gui/main_window.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace erica_gui {

using namespace Qt;

/*****************************************************************************
 ** Implementation [MainWindow]
 *****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
: QMainWindow(parent)
, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	 ** Logging
	 **********************/
	/*********************
	 ** Auto Start
	 **********************/
	qnode.init();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
 ** Implementation [Slots]
 *****************************************************************************/



/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */





/*****************************************************************************
 ** Implemenation [Slots][manually connected]
 *****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */


/*****************************************************************************
 ** Implementation [Menu]
 *****************************************************************************/



//ros communication
void MainWindow::on_base_module_clicked()
{
	qnode.enable_module_msg.data = "base_module";
	qnode.enable_module_pub.publish(qnode.enable_module_msg);
}

void MainWindow::on_arm_module_clicked()
{
	qnode.arm_displacement_msg.name = "left";
	qnode.enable_module_msg.data = "arm_module";
	qnode.enable_module_pub.publish(qnode.enable_module_msg);
}
void MainWindow::on_head_module_clicked()
{
	qnode.enable_module_msg.data = "head_module";
	qnode.enable_module_pub.publish(qnode.enable_module_msg);
}
void MainWindow::on_action_module_clicked()
{
	qnode.enable_module_msg.data = "action_module";
	qnode.enable_module_pub.publish(qnode.enable_module_msg);
}
void MainWindow::on_none_module_clicked()
{
	qnode.enable_module_msg.data = "none";
	qnode.enable_module_pub.publish(qnode.enable_module_msg);
}

void MainWindow::on_initial_pose_clicked()
{
	qnode.init_pose_msg.data = "init_pose";
	qnode.init_pose_pub.publish(qnode.init_pose_msg);

}

//manual control

void MainWindow::on_x_plus_clicked()
{
	qnode.arm_displacement_msg.name = "left";
	qnode.arm_displacement_msg.pose.position.x = 0.001;
	qnode.arm_displacement_msg.pose.position.y = 0;
	qnode.arm_displacement_msg.pose.position.z = 0;

	Eigen::Quaterniond rqyToQ;

	rqyToQ = robotis_framework::convertRPYToQuaternion(0,0,0);

	qnode.arm_displacement_msg.pose.orientation.x = rqyToQ.x();
	qnode.arm_displacement_msg.pose.orientation.y = rqyToQ.y();
	qnode.arm_displacement_msg.pose.orientation.z = rqyToQ.z();
	qnode.arm_displacement_msg.pose.orientation.w = rqyToQ.w();

	qnode.arm_displacement_pub.publish(qnode.arm_displacement_msg);
}
void MainWindow::on_x_minus_clicked()
{
	qnode.arm_displacement_msg.pose.position.x = -0.001;
	qnode.arm_displacement_msg.pose.position.y = 0;
	qnode.arm_displacement_msg.pose.position.z = 0;

	Eigen::Quaterniond rqyToQ;

	rqyToQ = robotis_framework::convertRPYToQuaternion(0,0,0);

	qnode.arm_displacement_msg.pose.orientation.x = rqyToQ.x();
	qnode.arm_displacement_msg.pose.orientation.y = rqyToQ.y();
	qnode.arm_displacement_msg.pose.orientation.z = rqyToQ.z();
	qnode.arm_displacement_msg.pose.orientation.w = rqyToQ.w();
	qnode.arm_displacement_pub.publish(qnode.arm_displacement_msg);
}

void MainWindow::on_y_plus_clicked()
{
	qnode.arm_displacement_msg.pose.position.x = 0;
	qnode.arm_displacement_msg.pose.position.y = 0.001;
	qnode.arm_displacement_msg.pose.position.z = 0;

	Eigen::Quaterniond rqyToQ;

	rqyToQ = robotis_framework::convertRPYToQuaternion(0,0,0);

	qnode.arm_displacement_msg.pose.orientation.x = rqyToQ.x();
	qnode.arm_displacement_msg.pose.orientation.y = rqyToQ.y();
	qnode.arm_displacement_msg.pose.orientation.z = rqyToQ.z();
	qnode.arm_displacement_msg.pose.orientation.w = rqyToQ.w();
	qnode.arm_displacement_pub.publish(qnode.arm_displacement_msg);
}
void MainWindow::on_y_minus_clicked()
{
	qnode.arm_displacement_msg.pose.position.x = 0;
	qnode.arm_displacement_msg.pose.position.y = -0.001;
	qnode.arm_displacement_msg.pose.position.z = 0;

	Eigen::Quaterniond rqyToQ;

	rqyToQ = robotis_framework::convertRPYToQuaternion(0,0,0);

	qnode.arm_displacement_msg.pose.orientation.x = rqyToQ.x();
	qnode.arm_displacement_msg.pose.orientation.y = rqyToQ.y();
	qnode.arm_displacement_msg.pose.orientation.z = rqyToQ.z();
	qnode.arm_displacement_msg.pose.orientation.w = rqyToQ.w();
	qnode.arm_displacement_pub.publish(qnode.arm_displacement_msg);
}

void MainWindow::on_z_plus_clicked()
{
	qnode.arm_displacement_msg.pose.position.x = 0;
	qnode.arm_displacement_msg.pose.position.y = 0;
	qnode.arm_displacement_msg.pose.position.z = 0.001;

	Eigen::Quaterniond rqyToQ;

	rqyToQ = robotis_framework::convertRPYToQuaternion(0,0,0);

	qnode.arm_displacement_msg.pose.orientation.x = rqyToQ.x();
	qnode.arm_displacement_msg.pose.orientation.y = rqyToQ.y();
	qnode.arm_displacement_msg.pose.orientation.z = rqyToQ.z();
	qnode.arm_displacement_msg.pose.orientation.w = rqyToQ.w();
	qnode.arm_displacement_pub.publish(qnode.arm_displacement_msg);
}
void MainWindow::on_z_minus_clicked()
{
	qnode.arm_displacement_msg.pose.position.x = 0;
	qnode.arm_displacement_msg.pose.position.y = 0;
	qnode.arm_displacement_msg.pose.position.z = -0.001;

	Eigen::Quaterniond rqyToQ;

	rqyToQ = robotis_framework::convertRPYToQuaternion(0,0,0);

	qnode.arm_displacement_msg.pose.orientation.x = rqyToQ.x();
	qnode.arm_displacement_msg.pose.orientation.y = rqyToQ.y();
	qnode.arm_displacement_msg.pose.orientation.z = rqyToQ.z();
	qnode.arm_displacement_msg.pose.orientation.w = rqyToQ.w();
	qnode.arm_displacement_pub.publish(qnode.arm_displacement_msg);
}

void MainWindow::on_roll_plus_clicked()
{
	qnode.arm_displacement_msg.pose.position.x = 0;
	qnode.arm_displacement_msg.pose.position.y = 0;
	qnode.arm_displacement_msg.pose.position.z = 0;

	Eigen::Quaterniond rqyToQ;

	rqyToQ = robotis_framework::convertRPYToQuaternion(0.01,0,0);

	qnode.arm_displacement_msg.pose.orientation.x = rqyToQ.x();
	qnode.arm_displacement_msg.pose.orientation.y = rqyToQ.y();
	qnode.arm_displacement_msg.pose.orientation.z = rqyToQ.z();
	qnode.arm_displacement_msg.pose.orientation.w = rqyToQ.w();
	qnode.arm_displacement_pub.publish(qnode.arm_displacement_msg);
}
void MainWindow::on_roll_minus_clicked()
{
	qnode.arm_displacement_msg.pose.position.x = 0;
	qnode.arm_displacement_msg.pose.position.y = 0;
	qnode.arm_displacement_msg.pose.position.z = 0;

	Eigen::Quaterniond rqyToQ;

	rqyToQ = robotis_framework::convertRPYToQuaternion(-0.01,0,0);

	qnode.arm_displacement_msg.pose.orientation.x = rqyToQ.x();
	qnode.arm_displacement_msg.pose.orientation.y = rqyToQ.y();
	qnode.arm_displacement_msg.pose.orientation.z = rqyToQ.z();
	qnode.arm_displacement_msg.pose.orientation.w = rqyToQ.w();
	qnode.arm_displacement_pub.publish(qnode.arm_displacement_msg);
}

void MainWindow::on_pitch_plus_clicked()
{
	qnode.arm_displacement_msg.pose.position.x = 0;
	qnode.arm_displacement_msg.pose.position.y = 0;
	qnode.arm_displacement_msg.pose.position.z = 0;

	Eigen::Quaterniond rqyToQ;

	rqyToQ = robotis_framework::convertRPYToQuaternion(0,0.01,0);

	qnode.arm_displacement_msg.pose.orientation.x = rqyToQ.x();
	qnode.arm_displacement_msg.pose.orientation.y = rqyToQ.y();
	qnode.arm_displacement_msg.pose.orientation.z = rqyToQ.z();
	qnode.arm_displacement_msg.pose.orientation.w = rqyToQ.w();
	qnode.arm_displacement_pub.publish(qnode.arm_displacement_msg);
}
void MainWindow::on_pitch_minus_clicked()
{
	qnode.arm_displacement_msg.pose.position.x = 0;
	qnode.arm_displacement_msg.pose.position.y = 0;
	qnode.arm_displacement_msg.pose.position.z = 0;

	Eigen::Quaterniond rqyToQ;

	rqyToQ = robotis_framework::convertRPYToQuaternion(0,-0.01,0);

	qnode.arm_displacement_msg.pose.orientation.x = rqyToQ.x();
	qnode.arm_displacement_msg.pose.orientation.y = rqyToQ.y();
	qnode.arm_displacement_msg.pose.orientation.z = rqyToQ.z();
	qnode.arm_displacement_msg.pose.orientation.w = rqyToQ.w();
	qnode.arm_displacement_pub.publish(qnode.arm_displacement_msg);
}

void MainWindow::on_yaw_plus_clicked()
{
	qnode.arm_displacement_msg.pose.position.x = 0;
	qnode.arm_displacement_msg.pose.position.y = 0;
	qnode.arm_displacement_msg.pose.position.z = 0;

	Eigen::Quaterniond rqyToQ;

	rqyToQ = robotis_framework::convertRPYToQuaternion(0,0,0.01);

	qnode.arm_displacement_msg.pose.orientation.x = rqyToQ.x();
	qnode.arm_displacement_msg.pose.orientation.y = rqyToQ.y();
	qnode.arm_displacement_msg.pose.orientation.z = rqyToQ.z();
	qnode.arm_displacement_msg.pose.orientation.w = rqyToQ.w();
	qnode.arm_displacement_pub.publish(qnode.arm_displacement_msg);
}
void MainWindow::on_yaw_minus_clicked()
{
	qnode.arm_displacement_msg.pose.position.x = 0;
	qnode.arm_displacement_msg.pose.position.y = 0;
	qnode.arm_displacement_msg.pose.position.z = 0;

	Eigen::Quaterniond rqyToQ;

	rqyToQ = robotis_framework::convertRPYToQuaternion(0,0,-0.01);

	qnode.arm_displacement_msg.pose.orientation.x = rqyToQ.x();
	qnode.arm_displacement_msg.pose.orientation.y = rqyToQ.y();
	qnode.arm_displacement_msg.pose.orientation.z = rqyToQ.z();
	qnode.arm_displacement_msg.pose.orientation.w = rqyToQ.w();
	qnode.arm_displacement_pub.publish(qnode.arm_displacement_msg);
}



// head module
void MainWindow::on_button_manual_send_clicked() {

	//excutable


	QString text1 = ui.head_yaw->toPlainText();
	double s_text1 = text1.toFloat();
	QString text2 = ui.head_pitch->toPlainText();
	double s_text2 = text2.toFloat();
	QString text3 = ui.head_roll->toPlainText();
	double s_text3 = text3.toFloat();
	qnode.motor_rad_value.data.clear();
	qnode.motor_rad_value.data.push_back(s_text1);
	qnode.motor_rad_value.data.push_back(s_text2);
	qnode.motor_rad_value.data.push_back(s_text3);
	qnode.head_gui_motor_publisher.publish(qnode.motor_rad_value);

}

void MainWindow::on_button_manual_clicked() {
	qnode.enable_manual.data=true;
	qnode.head_gui_manual_publisher.publish(qnode.enable_manual);
}
void MainWindow::on_button_tracking_clicked() {
	qnode.enable_tracking.data=true;
	qnode.head_gui_tracking_publisher.publish(qnode.enable_tracking);
}




void MainWindow::on_motion_num_send_button_clicked()
{

	QString script_number_str;
	int  script_number_int = 0;


	script_number_str = ui.motion_num_line_edit->text();
	script_number_int = script_number_str.toInt();

	qnode.script_number_msg.data = script_number_int;
	qnode.script_number_pub.publish(qnode.script_number_msg);
}
void MainWindow::on_dummy_send_button_clicked()
{
	savePeoplePosition(ui.people_x1_line_edit->text(), ui.people_y1_line_edit->text(), ui.people_z1_line_edit->text());
	savePeoplePosition(ui.people_x2_line_edit->text(), ui.people_y2_line_edit->text(), ui.people_z2_line_edit->text());
	savePeoplePosition(ui.people_x3_line_edit->text(), ui.people_y3_line_edit->text(), ui.people_z3_line_edit->text());
	savePeoplePosition(ui.people_x4_line_edit->text(), ui.people_y4_line_edit->text(), ui.people_z4_line_edit->text());

	qnode.g_people_position_pub.publish(qnode.people_position_msg);
	qnode.people_position_msg.people_position.clear();
}


/*****************************************************************************
 ** Implementation [Configuration]
 *****************************************************************************/

void MainWindow::ReadSettings() {

}

void MainWindow::WriteSettings() {


}
void MainWindow::savePeoplePosition(QString x, QString y, QString z)
{
	QString people_position_x_str;
	double  people_position_x_double = 0;

	QString people_position_y_str;
	double  people_position_y_double = 0;

	QString people_position_z_str;
	double  people_position_z_double = 0;

	people_position_x_str = x;
	people_position_x_double = people_position_x_str.toDouble();

	people_position_y_str = y;
	people_position_y_double = people_position_y_str.toDouble();

	people_position_z_str = z;
	people_position_z_double = people_position_z_str.toDouble();

	qnode.temp_people_position.x = (float) people_position_x_double;
	qnode.temp_people_position.y = (float) people_position_y_double;
	qnode.temp_people_position.z = (float) people_position_z_double;

	if(pow(people_position_x_double,2) + pow(people_position_y_double,2) + pow(people_position_z_double,2) != 0 )
	{

		qnode.people_position_msg.people_position.push_back(qnode.temp_people_position);

	}
	else
		return;

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace erica_gui

