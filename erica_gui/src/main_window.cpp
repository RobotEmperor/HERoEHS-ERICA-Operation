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
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    qnode.init();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}


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


/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "erica_gui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "erica_gui");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace erica_gui

