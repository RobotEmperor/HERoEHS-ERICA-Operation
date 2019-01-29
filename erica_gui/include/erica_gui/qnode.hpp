/**
 * @file /include/erica_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef erica_gui_QNODE_HPP_
#define erica_gui_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <string>
#include <QThread>
#include <QStringListModel>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "erica_arm_module_msgs/ArmCmd.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include <std_msgs/Int32.h>
#include "erica_perception_msgs/PeoplePositionArray.h"
#include "geometry_msgs/Point32.h"
#endif


/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace erica_gui {

/*****************************************************************************
 ** Class
 *****************************************************************************/

class QNode : public QThread {
	Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	 ** Logging
	 **********************/
	enum LogLevel {
		Debug,
		Info,
		Warn,
		Error,
		Fatal
	};

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

	//ros communication

	ros::Publisher enable_module_pub;
	ros::Publisher init_pose_pub;

	ros::Publisher arm_displacement_pub;
	ros::Publisher script_number_pub;


	ros::Subscriber status_sub;
	ros::Publisher g_people_position_pub;

	ros::Publisher head_gui_motor_publisher;
	ros::Publisher head_gui_manual_publisher;
	ros::Publisher head_gui_tracking_publisher;


	std_msgs::String enable_module_msg;
	std_msgs::String init_pose_msg;

	erica_arm_module_msgs::ArmCmd arm_displacement_msg;

	std_msgs::Int32 script_number_msg;

	erica_perception_msgs::PeoplePositionArray people_position_msg;
	geometry_msgs::Point32 temp_people_position;

	std_msgs::Float64MultiArray motor_rad_value;
	std_msgs::Bool enable_manual;
	std_msgs::Bool enable_tracking;

	//callback
	void StatusMsgsCallBack(const robotis_controller_msgs::StatusMsg::ConstPtr& msg);

	Q_SIGNALS:
	void loggingUpdated();
	void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
	QStringListModel logging_model;
};

}  // namespace erica_gui

#endif /* erica_gui_QNODE_HPP_ */
