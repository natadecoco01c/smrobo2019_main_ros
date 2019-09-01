/*
 * mr1_main_semiauto.cpp
 *
 *  Created on: Feb 27, 2019
 *      Author: yuto
 *      Modifier: takeyabuyaketa
 */

#include <ros/ros.h>
#include <ros/duration.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Joy.h>
//#include <vector>
#include <string>

enum class fullOP
	: uint8_t
	{
	standby,
	start,
	sz_to_tp,
	launch,
	reload,
	tp_to_sz,
	release_reloader,
	wait_next_throw,

};

enum class BaseStatus
	: uint8_t
	{
		shutdown = 0x00, restart = 0x01,
};
enum class BaseCommands
	: uint8_t
	{
		disable = 0x00, enable = 0x01, homing = 0x10,
};

enum class LauncherReloaderCommands
	: uint8_t
	{
		disable = 0x00, enable = 0x01, homing = 0x10,
};

enum class LauncherTriggerCommands
	: uint8_t
	{
		disable = 0x00, launch = 0x01, enable = 0x10,
};

class pose {
public:
	double X;
	double Y;
	double YAW;
};

class Mr1Main {
public:
	Mr1Main(void);

private:
	void baseStatusCallback(const std_msgs::UInt8::ConstPtr &msg);
	void baseConfCallback(const std_msgs::UInt8::ConstPtr &msg);
	void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
//    void goalReachedCallback(const std_msgs::Bool::ConstPtr &msg);
	void control_timer_callback(const ros::TimerEvent &event);
	void slipper_loaded_callback(const std_msgs::UInt8::ConstPtr &msg);
	void GetYawCallback(const std_msgs::Float64::ConstPtr &msg);
	void TargetReachedCallback(const std_msgs::UInt8::ConstPtr &msg);
	void Steer0StatusCallback(const std_msgs::UInt8::ConstPtr &msg);
	void Steer1StatusCallback(const std_msgs::UInt8::ConstPtr &msg);
	void Steer2StatusCallback(const std_msgs::UInt8::ConstPtr &msg);
	void Steer3StatusCallback(const std_msgs::UInt8::ConstPtr &msg);
	void LauncherReloaderStatusCallback(const std_msgs::UInt8::ConstPtr &msg);

	ros::NodeHandle nh_;

	ros::Subscriber base_status_sub;
	ros::Publisher base_cmd_pub;
	std_msgs::UInt8 base_cmd_msg; //launchした時のエラーの原因はおそらくここ 後で下も含めて書き換える時直す

	ros::Subscriber base_odom_yaw_sub;
	double current_yaw = 0;
//	static constexpr double Kp = 0.5;
//	static constexpr double eps = 0.1;

	ros::Publisher launcher_cmd_pub; //cmdはbaseと分けました^^
	std_msgs::UInt8 launcher_cmd_msg;

	ros::Publisher launcher_reloader_cmd_pos_pub;
	std_msgs::Float32 launcher_reloader_cmd_pos_msg;

	ros::Publisher launcher_trigger_pub; //トリガー
	std_msgs::UInt8 launcher_trigger_msg;

	ros::Subscriber base_conf_sub;

	ros::Subscriber base_steer0_status_sub;
	ros::Subscriber base_steer1_status_sub;
	ros::Subscriber base_steer2_status_sub;
	ros::Subscriber base_steer3_status_sub;

	ros::Subscriber launcher_reloader_status_sub; //ランチャー下げる奴のstatus

	ros::Subscriber joy_sub;

	ros::Subscriber slipper_loaded_sub; //スリッパ検出かつスタンバイからのスタート

//  ros::Subscriber goal_reached_sub;
	ros::Publisher target_pub;
	ros::Publisher fine_target_pub;
	ros::Publisher abort_pub;
	ros::Publisher initialpose_pub;
	ros::Publisher cmd_vel_pub;
	ros::Publisher manual_pub;

	std_msgs::Bool abort_msg;
	std_msgs::Bool manual_msg;
	geometry_msgs::Twist cmd_vel_msg;

	ros::Publisher target_pos_pub;
	geometry_msgs::Twist target_pos_msg;
	ros::Subscriber target_reached_sub;

	pose start_zone;
	pose throwing_zone;

//	OpMode _op_mode;

	int _delay_s = 0;

	ros::Timer control_timer;

	void shutdown(void); //要らないと思う
	void start(void);
	void init(void);

	void reloader(float);
	void move_sz_tp(double move, double rot);
	void move_tp_sz(double move, double rot);
	double last_time;
	double current_velocity;
//	static constexpr double limit_vel = 1.0;
//	static constexpr double acc = 4.0;

	void trigger(uint8_t);
	double launcher_down = -11.4;

	void set_delay(double delay_s);

	uint8_t now_command = (uint8_t) fullOP::standby;

	bool operating = false;

	static constexpr uint8_t threshold = 10; //全てのモタドラから送られてきたbase_statusの1の数
	uint8_t status_counter_l = 0;
	uint8_t status_counter_0 = 0;
	uint8_t status_counter_1 = 0;
	uint8_t status_counter_2 = 0;
	uint8_t status_counter_3 = 0;
	bool first_time_restart = false; //ESshutdown後のrestart用 standbyに仕込む

	int slipper_detect_counter = 0;
	int slipper_detect_threshold = 40; //20Hzでくるらしいよ
	int sensor_0 = 0;
	int sensor_err_tolerance = 2; //センサがミスった時用

	bool steer0_status = true;
	bool steer1_status = true;
	bool steer2_status = true;
	bool steer3_status = true;
	bool launcher_status = true;

	bool once_started = false; //動作中にstart()がまたされないように

	uint8_t target_reached_count = 0;

	static int ButtonA;
	static int ButtonB;
	static int ButtonX;
	static int ButtonY;
	static int ButtonLB;
	static int ButtonRB;
	static int ButtonSelect;
	static int ButtonStart;
	static int ButtonLeftThumb;
	static int ButtonRightThumb;

	static int AxisDPadX;
	static int AxisDPadY;
	static int AxisLeftThumbX;
	static int AxisLeftThumbY;
	static int AxisRightThumbX;
	static int AxisRightThumbY;
	static int AxisLeftTrigger;
	static int AxisRightTrigger;

	bool _reload = false;
	bool _release = false;
	bool _trigger = false;

};

int Mr1Main::ButtonA = 0;
int Mr1Main::ButtonB = 1;
int Mr1Main::ButtonX = 2;
int Mr1Main::ButtonY = 3;
int Mr1Main::ButtonLB = 4;
int Mr1Main::ButtonRB = 5;
int Mr1Main::ButtonSelect = 6;
int Mr1Main::ButtonStart = 7;
int Mr1Main::ButtonLeftThumb = 9;
int Mr1Main::ButtonRightThumb = 10;

int Mr1Main::AxisDPadX = 0;
int Mr1Main::AxisDPadY = 1;
int Mr1Main::AxisLeftThumbX = 6;
int Mr1Main::AxisLeftThumbY = 7;
int Mr1Main::AxisRightThumbX = 3;
int Mr1Main::AxisRightThumbY = 4;
int Mr1Main::AxisLeftTrigger = 2;
int Mr1Main::AxisRightTrigger = 5;

Mr1Main::Mr1Main(void) {
	nh_.param("/main/start_zone_X", this->start_zone.X, 0.230);
	nh_.param("/main/start_zone_Y", this->start_zone.Y, 0.268);
	nh_.param("/main/start_zone_YAW", this->start_zone.YAW, 0.0);
	nh_.param("/main/throwing_zone_X", this->throwing_zone.X, 2.962);
	nh_.param("/main/throwing_zone_Y", this->throwing_zone.Y, 0.538);
	nh_.param("/main/throwing_zone_YAW", this->throwing_zone.YAW, 2.7);
	nh_.param("/main/slipper_detect_threshold", this->slipper_detect_threshold,
			40);
	nh_.param("/main/sensor_err_tolerance", this->sensor_err_tolerance, 2);

	ROS_INFO("start_zone_X : %lf", this->start_zone.X);
	ROS_INFO("start_zone_Y : %lf", this->start_zone.Y);
	ROS_INFO("start_zone_YAW : %lf", this->start_zone.YAW);
	ROS_INFO("throwing_zone_X : %lf", this->throwing_zone.X);
	ROS_INFO("throwing_zone_Y : %lf", this->throwing_zone.Y);
	ROS_INFO("throwing_zone_YAW : %lf", this->throwing_zone.YAW);
	ROS_INFO("slipper_detect_threshold : %d", this->slipper_detect_threshold);
	ROS_INFO("sensor_err_tolerance : %d", this->sensor_err_tolerance);

	nh_.param("/main/launcher_down", this->launcher_down, -11.4);
	ROS_INFO("launcher_down : %lf", this->launcher_down);

	this->base_status_sub = nh_.subscribe < std_msgs::UInt8
			> ("base/status", 10, &Mr1Main::baseStatusCallback, this);
	this->base_cmd_pub = nh_.advertise < std_msgs::UInt8 > ("base/cmd", 10); //UInt16->8

//	this->base_odom_yaw_sub = nh_.subscribe < std_msgs::Float64
//			> ("base/odom/yaw", 10, &Mr1Main::GetYawCallback, this);

	this->slipper_loaded_sub = nh_.subscribe < std_msgs::UInt8
			> ("slipper_loaded", 10, &Mr1Main::slipper_loaded_callback, this);

	this->launcher_cmd_pub = nh_.advertise < std_msgs::UInt8
			> ("launcher/cmd", 10);
	this->launcher_reloader_cmd_pos_pub = nh_.advertise < std_msgs::Float32
			> ("launcher_reloader_cmd_pos", 10);
	this->launcher_trigger_pub = nh_.advertise < std_msgs::UInt8
			> ("launcher_trigger", 10);

	this->base_conf_sub = nh_.subscribe < std_msgs::UInt8
			> ("base/conf", 10, &Mr1Main::baseConfCallback, this); //これ要る?

	this->joy_sub = nh_.subscribe < sensor_msgs::Joy
			> ("joy", 10, &Mr1Main::joyCallback, this);

	this->target_pos_pub = nh_.advertise < geometry_msgs::Twist
			> ("target_pos", 10, true);
	this->target_reached_sub = nh_.subscribe < std_msgs::UInt8
			> ("target_reached", 10, &Mr1Main::TargetReachedCallback, this);

	this->base_steer0_status_sub = nh_.subscribe < std_msgs::UInt8
			> ("steer0_status", 10, &Mr1Main::Steer0StatusCallback, this);
	this->base_steer1_status_sub = nh_.subscribe < std_msgs::UInt8
			> ("steer1_status", 10, &Mr1Main::Steer1StatusCallback, this);
	this->base_steer2_status_sub = nh_.subscribe < std_msgs::UInt8
			> ("steer2_status", 10, &Mr1Main::Steer2StatusCallback, this);
	this->base_steer3_status_sub = nh_.subscribe < std_msgs::UInt8
			> ("steer3_status", 10, &Mr1Main::Steer3StatusCallback, this);

	this->launcher_reloader_status_sub =
			nh_.subscribe < std_msgs::UInt8
					> ("launcher_reloader_status", 10, &Mr1Main::LauncherReloaderStatusCallback, this);

//	this->target_pub = nh_.advertise < nav_msgs::Path > ("target_path", 1);
//	this->fine_target_pub = nh_.advertise < nav_msgs::Path
//			> ("fine_target_path", 1);
//	this->abort_pub = nh_.advertise < std_msgs::Bool > ("abort", 1);
//	this->initialpose_pub = nh_.advertise
//			< geometry_msgs::PoseWithCovarianceStamped > ("/initialpose", 1);
	this->cmd_vel_pub = nh_.advertise < geometry_msgs::Twist > ("cmd_vel", 1);
	this->manual_pub = nh_.advertise < std_msgs::Bool > ("manual", 1);

	nh_.getParam("ButtonA", ButtonA);
	nh_.getParam("ButtonB", ButtonB);
	nh_.getParam("ButtonX", ButtonX);
	nh_.getParam("ButtonY", ButtonY);
	nh_.getParam("ButtonLB", ButtonLB);
	nh_.getParam("ButtonRB", ButtonRB);
	nh_.getParam("ButtonSelect", ButtonSelect);
	nh_.getParam("ButtonStart", ButtonStart);
	nh_.getParam("ButtonLeftThumb", ButtonLeftThumb);
	nh_.getParam("ButtonRightThumb", ButtonRightThumb);

	nh_.getParam("AxisLeftThumbX", AxisLeftThumbX);
	nh_.getParam("AxisLeftThumbY", AxisLeftThumbY);
	nh_.getParam("AxisRightThumbX", AxisRightThumbX);
	nh_.getParam("AxisRightThumbY", AxisRightThumbY);

	// timer starts immediately
	this->control_timer = nh_.createTimer(ros::Duration(0.05),
			&Mr1Main::control_timer_callback, this); //20Hz?
}

void Mr1Main::TargetReachedCallback(const std_msgs::UInt8::ConstPtr &msg) {
	ROS_INFO("reached");
	if (this->target_reached_count==0) {
		this->target_reached_count++;
		this->now_command = (uint8_t) fullOP::launch;
		this->operating = false;
	}else if (this->target_reached_count==1){

	}

}

//void Mr1Main::GetYawCallback(const std_msgs::Float64::ConstPtr &msg) {
//	if(!this->first_time_restart){
//		this->restart();
//		first_time_restart = true;
//	}
//	this->current_yaw = msg->data;
//}

void Mr1Main::baseStatusCallback(const std_msgs::UInt8::ConstPtr &msg) {
	ROS_INFO("restart signal detected");
	if ((BaseStatus) msg->data == BaseStatus::restart) {
		ros::shutdown();
	}

}

void Mr1Main::baseConfCallback(const std_msgs::UInt8::ConstPtr &msg) //雛形は残しときたい
		{
//	if (this->currentCommandIndex != -1 && this->currentCommandIndex != 0) {
//		return;
//	}
//
//	if (this->_op_mode != (OpMode) msg->data) {
//		this->_op_mode = (OpMode) msg->data;
//
//		if (this->_op_mode == OpMode::full_op) {
//			this->command_list = &Mr1Main::full_op_commands;
//			ROS_INFO("operation mode set to full_op.");
//		} else if (this->_op_mode == OpMode::move_test) {
//			this->command_list = &Mr1Main::move_test_commands;
//			ROS_INFO("operation mode set to move_test.");
//		} else if (this->_op_mode == OpMode::pickup_test) {
//			this->command_list = &Mr1Main::pickup_test_commands;
//			ROS_INFO("operation mode set to pickup_test.");
//		} else if (this->_op_mode == OpMode::throw_test) {
//			this->command_list = &Mr1Main::throw_test_commands;
//			ROS_INFO("operation mode set to throw_test.");
//		} else if (this->_op_mode == OpMode::pickup_and_throw_test) {
//			this->command_list = &Mr1Main::pickup_and_throw_test_commands;
//			ROS_INFO("operation mode set to pickup_and_throw_test.");
//		}
//	}
}

void Mr1Main::LauncherReloaderStatusCallback(
		const std_msgs::UInt8::ConstPtr &msg) {
//	if (msg->data == 1) {
//		this->launcher_status = true;
//		this->status_counter_l++;
//
//	} else {
//		this->launcher_status = false;
//		if (this->status_counter_l > this->threshold) {
//			ros::shutdown();
//		}
//
//	}
}
void Mr1Main::Steer0StatusCallback(const std_msgs::UInt8::ConstPtr &msg) {
//	if (msg->data == 1) {
//		this->steer0_status = true;
//		this->status_counter_0++;
//	} else {
//		this->steer0_status = false;
//		if (this->status_counter_0 > this->threshold) {
//			ros::shutdown();
//		}
//	}
}
void Mr1Main::Steer1StatusCallback(const std_msgs::UInt8::ConstPtr &msg) {
//	if (msg->data == 1) {
//		this->steer1_status = true;
//		this->status_counter_1++;
//	} else {
//		this->steer1_status = false;
//		if (this->status_counter_1 > this->threshold) {
//
//			ros::shutdown();
//		}
//	}
}
void Mr1Main::Steer2StatusCallback(const std_msgs::UInt8::ConstPtr &msg) {
//
//	if (msg->data == 1) {
//		this->steer2_status = true;
//		this->status_counter_2++;
//	} else {
//		this->steer2_status = false;
//		if (this->status_counter_2 > this->threshold) {
//
//			ros::shutdown();
//		}
//	}
}
void Mr1Main::Steer3StatusCallback(const std_msgs::UInt8::ConstPtr &msg) {
//	if (msg->data == 1) {
//		this->steer3_status = true;
//		this->status_counter_3++;
//	} else {
//		this->steer3_status = false;
//		if (this->status_counter_3 > this->threshold) {
//
//			ros::shutdown();
//		}
//	}
}

void Mr1Main::slipper_loaded_callback(const std_msgs::UInt8::ConstPtr &msg) { //連続で1が41回以上来るとstart //0が2回連続でカウントリセット 初回のみ非接点スイッチの役割を兼ねる
	if (msg->data == 0x01) {
		this->slipper_detect_counter++;
		this->sensor_0 = 0;
		if (this->slipper_detect_counter > this->slipper_detect_threshold
				&& !this->once_started) {
			this->now_command = (uint8_t) fullOP::start;
			this->once_started = true;
			this->slipper_detect_counter = 0;
		} else {
			if (this->sensor_0 > this->sensor_err_tolerance) {
				this->slipper_detect_counter = 0;
				this->sensor_0 = 0;
			}
			this->sensor_0++;
		}
	}
}

void Mr1Main::control_timer_callback(const ros::TimerEvent &event) { //最初はlauncherは下がった状態　戻す奴は上
	if (this->now_command == (uint8_t) fullOP::standby && !this->operating) { //フラグつけないと何度も呼び出しそう
		//ROS_INFO("standby");
		if (!this->first_time_restart) { //起動時一回のみ
			this->operating = true;
			ros::Duration duration(1);
			duration.sleep();
			this->init();
			this->start(); //各種enable
			first_time_restart = true;
			this->operating = false;
		}
	} else if (this->now_command == (uint8_t) fullOP::start //now_commandをstartにしてもらうのは外部から
	&& !this->operating) {
		this->operating = true;
		this->now_command = (uint8_t) fullOP::sz_to_tp;
		this->operating = false;
	} else if (this->now_command == (uint8_t) fullOP::sz_to_tp
			&& !this->operating) {
		ROS_INFO("sz_to_tp");
		this->operating = true; //falseはcallbackで
		target_pos_msg.linear.x = this->throwing_zone.X;
		target_pos_msg.linear.y = this->throwing_zone.Y;
		target_pos_msg.angular.z = this->throwing_zone.YAW;
		target_pos_pub.publish(target_pos_msg);
		this->now_command = (uint8_t) fullOP::standby; // this->operatingはtarget_reached_callbackで now_commandはtpで待機するためにstandbyに
		ROS_INFO("target pos pub");
		//launchにするのはtargetreachedcallbackの中
		//なんか動かす奴
	} else if (this->now_command == (uint8_t) fullOP::launch
			&& !this->operating) {
		this->operating = true;
		ROS_INFO("launch");
		ros::Duration ranchi(1);
		this->trigger(0x01);
		ranchi.sleep();
		this->trigger(0x00); //電磁弁を切る
		this->now_command = (uint8_t) fullOP::reload;
		this->operating = false;
	} else if (this->now_command == (uint8_t) fullOP::reload
			&& !this->operating) {
		ros::Duration duration(3);
		this->operating = true;
		target_pos_msg.linear.x = this->start_zone.X;
		target_pos_msg.linear.y = this->start_zone.Y;
		target_pos_msg.angular.z = this->start_zone.YAW;
		target_pos_pub.publish(target_pos_msg);
		this->reloader(this->launcher_down); //下げる
		duration.sleep(); //リロードちゃんとできるまで待つ
		this->now_command = (uint8_t) fullOP::release_reloader;
		this->operating = false;
	}
//	else if (this->now_command == (uint8_t) fullOP::tp_to_sz
//			&& !this->operating) { //要らない子
//		this->operating = true;
//		target_pos_msg.linear.x = this->start_zone.X;
//		target_pos_msg.linear.y = this->start_zone.Y;
//		target_pos_msg.angular.z = this->start_zone.YAW;
//		target_pos_pub.publish(target_pos_msg);
//		ROS_INFO("tptosz");
//		//なんか動かす奴
	else if (this->now_command == (uint8_t) fullOP::release_reloader
			&& !this->operating) {
		this->operating = true;
		this->reloader(0.0); //発射の邪魔なので戻す
		this->now_command = (uint8_t) fullOP::wait_next_throw;
		this->operating = false;
	} else if (this->now_command == (uint8_t) fullOP::wait_next_throw
			&& !this->operating && this->target_reached_count==2) {
		this->operating = true;
		this->trigger(0x10); //電磁弁enable
		this->target_reached_count=0;
		this->now_command = (uint8_t) fullOP::standby;
		this->operating = false;
	}
}

void Mr1Main::joyCallback(const sensor_msgs::Joy::ConstPtr &joy) {
	static bool last_a = false;
	static bool last_b = false;
	static bool last_x = false;
	static bool last_y = false;
	static bool last_start = false;

	bool _a = joy->buttons[ButtonA];
	bool _b = joy->buttons[ButtonB];
	bool _x = joy->buttons[ButtonX];
	bool _y = joy->buttons[ButtonY];

	bool _start = joy->buttons[ButtonStart];

//	if (_start) {
//		this->shutdown();
//	}
	if (_a && !last_a) {
		ROS_INFO("a");
		launcher_reloader_cmd_pos_msg.data = this->launcher_down;
		launcher_reloader_cmd_pos_pub.publish(launcher_reloader_cmd_pos_msg);
	} else if (_b && !last_b) {
		ROS_INFO("b");
		launcher_reloader_cmd_pos_msg.data = 0;
		launcher_reloader_cmd_pos_pub.publish(launcher_reloader_cmd_pos_msg);
	} else if (_x && !last_x) {
		launcher_trigger_msg.data = 0x01;
		launcher_trigger_pub.publish(launcher_trigger_msg);
	} else if (_y && !last_y) {
		ROS_INFO("reset");
		ros::Duration duration(0.00003);
		base_cmd_msg.data = (uint8_t) BaseCommands::enable;
		base_cmd_pub.publish(base_cmd_msg);
		duration.sleep();
		launcher_cmd_msg.data = (uint8_t) LauncherReloaderCommands::enable;
		launcher_cmd_pub.publish(launcher_cmd_msg);
		duration.sleep();
		this->trigger(0x10);
	} else if (_start && !last_start) {
		ROS_INFO("start_botan");
		this->start();
	}

	last_a = _a;
	last_b = _b;
	last_x = _x;
	last_y = _y;

//	if (this->_is_manual_enabled) {
//		double vel_x = joy->axes[AxisRightThumbX];
//		double vel_y = joy->axes[AxisRightThumbY];
//		double vel_yaw_l = (joy->axes[AxisLeftTrigger] - 1.0) * (1.0 - 0.0)
//				/ (-1.0 - 1.0) + 0.0;
//		double vel_yaw_r = (joy->axes[AxisRightTrigger] - 1.0) * (-1.0 - 0.0)
//				/ (-1.0 - 1.0) + 0.0;
//		double vel_yaw = vel_yaw_l + vel_yaw_r;
//
//		double vel_norm = hypot(vel_x, vel_y);
//		if (vel_norm > 1.0) {
//			vel_x /= vel_norm;
//			vel_y /= vel_norm;
//		}
//
//		this->cmd_vel_msg.linear.x = -vel_x;
//		this->cmd_vel_msg.linear.y = vel_y;
//		this->cmd_vel_msg.angular.z = vel_yaw;
//		this->cmd_vel_pub.publish(this->cmd_vel_msg);
//	}
}

void Mr1Main::start(void) {
	ROS_INFO("start.");
	ros::Duration duration(0.00003);
	for (int i = 0; i < 10; i++) {
		base_cmd_msg.data = ((uint8_t) BaseCommands::enable);
		base_cmd_pub.publish(base_cmd_msg);
		duration.sleep();
		launcher_cmd_msg.data = (uint8_t) LauncherReloaderCommands::enable;
		launcher_cmd_pub.publish(launcher_cmd_msg);
		duration.sleep();
		this->trigger(0x10); //enable
	}
}
void Mr1Main::reloader(float data) {
	this->operating = true;
	launcher_reloader_cmd_pos_msg.data = data;
	launcher_reloader_cmd_pos_pub.publish(launcher_reloader_cmd_pos_msg);
	this->operating = false;
}

void Mr1Main::init(void) {

	ROS_INFO("initializing.");
	ros::Duration duration(0.00003);
	for (int i = 0; i < 10; i++) {
		base_cmd_msg.data = (uint8_t) BaseCommands::disable;
		base_cmd_pub.publish(base_cmd_msg);
		duration.sleep();
		launcher_cmd_msg.data = (uint8_t) LauncherReloaderCommands::disable;
		launcher_cmd_pub.publish(launcher_cmd_msg);
		duration.sleep();
		launcher_trigger_msg.data = (uint8_t) LauncherTriggerCommands::disable;
		launcher_trigger_pub.publish(launcher_trigger_msg);
		duration.sleep();
		base_cmd_msg.data = (uint8_t) BaseCommands::homing;
		base_cmd_pub.publish(base_cmd_msg);
		duration.sleep();
		launcher_cmd_msg.data = (uint8_t) LauncherReloaderCommands::homing;
		launcher_cmd_pub.publish(launcher_cmd_msg);
	}

}

void Mr1Main::trigger(uint8_t data) {
	launcher_trigger_msg.data = data;
	launcher_trigger_pub.publish(launcher_trigger_msg);
}

//void Mr1Main::move_tp_sz(double move, double rot) {
//
//	this->operating = true;
//	ros::Rate loop_rate(200);
//	double dt = 1.0 / 200;
//
//	for (; fabs(rot - this->current_yaw) > this->eps;) {
//		cmd_vel_msg.angular.z = this->Kp * (rot - current_yaw);
//		cmd_vel_pub.publish(cmd_vel_msg);
//	}
//
//	double moved = 0;
//	double acc_moved = 0;
//	for (; -this->current_velocity < this->limit_vel;) {
//		moved += this->current_velocity * dt;
//		this->current_velocity += -this->acc * dt;
//		cmd_vel_msg.linear.x = current_velocity;
//		cmd_vel_pub.publish(cmd_vel_msg);
//		loop_rate.sleep();
//	}
//	acc_moved = moved;
//	current_velocity = -this->limit_vel;
//	cmd_vel_msg.linear.x = current_velocity;
//
//	for (; fabs(moved) < fabs(move - acc_moved);) {
//		moved += current_velocity * dt;
//		cmd_vel_pub.publish(cmd_vel_msg);
//		loop_rate.sleep();
//	}
//
//	for (; -this->current_velocity > 0;) {
//		moved += this->current_velocity * dt;
//		this->current_velocity -= -this->acc * dt; //減速
//		cmd_vel_msg.linear.x = current_velocity;
//		cmd_vel_pub.publish(cmd_vel_msg);
//		loop_rate.sleep();
//	}
//	current_velocity = 0;
//	cmd_vel_msg.linear.x = current_velocity;
//	cmd_vel_pub.publish(cmd_vel_msg);
//	ROS_INFO("moved %lf", moved);
//
//	this->operating = false;
//}
//
//void Mr1Main::move_sz_tp(double move, double rot) {
//
//	this->operating = true;
//	ros::Rate loop_rate(200);
//	double dt = 1.0 / 200;
//
//	double moved = 0;
//	double acc_moved = 0;
//	for (; this->current_velocity < this->limit_vel;) {
//		moved += this->current_velocity * dt;
//		this->current_velocity += this->acc * dt;
//		cmd_vel_msg.linear.x = current_velocity;
//		cmd_vel_pub.publish(cmd_vel_msg);
//		ROS_INFO("dt %lf", dt);
//		ROS_INFO("current_vel %lf", current_velocity);
//		loop_rate.sleep();
//	}
//	ROS_INFO("kasokuowari");
//	acc_moved = moved;
//	ROS_INFO("acc_moved %lf", acc_moved);
//	current_velocity = this->limit_vel;
//	cmd_vel_msg.linear.x = current_velocity;
//
//	for (; moved < (move - acc_moved);) {
//		ROS_INFO("%lf", dt);
//		moved += current_velocity * dt;
//		cmd_vel_pub.publish(cmd_vel_msg);
//		ROS_INFO("current_vel %lf", current_velocity);
//		ROS_INFO("nokori %lf", (move - acc_moved) - moved);
//		loop_rate.sleep();
//	}
//	ROS_INFO("saikousoku");
//
//	for (; this->current_velocity > 0;) {
//		//ROS_INFO("current_vel %lf", current_velocity);
//		moved += this->current_velocity * dt;
//		this->current_velocity -= this->acc * dt; //減速
//		cmd_vel_msg.linear.x = current_velocity;
//		cmd_vel_pub.publish(cmd_vel_msg);
//		ROS_INFO("current_vel %lf", current_velocity);
//		loop_rate.sleep();
//	}
//	ROS_INFO("gensokuowari");
//	current_velocity = 0;
//	cmd_vel_msg.linear.x = current_velocity;
//	cmd_vel_pub.publish(cmd_vel_msg);
//	ROS_INFO("moved %lf", moved);
//
//	for (; fabs(rot - this->current_yaw) > this->eps;) {
//		cmd_vel_msg.angular.z = this->Kp * (rot - current_yaw);
//		cmd_vel_pub.publish(cmd_vel_msg);
//		//ROS_INFO("PID");
//		loop_rate.sleep();
//	}
//
//	//ROS_INFO("change flag release_reloader");
//
//	this->operating = false;
//	ROS_INFO("operating false");
//}

int main(int argc, char **argv) {
	ros::init(argc, argv, "mr1_main");

	Mr1Main *instance = new Mr1Main();
	ROS_INFO("MR1 main node has started.");

	ros::spin();
	ROS_INFO("MR1 main node has been terminated.");
}
