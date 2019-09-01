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
//#include <vector> //使いこなせませんでしたごめんなさい
#include <string>

enum class fullOP : uint8_t {
	standby, start, sz_to_tp, launch, reload, release_reloader, wait_next_throw,

};

enum class BaseStatus : uint8_t {
	shutdown = 0x00, restart = 0x01,
};
enum class BaseCommands : uint8_t {
	disable = 0x00, enable = 0x01, homing = 0x10,
};

enum class LauncherReloaderCommands : uint8_t {
	disable = 0x00, enable = 0x01, homing = 0x10,
};

enum class LauncherTriggerCommands : uint8_t {
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
	void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
	void control_timer_callback(const ros::TimerEvent &event);
	void slipper_loaded_callback(const std_msgs::UInt8::ConstPtr &msg);
	void TargetReachedCallback(const std_msgs::UInt8::ConstPtr &msg);

	ros::NodeHandle nh_;

	ros::Subscriber base_status_sub;
	ros::Publisher base_cmd_pub;
	std_msgs::UInt8 base_cmd_msg;

	ros::Publisher launcher_cmd_pub; //cmdはbaseと分けました
	std_msgs::UInt8 launcher_cmd_msg;

	ros::Publisher launcher_reloader_cmd_pos_pub;
	std_msgs::Float32 launcher_reloader_cmd_pos_msg;

	ros::Publisher launcher_trigger_pub; //トリガー
	std_msgs::UInt8 launcher_trigger_msg;

	ros::Subscriber base_conf_sub;

	ros::Subscriber joy_sub;

	ros::Subscriber slipper_loaded_sub; //スリッパ検出かつスタンバイからのスタート

	ros::Publisher target_pos_pub; //移動先の座標送るアレ 移動の部分はmove.py
	geometry_msgs::Twist target_pos_msg;
	ros::Subscriber target_reached_sub;

	pose start_zone;
	pose throwing_zone;

	ros::Timer control_timer;

	void shutdown(void); //要らないと思う
	void start(void);
	void init(void);

	void reloader(float);

	void trigger(uint8_t);
	double launcher_down = -11.4;

	uint8_t now_command = (uint8_t) fullOP::standby;

	bool operating = false; //動作中に他のが割り込まないように 意味あるのか不明

	bool first_time_restart = false; //ESshutdown後のrestart用 standbyに仕込む

	int slipper_detect_counter = 0;
	int slipper_detect_threshold = 40; //20Hzでセンサの状態くるらしいので2秒ほどスリッパを検出すると発射する
	int sensor_0 = 0;
	int sensor_err_tolerance = 2; //センサの誤検出でいつまでも発射できないとつらみが深いので許容分

	bool once_started = false; //動作中にstart()がまたされないように

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
	nh_.param("/main/launcher_down", this->launcher_down, -11.4);

	ROS_INFO("start_zone_X : %lf", this->start_zone.X);
	ROS_INFO("start_zone_Y : %lf", this->start_zone.Y);
	ROS_INFO("start_zone_YAW : %lf", this->start_zone.YAW);
	ROS_INFO("throwing_zone_X : %lf", this->throwing_zone.X);
	ROS_INFO("throwing_zone_Y : %lf", this->throwing_zone.Y);
	ROS_INFO("throwing_zone_YAW : %lf", this->throwing_zone.YAW);
	ROS_INFO("slipper_detect_threshold : %d", this->slipper_detect_threshold);
	ROS_INFO("sensor_err_tolerance : %d", this->sensor_err_tolerance);
	ROS_INFO("launcher_down : %lf", this->launcher_down);

	this->base_status_sub = nh_.subscribe < std_msgs::UInt8
			> ("base/status", 10, &Mr1Main::baseStatusCallback, this);
	this->base_cmd_pub = nh_.advertise < std_msgs::UInt8 > ("base/cmd", 10);

	this->slipper_loaded_sub = nh_.subscribe < std_msgs::UInt8
			> ("slipper_loaded", 10, &Mr1Main::slipper_loaded_callback, this);

	this->launcher_cmd_pub = nh_.advertise < std_msgs::UInt8
			> ("launcher/cmd", 10);
	this->launcher_reloader_cmd_pos_pub = nh_.advertise < std_msgs::Float32
			> ("launcher_reloader_cmd_pos", 10);
	this->launcher_trigger_pub = nh_.advertise < std_msgs::UInt8
			> ("launcher_trigger", 10);

	this->joy_sub = nh_.subscribe < sensor_msgs::Joy
			> ("joy", 10, &Mr1Main::joyCallback, this);

	this->target_pos_pub = nh_.advertise < geometry_msgs::Twist
			> ("target_pos", 10, true);
	this->target_reached_sub = nh_.subscribe < std_msgs::UInt8
			> ("target_reached", 10, &Mr1Main::TargetReachedCallback, this);

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
	this->operating = false;
}

void Mr1Main::baseStatusCallback(const std_msgs::UInt8::ConstPtr &msg) {
	ROS_INFO("restart signal detected");
	if ((BaseStatus) msg->data == BaseStatus::restart) {
		ros::shutdown();
	}
}

void Mr1Main::slipper_loaded_callback(const std_msgs::UInt8::ConstPtr &msg) { //連続で1が41回以上来るとstart //0が2回連続でカウントリセット 初回のみ非接点スイッチの役割を兼ねる
	if (msg->data == 0x01) {
		this->slipper_detect_counter++;
		this->sensor_0 = 0;
		if (this->slipper_detect_counter > 10 && !this->once_started) {
			this->now_command = (uint8_t) fullOP::start;
			this->once_started = true;
			this->slipper_detect_counter = 0;
		} else if (this->slipper_detect_counter
				> this->slipper_detect_threshold) {
			this->now_command = (uint8_t) fullOP::launch;
			this->slipper_detect_counter = 0;
		}
	} else {
		if (this->sensor_0 > this->sensor_err_tolerance) {
			this->slipper_detect_counter = 0;
			this->sensor_0 = 0;
		}
		this->sensor_0++;
	}
}

void Mr1Main::control_timer_callback(const ros::TimerEvent &event) { //最初はlauncherは下がった状態　戻す奴は上
	if (this->now_command == (uint8_t) fullOP::standby && !this->operating) {
		if (!this->first_time_restart) { //起動時一回のみ
			this->operating = true;
			ros::Duration duration(1);
			duration.sleep();
			this->init();
			this->start(); //各種enable
			first_time_restart = true;
			this->operating = false;
		}
	} else if (this->now_command == (uint8_t) fullOP::start //now_commandをstartにするのはslipper_loaded_callback
	&& !this->operating) {
		this->operating = true;
		this->now_command = (uint8_t) fullOP::sz_to_tp;
		this->operating = false;
	} else if (this->now_command == (uint8_t) fullOP::sz_to_tp
			&& !this->operating) {
		this->operating = true;
		target_pos_msg.linear.x = this->throwing_zone.X;
		target_pos_msg.linear.y = this->throwing_zone.Y;
		target_pos_msg.angular.z = this->throwing_zone.YAW;
		target_pos_pub.publish(target_pos_msg);
		this->now_command = (uint8_t) fullOP::standby; //operatingはtarget_reached_callbackでfalseに now_commandはtpで待機するためにstandbyに
		//now_commandをlaunchにするのはslipper_loaded_callback内
	} else if (this->now_command == (uint8_t) fullOP::launch
			&& !this->operating) {
		this->operating = true;
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
		this->reloader(this->launcher_down); //下げる
		duration.sleep(); //リロードちゃんとできるまで待つ
		this->now_command = (uint8_t) fullOP::release_reloader;
		this->operating = false;
	} else if (this->now_command == (uint8_t) fullOP::release_reloader
			&& !this->operating) {
		this->operating = true;
		this->reloader(0.0); //発射の邪魔なので戻す
		this->now_command = (uint8_t) fullOP::wait_next_throw;
		this->operating = false;
	} else if (this->now_command == (uint8_t) fullOP::wait_next_throw
			&& !this->operating) {
		this->operating = true;
		this->trigger(0x10); //電磁弁enable
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

}

void Mr1Main::start(void) {
	ROS_INFO("start.");
	ros::Duration duration(0.00003);
	for (int i = 0; i < 10; i++) { //たまにCANがうまく届かなくて悲しみを感じたのでこうしたった
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

int main(int argc, char **argv) {
	ros::init(argc, argv, "mr1_main");

	Mr1Main *instance = new Mr1Main();
	ROS_INFO("MR1 main node has started.");

	ros::spin();
	ROS_INFO("MR1 main node has been terminated.");
}
