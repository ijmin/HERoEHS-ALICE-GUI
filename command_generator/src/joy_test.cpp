#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <ctime>
#include <diagnostic_msgs/KeyValue.h>
#include <alice_foot_step_generator/FootStepCommand.h>

using namespace std;

class Command_generator 
{
public:
  Command_generator();
  std_msgs::String motion_command;
  std_msgs::String speed_command;
  alice_foot_step_generator::FootStepCommand FootParam;
  int joy_alice_id;
  double default_step_num;
  double default_step_length;
  double default_side_step_length;
  double default_step_angle_rad;
  double default_step_time;
  double expanded_step_num;
  double expanded_step_length;
  double expanded_side_step_length;
  double expanded_step_angle_rad;
  double expanded_step_time;
  double centered_step_num;
  double centered_step_length;
  double centered_side_step_length;
  double centered_step_angle_rad;
  double centered_step_time;

  ros::Publisher vel_pub_;
  string step_type;
  void Set_FootParam(void);
  int command_switch;
  string speed_switch;
  ofstream out;
  //Text_Input//
  float Command_Period;
  /////////////

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void alice_id_Callback(const std_msgs::String::ConstPtr& alice_id);
  void parse_step_param_data(std::string path);
  void Input_Text(void);

  ros::NodeHandle nh_;
  
  int linear_, angular_;
  
  ros::Subscriber joy_sub_;
  ros::Subscriber alice_id_sub_;
};

Command_generator::Command_generator():
  linear_(1),
  angular_(2)
{
  //Default_Setting//
  Input_Text();
  command_switch = 0;
  speed_switch = 2;
  FootParam.step_num = 0;
  FootParam.step_length = 0;
  FootParam.side_step_length = 0;
  FootParam.step_angle_rad = 0;
  FootParam.step_time = 0;
  //////////////////
  
  ROS_INFO("command_generator_start");
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  vel_pub_ = nh_.advertise<alice_foot_step_generator::FootStepCommand>("/heroehs/command_generator", 10);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Command_generator::joyCallback, this);
  alice_id_sub_ = nh_.subscribe<std_msgs::String>("/heroehs/alice_id", 10, &Command_generator::alice_id_Callback, this);
}

void Command_generator::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if(joy->axes[6] == 1)
  {
    FootParam.command = "left";
    step_type = "default";
    command_switch = 2;
  }
  else if(joy->axes[6] == -1)
  {
    FootParam.command = "right";
    step_type = "default";
    command_switch = 2;
  }
  else if(joy->axes[7] == 1)
  {
    FootParam.command = "forward";
    step_type = "default";
    command_switch = 2;
  }
  else if(joy->axes[7] == -1)
  {
    FootParam.command = "backward";
    step_type = "default";
    command_switch = 2;
  }
  else if(joy->axes[2] == -1)
  {
    FootParam.command = "centered left";
    step_type = "centered";
    command_switch = 2;
  }
  else if(joy->axes[5] == -1)
  {
    FootParam.command = "centered right";
    step_type = "centered";
    command_switch = 2;
  }
  else if(joy->buttons[4] == 1)
  {
    FootParam.command = "turn left";
    step_type = "default";
    command_switch = 2;
  }
  else if(joy->buttons[5] == 1)
  {
    FootParam.command = "turn right";
    step_type = "default";
    command_switch = 2;
  }
  else if(joy->buttons[0] == 1)
  {
    FootParam.command = "expanded left";
    step_type = "expanded";
    command_switch = 2;
  }
  else if(joy->buttons[1] == 1)
  {
    FootParam.command = "expanded right";
    step_type = "expanded";
    command_switch = 2;
  }
  else if(joy->buttons[9] == 1)
  {
    if(joy_alice_id == 1)
    {
      FootParam.command = "left kick";
      step_type = "default";
      command_switch = 2;
    }
  }
  else if(joy->buttons[10] == 1)
  {
    if(joy_alice_id == 1)
    {
      FootParam.command = "right kick";
      step_type = "default";
      command_switch = 2;
    }
  }
  else if(joy->buttons[2] == 1)
  {
    FootParam.command = "stop";
    step_type = "default";
    command_switch = 2;
  }
  else if(joy->buttons[6] == 1)
  {
    speed_switch = "1";
  }
  else if(joy->buttons[7] == 1)
  {
    speed_switch = "3";
  }
  else if(joy->buttons[8] == 1)
  {
    speed_switch = "2";
  }
  else
  {
    if(command_switch == 2)
    {
      FootParam.command = "stop";
      step_type = "default";
      command_switch = 0;
    }
    else if(command_switch == 1)
    {
      FootParam.command = "stop";
      step_type = "default";
    }
    else if(command_switch == 0)command_switch = 0;
  }
  Set_FootParam();
}

void Command_generator::alice_id_Callback(const std_msgs::String::ConstPtr& alice_id)
{
  std::string step_path_;
  if(alice_id->data == "1")
  {
    joy_alice_id = 1;
    step_path_ = ros::package::getPath("alice_gui") + "/config/step_parameter1.yaml";
  }
  else if(alice_id->data == "2")
  {
    joy_alice_id = 2;
    step_path_ = ros::package::getPath("alice_gui") + "/config/step_parameter2.yaml";
  }
  parse_step_param_data(step_path_);
}

void Command_generator::Set_FootParam(void)
{
  if(step_type == "default")
  {
    FootParam.step_num = default_step_num;
    FootParam.step_length = default_step_length;
    FootParam.side_step_length = default_side_step_length;
    FootParam.step_angle_rad = default_step_angle_rad;
    FootParam.step_time = default_step_time;
  }
  else if(step_type == "expanded")
  {
    FootParam.step_num = expanded_step_num;
    FootParam.step_length = expanded_step_length;
    FootParam.side_step_length = expanded_side_step_length;
    FootParam.step_angle_rad = expanded_step_angle_rad;
    FootParam.step_time = expanded_step_time;
  }
  else if(step_type == "centered")
  {
    FootParam.step_num = centered_step_num;
    FootParam.step_length = centered_step_length;
    FootParam.side_step_length = centered_side_step_length;
    FootParam.step_angle_rad = centered_step_angle_rad;
    FootParam.step_time = centered_step_time;
  }
  if(joy_alice_id == 1)
  {
    if(speed_switch == "1")
    {
      FootParam.step_time = FootParam.step_time*1.5;
    }
    else if(speed_switch == "2")
    {
      FootParam.step_time = FootParam.step_time*1;
    }
    else if(speed_switch == "3")
    {
      FootParam.step_time = FootParam.step_time*0.5;
    }
  }
}
void Command_generator::parse_step_param_data(std::string path)
{
	YAML::Node doc;
	try
	{
		// load yaml
		doc = YAML::LoadFile(path.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}
	default_step_num = doc["default_step_num"].as<double>();
	default_step_length = doc["default_step_length"].as<double>();
	default_side_step_length = doc["default_side_step_length"].as<double>();
	default_step_angle_rad = doc["default_step_angle_radian"].as<double>();
	default_step_time = doc["default_step_time"].as<double>();

	expanded_step_num = doc["expanded_step_num"].as<double>();
	expanded_step_length = doc["expanded_step_length"].as<double>();
	expanded_side_step_length = doc["expanded_side_step_length"].as<double>();
	expanded_step_angle_rad = doc["expanded_step_angle_radian"].as<double>();
	expanded_step_time = doc["expanded_step_time"].as<double>();

	centered_step_num = doc["centered_step_num"].as<double>();
	centered_step_length = doc["centered_step_length"].as<double>();
	centered_side_step_length = doc["centered_side_step_length"].as<double>();
	centered_step_angle_rad = doc["centered_step_angle_radian"].as<double>();
	centered_step_time = doc["centered_step_time"].as<double>();
}
void Command_generator::Input_Text(void)
{
  int i = 0;
  size_t found;
  string init_pose_path;
  ifstream inFile;
  init_pose_path = ros::package::getPath("command_generator") + "/command_input.txt";
  inFile.open(init_pose_path.c_str());
  for(string line; std::getline(inFile,line);)
  {
      found=line.find("=");

      switch(i)
      {
      case 0: Command_Period = atof(line.substr(found+2).c_str()); break;
      }
      i +=1;
  }
  inFile.close();
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "command_generator");
  Command_generator command_controller;
  float count;
  count = 0;
  //ROS_INFO("%f",command_controller.Command_Period);

  while(ros::ok())
  {
    if(count > 1000*command_controller.Command_Period)
    {
      if(command_controller.command_switch > 0)
      {
        command_controller.vel_pub_.publish(command_controller.FootParam);
      }
      count = 0;
    }
    else count += 1;
    usleep(1000);
    ros::spinOnce();
  }
}

