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
  diagnostic_msgs::KeyValue joystic_command;
  int joy_alice_id;

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
  joystic_command.value = "2";
  speed_switch = 2;
  //////////////////
  
  ROS_INFO("command_generator_start");
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  vel_pub_ = nh_.advertise<diagnostic_msgs::KeyValue>("/heroehs/move_command", 10);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Command_generator::joyCallback, this);
  alice_id_sub_ = nh_.subscribe<std_msgs::String>("/heroehs/alice_id", 10, &Command_generator::alice_id_Callback, this);
}

void Command_generator::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if(joy->axes[6] == 1)
  {
    joystic_command.key = "left";
    command_switch = 2;
  }
  else if(joy->axes[6] == -1)
  {
    joystic_command.key = "right";
    command_switch = 2;
  }
  else if(joy->axes[7] == 1)
  {
    joystic_command.key = "forward";
    command_switch = 2;
  }
  else if(joy->axes[7] == -1)
  {
    joystic_command.key = "backward";
    command_switch = 2;
  }
  else if(joy->axes[2] == -1)
  {
    joystic_command.key = "centered left";
    command_switch = 2;
  }
  else if(joy->axes[5] == -1)
  {
    joystic_command.key = "centered right";
    command_switch = 2;
  }
  else if(joy->buttons[4] == 1)
  {
    joystic_command.key = "turn left";
    command_switch = 2;
  }
  else if(joy->buttons[5] == 1)
  {
    joystic_command.key = "turn right";
    command_switch = 2;
  }
  else if(joy->buttons[0] == 1)
  {
    joystic_command.key = "expanded left";
    command_switch = 2;
  }
  else if(joy->buttons[1] == 1)
  {
    joystic_command.key = "expanded right";
    command_switch = 2;
  }
  else if(joy->buttons[9] == 1)
  {
    if(joy_alice_id == 1)
    {
      joystic_command.key = "left kick";
      command_switch = 2;
    }
  }
  else if(joy->buttons[10] == 1)
  {
    if(joy_alice_id == 1)
    {
      joystic_command.key = "right kick";
      command_switch = 2;
    }
  }
  else if(joy->buttons[2] == 1)
  {
    joystic_command.key = "stop";
    command_switch = 2;
  }
  else if(joy->buttons[6] == 1)
  {
    joystic_command.value = "1";
  }
  else if(joy->buttons[7] == 1)
  {
    joystic_command.value = "3";
  }
  else if(joy->buttons[8] == 1)
  {
    joystic_command.value = "2";
  }
  else
  {
    if(command_switch == 2)
    {
      joystic_command.key = "stop";
      command_switch = 0;
    }
    else if(command_switch == 1)
    {
      joystic_command.key = "stop";
    }
    else if(command_switch == 0)command_switch = 0;
  }
}

void Command_generator::alice_id_Callback(const std_msgs::String::ConstPtr& alice_id)
{
  std::string step_path_;
  if(alice_id->data == "1")
  {
    joy_alice_id = 1;
  }
  else if(alice_id->data == "2")
  {
    joy_alice_id = 2;
  }
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
        command_controller.vel_pub_.publish(command_controller.joystic_command);
      }
      count = 0;
    }
    else count += 1;
    usleep(1000);
    ros::spinOnce();
  }
}

