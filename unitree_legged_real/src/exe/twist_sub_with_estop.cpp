#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/Twist.h>

using namespace UNITREE_LEGGED_SDK;
class Custom
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd temp_high_cmd = {0};
    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};
    bool estop = false;

    Safety safe;
    xRockerBtnDataStruct _keyData;
    int motiontime = 0;
    float dt = 0.002; // 0.001~0.01

public:
    Custom()
        : 
        // low_udp(LOWLEVEL),
        low_udp(LOWLEVEL, 8090, "192.168.123.10", 8007),
        high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)),
        safe(LeggedType::Go1)
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
    }

    void highUdpSend()
    {
        // printf("high udp send is running\n");

        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void lowUdpSend()
    {

        // low_udp.SetSend(low_cmd);
        low_udp.Send();
    }

    void lowUdpRecv()
    {

        low_udp.Recv();
        // low_udp.GetRecv(low_state);
    }

    void highUdpRecv()
    {
        // printf("high udp recv is running\n");

        high_udp.Recv();
        high_udp.GetRecv(high_state);
    }

    void RobotControl();
};

void Custom::RobotControl()
{
  motiontime++;
  low_udp.GetRecv(low_state);

  memcpy(&_keyData, &low_state.wirelessRemote[0], 40);

  if ((int)_keyData.btn.components.A == 1)
  {
    std::cout << "The key A is pressed, and the value of lx is " << _keyData.lx << std::endl;
	estop = true;
  }

  if (motiontime > 10)
  {
    int res1 = safe.PowerProtect(low_cmd, low_state, 1);
    // You can uncomment it for position protection
    // int res2 = safe.PositionProtect(cmd, state, 10);
    if (res1 < 0)
      exit(-1);
  }

//   low_udp.SetSend(low_cmd);

//   safe.PowerProtect(low_cmd, low_state, 1);
//   low_udp.SetSend(low_cmd);
}


Custom custom;

ros::Subscriber sub_cmd_vel;
ros::Publisher pub_high;

ros::Time velocity_timestamp;
// geometry_msgs::Twist velocity;

long cmd_vel_count = 0;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{   
    // printf("cmdVelCallback is running!\t%ld\n", cmd_vel_count);

    custom.temp_high_cmd = rosMsg2Cmd(msg);
    // custom.high_cmd = rosMsg2Cmd(msg);
    velocity_timestamp = ros::Time::now();
    // velocity = *msg;

    // if (custom.estop)
    // {
    //     custom.high_cmd.velocity[0] = 0;
    //     custom.high_cmd.velocity[1] = 0;
    //     custom.high_cmd.yawSpeed = 0;
    // }
    // printf("cmd_x_vel = %f\n", custom.high_cmd.velocity[0]);
    // printf("cmd_y_vel = %f\n", custom.high_cmd.velocity[1]);
    // printf("cmd_yaw_vel = %f\n", custom.high_cmd.yawSpeed);

    // unitree_legged_msgs::HighState high_state_ros;

    // high_state_ros = state2rosMsg(custom.high_state);

    // pub_high.publish(high_state_ros);

    // printf("cmdVelCallback ending!\t%ld\n\n", cmd_vel_count++);
}

void pubVel(const ros::TimerEvent &) {      
    // custom.high_cmd = rosMsg2Cmd(msg);
    ros::Time curr_time = ros::Time::now();

    ros::Duration time_elapsed = curr_time - velocity_timestamp;
    auto time_elapsed_secs = time_elapsed.toSec();
    ROS_INFO("estop pressed: %d", custom.estop);
    ROS_INFO("time elapsed: %lf", time_elapsed_secs);
    
    if (custom.estop || (time_elapsed_secs > 1))
    {
        custom.high_cmd.velocity[0] = 0;
        custom.high_cmd.velocity[1] = 0;
        custom.high_cmd.yawSpeed = 0;
        printf("Setting to 0\n\n");
    } else {
        custom.high_cmd = custom.temp_high_cmd;
        printf("Setting to some velocity\n\n");
        printf("cmd_x_vel = %f\n", custom.high_cmd.velocity[0]);
        printf("cmd_y_vel = %f\n", custom.high_cmd.velocity[1]);
        printf("cmd_yaw_vel = %f\n", custom.high_cmd.yawSpeed);
    }


    // unitree_legged_msgs::HighState high_state_ros;

    // high_state_ros = state2rosMsg(custom.high_state);

    // pub_high.publish(high_state_ros);

    // printf("pub vel ending!\t\n\n");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_sub_with_estop");

    ros::NodeHandle nh;

	ros::Rate loop_rate(500);
    pub_high = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);

    sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdVelCallback);

    custom.temp_high_cmd.velocity[0] = 0;
    custom.temp_high_cmd.velocity[1] = 0;
    custom.temp_high_cmd.yawSpeed = 0;

    velocity_timestamp = ros::Time::now();

    LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
    LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));
    LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
	LoopFunc loop_udpSend_low("low_udp_send", custom.dt, 3, boost::bind(&Custom::lowUdpSend, &custom));
	LoopFunc loop_udpRecv_low("low_udp_recv", custom.dt, 3, boost::bind(&Custom::lowUdpRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();
    loop_udpSend_low.start();
    loop_udpRecv_low.start();

    ros::Timer pub_timer = nh.createTimer(ros::Duration(1/500), pubVel);
    
    // ros::spin();
    while (ros::ok())
	{
        // Kill the nodes that publish /cmd_vel
		if (custom.estop)
		{
			std::string command = "bash /home/unitree/ghar_ws/src/unitree_ros_to_real/unitree_legged_real/bash/kill_cmd_vel_nodes.sh";
            
			if (system(command.c_str()) == 0)
			{
				ROS_INFO("Successfully killed nodes");
			}
			else
			{
				ROS_ERROR("Failed to kill nodes");
			}
			
			// pub.publish(twist_estop);


		}
		ros::spinOnce();
		loop_rate.sleep();
	}

    return 0;
}
