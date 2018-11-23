#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "dbw_mkz_msgs/ThrottleCmd.h"
#include "dbw_mkz_msgs/SteeringCmd.h"
#include "dbw_mkz_msgs/BrakeCmd.h"
#include "dbw_mkz_msgs/SteeringReport.h"

#include "pid.h"


using namespace std;
float steering_wheel_angle, steering_wheel_angle_cmd, speed;
float speed_cmd, angular_speed_cmd;// omega = v / L * steering
float current_angular_speed;
bool is_cmd_vel_init = false;
bool is_steering_report_init = false;
bool is_imu_init = false;
void steering_report_cb(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg)
{
    
    steering_wheel_angle = msg->steering_wheel_angle;
    steering_wheel_angle_cmd = msg->steering_wheel_angle_cmd;
    speed = msg->speed;
    is_cmd_vel_init = true;
}
void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    is_steering_report_init = true;
    speed_cmd = float(msg->linear.x);
    angular_speed_cmd = float(msg->angular.z);
}
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    if(! is_imu_init)
    {
        is_imu_init = true;
        current_angular_speed = float(msg->angular_velocity.z);
    }
    else
    {
        float alpha = 0.9;
        current_angular_speed = alpha * float(msg->angular_velocity.z) + (1 - alpha) * current_angular_speed;
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "/vehicle/twist_controller");
    ros::NodeHandle n;
    ros::Publisher pub_handle1 = n.advertise<dbw_mkz_msgs::ThrottleCmd>("/vehicle/throttle_cmd", 1);
    ros::Publisher pub_handle2 = n.advertise<dbw_mkz_msgs::SteeringCmd>("/vehicle/steering_cmd", 1);
    ros::Publisher pub_handle3 = n.advertise<dbw_mkz_msgs::BrakeCmd>("/vehicle/brake_cmd", 1);

    ros::Subscriber sub_handle_1 = n.subscribe("/vehicle/steering_report", 1, steering_report_cb);
    ros::Subscriber sub_handle_2 = n.subscribe("/vehicle/cmd_vel", 1, cmd_vel_cb);
    ros::Subscriber sub_handle_3 = n.subscribe("/vehicle/imu/data_raw", 1, imu_cb);
    ros::Rate loop_rate(50);

    PID pid_instance_speed(0.02, 10, 2, 2);
    pid_instance_speed.integrate_max = 500;
    PID pid_instance_angle(0.02, 80, 0.000, 10);
    pid_instance_angle.integrate_max = 100;
    pid_instance_angle.output_max = 4;
    pid_instance_angle.output_min = -4;
    while(!(is_cmd_vel_init && is_steering_report_init && is_imu_init))
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
    const float Car_length_estimated = 2;
    while (ros::ok())
    {
        float delta_speed = (speed_cmd - speed)/(abs(speed_cmd) + 0.01);
        float delta_angle = (angular_speed_cmd - current_angular_speed) * Car_length_estimated / (abs(speed_cmd) + 0.01);

        float steering_output = pid_instance_angle.step(delta_angle);
        
        float speed_output = pid_instance_speed.step(delta_speed);
        dbw_mkz_msgs::ThrottleCmd throttle_cmd;
        throttle_cmd.pedal_cmd_type = throttle_cmd.CMD_PERCENT;
        throttle_cmd.pedal_cmd = 0;
        if(speed_output > 0)
            throttle_cmd.pedal_cmd = speed_output;
        throttle_cmd.enable = true;
        throttle_cmd.ignore = false;
        throttle_cmd.clear = false;
        throttle_cmd.count = 0;
        pub_handle1.publish(throttle_cmd);

        dbw_mkz_msgs::SteeringCmd steering_cmd;
        steering_cmd.steering_wheel_angle_velocity = 0;
        steering_cmd.clear = false;
        steering_cmd.ignore = false;
        steering_cmd.enable = true;
        steering_cmd.count = 0;
        steering_cmd.quiet = false;
        steering_cmd.steering_wheel_angle_cmd = steering_output;
        pub_handle2.publish(steering_cmd);

        dbw_mkz_msgs::BrakeCmd brake_cmd;
        brake_cmd.boo_cmd = false;
        brake_cmd.clear = false;
        brake_cmd.count = 0;
        brake_cmd.enable = true;
        brake_cmd.ignore = false;
        brake_cmd.pedal_cmd_type = 2;
        brake_cmd.pedal_cmd = 0;
        if(speed_output < 0)
            brake_cmd.pedal_cmd = -speed_output;
        pub_handle3.publish(brake_cmd);

        ros::spinOnce();
        loop_rate.sleep();
    }
}