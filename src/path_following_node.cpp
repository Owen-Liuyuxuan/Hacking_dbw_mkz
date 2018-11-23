#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose2D.h"
#include "dbw_mkz_msgs/SteeringReport.h"
#include "geometry_msgs/Twist.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"

#include <vector>
#include <iostream>
#include <cmath>
typedef unsigned int uint;
using namespace std;
float steering_wheel_angle, steering_wheel_angle_cmd, speed;
bool is_steering_report_init = false;
bool is_path_init = false;

vector<geometry_msgs::Pose2D> normalized_path;
MPC mpc;
double output_omega;

double polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

inline geometry_msgs::Pose2D Pose2Pose2D(const geometry_msgs::Pose msg)
{
    geometry_msgs::Pose2D output_msg;
    output_msg.x = msg.position.x;
    output_msg.y = msg.position.y;
    output_msg.theta = 2 * atan2(msg.orientation.z, msg.orientation.w);
    return output_msg;
}

inline geometry_msgs::Pose2D get_relevant_pose(const geometry_msgs::Pose2D msg, const geometry_msgs::Pose2D reference_msg)
{
    geometry_msgs::Pose2D result;
    double delta_x = msg.x - reference_msg.x;
    double delta_y = msg.y - reference_msg.y;
    result.theta = msg.theta - reference_msg.theta;
    result.x = delta_x * cos(reference_msg.theta) + delta_y * sin(reference_msg.theta);
    result.y = -delta_x * sin(reference_msg.theta) + delta_y * cos(reference_msg.theta);
    return result;
}

void steering_report_cb(const dbw_mkz_msgs::SteeringReport::ConstPtr &msg)
{
    steering_wheel_angle = msg->steering_wheel_angle;
    steering_wheel_angle_cmd = msg->steering_wheel_angle_cmd;
    speed = msg->speed;
    is_steering_report_init = true;
}

void path_cb(const nav_msgs::Path::ConstPtr &msg)
{
    if(! is_steering_report_init)
        return;

    if(msg->poses.size() < 4)
        return;

    Eigen::VectorXd xvals(msg->poses.size());
    Eigen::VectorXd yvals(msg->poses.size());
    for (uint i = 0; i < msg->poses.size(); i++)
    {
        //geometry_msgs::Pose2D relevant_pose = get_relevant_pose(Pose2Pose2D(original_path[i]), reference_pose_2d);
        //normalized_path.push_back(relevant_pose);
        xvals[i] = msg->poses[i].pose.position.x;
        yvals[i] = msg->poses[i].pose.position.y;
    }
    Eigen::VectorXd coeffs = polyfit(xvals, yvals, 3);
    Eigen::VectorXd state(3);
    vector<double> result = mpc.Solve(state, coeffs);
    output_omega = result[0];
    is_path_init = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "/vehicle/path_following");
    ros::NodeHandle n;
    ros::Publisher pub_handle1 = n.advertise<geometry_msgs::Twist>("/vehicle/cmd_vel", 1);
    ros::Subscriber sub_handle1 = n.subscribe("/vehicle/steering_report", 1, steering_report_cb);
    ros::Subscriber sub_handle2 = n.subscribe("/vehicle/target_path", 1, path_cb);
    ros::Rate loop_rate(50);
    while(!(is_path_init && is_steering_report_init))
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    while (ros::ok())
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 15;
        cmd_vel.angular.z = output_omega;
        pub_handle1.publish(cmd_vel);
        ros::spinOnce();
        loop_rate.sleep();
    }
}