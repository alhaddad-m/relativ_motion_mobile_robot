// Simple Motion for Mobile Robot
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Joy.h"
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>


using namespace std;
void initialize_parameters(ros::NodeHandle n);
void orientation_controller(double angle_error);
double get_yaw(geometry_msgs::Quaternion q);

// declare message parameters
geometry_msgs::Twist tw_msg;
double t = 0;
double how_long = 0 , theta1 = 0 , theta2 , odom_theta;
double v_cmd = 0 , w_cmd = 0 , max_w = 0.35;
double angle_error = 0 , angle_1_error = 0, angle_2_error=0;
double compensate_angle = 0;
bool angle_1_done= false , angle_2_done = false , activate_cmd = false;


void joy_back(const sensor_msgs::Joy msg)
{
    ROS_INFO("joystek Reading");

    if(msg.buttons[2]==1)
    {
        v_cmd = 0.2;
        compensate_angle =0;
        t = 0;
        angle_1_done = false;
        angle_2_done = false;
        activate_cmd = true;
    }
    else if(msg.buttons[0]==1)
    {
        v_cmd = -0.2;
        compensate_angle = 3.14;
        t = 0;
        angle_1_done = false;
        angle_2_done = false;
        activate_cmd = true;
    }
}

void theta_1_errors()
{
    double err_theta1;
    err_theta1 =  theta1 - odom_theta; // - compensate_angle;
    angle_1_error = atan2(sin(err_theta1),cos(err_theta1));
    if(abs(angle_1_error)<0.1)
    {
        angle_1_done = true;
    }
}

void theta_2_errors()
{
    double err_theta2;
    err_theta2 =  theta2 - odom_theta;
    angle_2_error = atan2(sin(err_theta2),cos(err_theta2));
    if(abs(angle_2_error)<0.1)
    {
        angle_2_done = true;
        activate_cmd = false;
    }
}

void orientation_controller(double angle_error)
{
    // Motion 
    if(abs(angle_error)<0.1)  // case error angle very small
    {
        w_cmd = 1*angle_error;
    }
    else if(abs(angle_error)>=0.1 && abs(angle_error)<0.38)  // case error angle small
    {
        w_cmd = 0.9*angle_error;
    }
    else if(abs(angle_error)>=0.38 && abs(angle_error)<0.7)  // case error angle medium
    {
        w_cmd = 0.85*angle_error;
    }
    else if(abs(angle_error)>=0.7 && abs(angle_error)<1)  // case error angle big
    {
        w_cmd = 0.8*angle_error;
    }
    else if(abs(angle_error)>=1 && abs(angle_error)<1.25)  // case angle error very big
    {
        w_cmd = 0.75*angle_error;
    }
    else if(abs(angle_error)>=1.25)
    {
        w_cmd = 1*angle_error;
    }
    

    if(w_cmd>max_w)
    {w_cmd = max_w;}
    if(w_cmd<-max_w)
    {w_cmd = -max_w;}

    tw_msg.angular.z = w_cmd;  
}
double get_yaw(geometry_msgs::Quaternion q)
{
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

void initialize_parameters(ros::NodeHandle n)
{
    n.getParam("how_long",   how_long);
    n.getParam("theta1",   theta1);
    n.getParam("theta2",   theta2);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "relative_motion");
    ros::NodeHandle n;

    initialize_parameters(n);
    // Define the publishers and sunscribers
    ros::Publisher      cmd_pub  = n.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
    ros::Subscriber     jou_sub  = n.subscribe  ("/joy_teleop/joy", 5, joy_back);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate loop_rate(10); // ros spins 10 frames per second
 
    while (ros::ok())
    {
        geometry_msgs::TransformStamped transformStamped;
        try
        {    
            transformStamped = tfBuffer.lookupTransform("local_map_lidar", "base_link",
                               ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        odom_theta = get_yaw(transformStamped.transform.rotation);


        if(activate_cmd)
        {
            if(!angle_1_done)
            {
                theta_1_errors();
                orientation_controller(angle_1_error);
                tw_msg.linear.x = 0;
            }
            else if(t <= how_long && angle_1_done)
            {
                t = t + 0.1;
                tw_msg.linear.x = v_cmd;
                tw_msg.angular.z = 0;
            }
            else if(angle_1_done && t > how_long && !angle_2_done)
            {
                theta_2_errors();
                orientation_controller(angle_2_error);
                tw_msg.linear.x = 0;
            }
        }
        else
        {
            tw_msg.linear.x = 0;
            tw_msg.angular.z = 0;
        }
            
        cmd_pub.publish(tw_msg);
    
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

