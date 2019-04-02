#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#include <ball_chaser/DriveToTarget.h>

#include <iostream>
#include <vector>

class BallChaser {
public:
    BallChaser(ros::NodeHandle nh, ros::NodeHandle private_nh)
        : nh_(nh), private_nh_(private_nh)
    {
        ROS_INFO("Welcome to node '%s'", ros::this_node::getName().c_str());
        
        std::string robot_speed_topic;
        private_nh_.param<std::string>("robot_speed_topic", robot_speed_topic, "/cmd_vel");
        
        motor_command_publisher_ = nh_.advertise<geometry_msgs::Twist>(robot_speed_topic, 10);
    }
    
    void publishRobotSpeed(ros::Publisher& pub, double linear_x, double angular_z) {
        geometry_msgs::Twist msg;
        msg.linear.x = linear_x;
        msg.angular.z = angular_z;
        pub.publish(msg);
    }
    
    bool handleDriveRequest(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response& res) {
        this->publishRobotSpeed(motor_command_publisher_, req.linear_x, req.angular_z);
        
        ros::Duration(1).sleep();
        res.msg_feedback = "Moved robot";
        ROS_INFO("'%s' Driving requested (%.2f, %.2f). Res: %s", 
                 ros::this_node::getName().c_str(),
                 req.linear_x, req.angular_z,
                 res.msg_feedback.c_str()
                );
        
        return true;
    }
    
    void run(){
        ros::spin();
    }
    
private:
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher motor_command_publisher_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle nh, private_nh("~");
    
    std::string service_name;
    private_nh.param<std::string>("service_name", service_name, "ball_chaser/command_robot");
    
    BallChaser ball_chaser(nh, private_nh);
    ros::ServiceServer service = nh.advertiseService(service_name, &BallChaser::handleDriveRequest, &ball_chaser);
    
    ball_chaser.run();
    
    return 0;
}
