#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

class drive_bot
{
public:
    drive_bot()
    {
        // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
        motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

         // Define a drive /ball_chaser/command_robot service with a callback function
        service = n.advertiseService("/ball_chaser/command_robot", &drive_bot::handle_drive_request, this);
    }

    // TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
    // This function should publish the requested linear x and angular velocities to the robot wheel joints
    // After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities


    // TODO: Delete the loop, move the code to the inside of the callback function and make the necessary changes 
    // to publish the requested velocities instead of constant values
    bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
        ball_chaser::DriveToTarget::Response& res) {
        // Create a motor_command object of type geometry_msgs::Twist
        geometry_msgs::Twist motor_command;
        // Set wheel velocities, forward [0.5, 0.0]
        motor_command.linear.x = (float)req.linear_x;
        motor_command.angular.z = (float)req.angular_z;
        // Publish angles to drive the robot
        motor_command_publisher.publish(motor_command);

        res.msg_feedback = "Requested Wheel velocities, lin. x: " + std::to_string(motor_command.linear.x) + " , ang. z: " + std::to_string(motor_command.angular.z);
    }

private:
    ros::NodeHandle n;
    // ROS::Publisher motor commands;
    ros::Publisher motor_command_publisher;
    ros::ServiceServer service;
};

int main(int argc, char **argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    drive_bot adrive_bot;

    // Handle ROS communication events
    ros::spin();

    return 0;
}
