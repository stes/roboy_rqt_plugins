#include <ros/ros.h>
#include <roboy_communication_middleware/MotorCommand.h>

int main(){

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "motor_command_speed_test");
    }
    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle);
    ros::Publisher motorCommand = nh->advertise<roboy_communication_middleware::MotorCommand>("/roboy/middleware/MotorCommand", 1);

    roboy_communication_middleware::MotorCommand msg;
    msg.motors.push_back(0);
    msg.motors.push_back(1);
    msg.motors.push_back(2);
    msg.motors.push_back(3);
    msg.motors.push_back(4);
    msg.motors.push_back(5);
    msg.motors.push_back(6);
    msg.motors.push_back(7);
    msg.motors.push_back(8);
    msg.motors.push_back(9);
    msg.motors.push_back(10);
    msg.motors.push_back(11);
    msg.motors.push_back(12);
    msg.motors.push_back(13);

    msg.setPoints.push_back(0);
    msg.setPoints.push_back(1);
    msg.setPoints.push_back(2);
    msg.setPoints.push_back(3);
    msg.setPoints.push_back(4);
    msg.setPoints.push_back(5);
    msg.setPoints.push_back(6);
    msg.setPoints.push_back(7);
    msg.setPoints.push_back(8);
    msg.setPoints.push_back(9);
    msg.setPoints.push_back(10);
    msg.setPoints.push_back(11);
    msg.setPoints.push_back(12);
    msg.setPoints.push_back(13);

    while(ros::ok()){
        motorCommand.publish(msg);
        ros::spinOnce();
    }
}