#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <roboy_motor_command/ui_roboy_motor_command.h>
#include <roboy_communication_middleware/MotorCommand.h>
#include <QWidget>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <map>

#endif

#define NUMBER_OF_MOTORS_PER_FPGA 14
#define NUMBER_OF_FPGAS 6

using namespace std;

class RoboyMotorCommand
        : public rqt_gui_cpp::Plugin {
    Q_OBJECT
public:
    RoboyMotorCommand();

    virtual void initPlugin(qt_gui_cpp::PluginContext &context);

    virtual void shutdownPlugin();

    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                              qt_gui_cpp::Settings &instance_settings) const;

    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                 const qt_gui_cpp::Settings &instance_settings);
private:
    void stopButtonClicked();
    void stopButtonAllClicked();
private:
    Ui::RoboyMotorCommand ui;
    QWidget *widget_;
    ros::NodeHandlePtr nh;
    ros::Publisher motorCommand;
    ros::ServiceClient motorConfig;
private:
    map<int,bool> stopButton;
    map<int,map<int,int>> scale;
    map<int,map<int,int>> setpoint;
    map<int, Q
    enum MOTOR{ MOTOR0, MOTOR1, MOTOR2, MOTOR3, MOTOR4, MOTOR5, MOTOR6, MOTOR7,
        MOTOR8, MOTOR9, MOTOR10, MOTOR11, MOTOR12, MOTOR13, MOTORALL};
};
