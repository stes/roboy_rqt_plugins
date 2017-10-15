#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <pabiroboy_joint_control/ui_pabiroboy_joint_control.h>
#include <roboy_communication_middleware/JointStatus.h>
#include <roboy_communication_middleware/MotorCommand.h>
#include <QWidget>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QSlider>
#include <QLineEdit>
#include <QPushButton>
#include <map>

#endif

#define NUMBER_OF_JOINTS 4

using namespace std;

class PaBiRoboyJointControl : public rqt_gui_cpp::Plugin {
    Q_OBJECT
public:
    PaBiRoboyJointControl();

    virtual void initPlugin(qt_gui_cpp::PluginContext &context);

    virtual void shutdownPlugin();

    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                              qt_gui_cpp::Settings &instance_settings) const;

    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                 const qt_gui_cpp::Settings &instance_settings);

public Q_SLOTS:
    void plotData();
    void activate();
    void updateSetPointsJointControl();
    void updateSetPointsJointControlAll();
    void JointController();
private:
    void JointStatus(const roboy_communication_middleware::JointStatus::ConstPtr &msg);
Q_SIGNALS:
    void newData();
private:
    Ui::PaBiRoboyJointControl ui;
    QWidget *widget_;

    QVector<double> time;
    int counter = 0;
    QVector<double> jointData[NUMBER_OF_JOINTS], jointDataSetpoint[NUMBER_OF_JOINTS];
    int samples_per_plot = 300;
    QColor color_pallette[14] = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::darkGray, Qt::darkRed, Qt::darkGreen,
                                 Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow, Qt::black, Qt::gray};
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Publisher motorCommand;
    map<string,QLineEdit*> text;
    map<string,QPushButton*> button;
    map<string,QSlider*> slider;
    map<int,QLineEdit*> flexor, extensor;
    bool joint_control_active = false;
};
