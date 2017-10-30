#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <roboy_motor_calibration/ui_roboy_motor_calibration.h>
#include <roboy_communication_middleware/ADCvalue.h>
#include <roboy_communication_middleware/MotorCalibrationService.h>
#include <roboy_communication_middleware/MotorStatus.h>
#include <QWidget>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QPushButton>
#include <QLineEdit>
#include <map>
#include <mutex>
#include <common_utilities/CommonDefinitions.h>
#include <std_srvs/SetBool.h>
#include <boost/thread/thread.hpp>
#include <qcustomplot.h>
#include <yaml-cpp/yaml.h>
#include <QFileInfo>
#include <fstream>

#endif

#define NUMBER_OF_MOTORS_PER_FPGA 14
#define NUMBER_OF_FPGAS 6

using namespace std;

class RoboyMotorCalibration
        : public rqt_gui_cpp::Plugin {
    Q_OBJECT
public:
    RoboyMotorCalibration();

    virtual void initPlugin(qt_gui_cpp::PluginContext &context);

    virtual void shutdownPlugin();

    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                              qt_gui_cpp::Settings &instance_settings) const;

    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                 const qt_gui_cpp::Settings &instance_settings);
    /**
     * Reads a yaml motor config file
     * @param filepath to config
     * @return success
     */
    bool readConfig(string filepath);

    /**
     * Writes a yaml motor config file
     * @param filepath
     * @return success
     */
    bool writeConfig(string filepath);
public Q_SLOTS:
    void stopButtonAllClicked();
    void MotorCalibration();
    void plotData();
    void loadConfig();
private:
    void MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);
    void ADCvalue(const roboy_communication_middleware::ADCvalue::ConstPtr &msg);
Q_SIGNALS:
    void newData();

private:
    Ui::RoboyMotorCalibration ui;
    QWidget *widget_;
    ros::NodeHandlePtr nh;
    ros::Subscriber motorStatus, loadCells;
    ros::ServiceClient motorCalibration, emergencyStop;
private:
    mutex mux;
    boost::shared_ptr<boost::thread> calibration_thread;
    QVector<double> time, timeMotorData;
    static map<int,vector<float>> coeffs;
    int counter = 0, samples_per_plot = 300;
    QVector<double> motorData, motorDataCalibrated, loadCellLoad, loadCellValue;
    map<int,bool> stopButton;
    map<string, QPushButton*> button;
    map<string, QLineEdit*> text;
    QColor color_pallette[14] = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::darkGray, Qt::darkRed, Qt::darkGreen,
                                 Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow, Qt::black, Qt::gray};
};
