#include <roboy_motor_calibration/roboy_motor_calibration.hpp>

RoboyMotorCalibration::RoboyMotorCalibration()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("RoboyMotorCalibration");
}

void RoboyMotorCalibration::initPlugin(qt_gui_cpp::PluginContext &context) {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    for(uint fpga = 0; fpga<NUMBER_OF_FPGAS; fpga++) {
        stopButton[fpga] = false;
    }


    button["stop_button_all"] = widget_->findChild<QPushButton *>("stop_button_all");
    button["calibrate"] = widget_->findChild<QPushButton *>("calibrate");

    text["data_points"] = widget_->findChild<QLineEdit *>("data_points");
    text["timeout"] = widget_->findChild<QLineEdit *>("timeout");
    text["degree"] = widget_->findChild<QLineEdit *>("degree");
    text["displacement_min"] = widget_->findChild<QLineEdit *>("displacement_min");
    text["displacement_max"] = widget_->findChild<QLineEdit *>("displacement_max");

    text["data_points"]->setToolTip("amount of samples to use for regression");
    text["timeout"]->setToolTip("the calibration will be timed out\n"
                                        "if the number of samples was not reached");
    text["degree"]->setToolTip("degree of the polynomial regression");
    text["displacement_min"]->setToolTip("minimal/maximal displacment to be sampled from");
    text["displacement_max"]->setToolTip("minimal/maximal displacment to be sampled from");

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "motor_calibration_rqt_plugin");
    }

    motorStatus = nh->subscribe("/roboy/middleware/MotorStatus", 1, &RoboyMotorCalibration::MotorStatus, this);
    loadCells = nh->subscribe("/roboy/middleware/LoadCells", 1, &RoboyMotorCalibration::ADCvalue, this);
    motorCalibration = nh->serviceClient<roboy_communication_middleware::MotorCalibrationService>("/roboy/middleware/MotorCalibration");

    ui.stop_button_all->setStyleSheet("background-color: green");
    QObject::connect(button["stop_button_all"], SIGNAL(clicked()), this, SLOT(stopButtonAllClicked()));
    QObject::connect(button["calibrate"], SIGNAL(clicked()), this, SLOT(MotorCalibration()));

    ui.load->addGraph();
    ui.load->graph(0)->setPen(QPen(color_pallette[0]));

    ui.adc_value->addGraph();
    ui.adc_value->graph(0)->setPen(QPen(color_pallette[1]));

    ui.displacement->addGraph();
    ui.displacement->graph(0)->setPen(QPen(color_pallette[2]));

    ui.load->xAxis->setLabel("x");
    ui.load->yAxis->setLabel("Newton");
    ui.load->yAxis->setRange(0, 500);
    ui.load->replot();

    ui.adc_value->xAxis->setLabel("x");
    ui.adc_value->yAxis->setLabel("adc_value");
    ui.adc_value->yAxis->setRange(0, 4095);
    ui.adc_value->replot();

    ui.displacement->xAxis->setLabel("x");
    ui.displacement->yAxis->setLabel("displacement");
    ui.displacement->replot();

    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));
}

void RoboyMotorCalibration::shutdownPlugin() {
    // unregister all publishers here
}

void RoboyMotorCalibration::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                         qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void RoboyMotorCalibration::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                            const qt_gui_cpp::Settings &instance_settings){

}

void RoboyMotorCalibration::stopButtonAllClicked(){
    std_srvs::SetBool msg;
    if(button["stop_button_all"]->isChecked()) {
        ui.stop_button_all->setStyleSheet("background-color: red");
        msg.request.data = 1;
        emergencyStop.call(msg);
        button["calibrate"]->setEnabled(false);
    }else {
        ui.stop_button_all->setStyleSheet("background-color: green");
        msg.request.data = 0;
        emergencyStop.call(msg);
        button["calibrate"]->setEnabled(true);
    }
}

void RoboyMotorCalibration::MotorCalibration(){
    ROS_INFO("starting motor calibration");
    if(button["calibrate"]->isChecked()){
        roboy_communication_middleware::MotorCalibrationService msg;
        msg.request.fpga = ui.fpga->value();
        msg.request.motor = ui.motor->value();
        msg.request.numberOfDataPoints = text["data_points"]->text().toInt();
        msg.request.timeout = text["timeout"]->text().toInt();
        msg.request.displacement_min = text["displacement_min"]->text().toInt();
        msg.request.displacement_max = text["displacement_max"]->text().toInt();
        calibration_thread.reset( new boost::thread(
                        [this, &msg]() { this->motorCalibration.call(msg); }
                ));
    }
}

void RoboyMotorCalibration::MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
    ROS_INFO_THROTTLE(5, "receiving motor status");
    time.push_back(counter++);
    motorData.push_back(msg->displacement[ui.motor->value()]);
    if (motorData.size() > samples_per_plot) {
        motorData.pop_front();
    }
    if (time.size() > samples_per_plot)
        time.pop_front();

    if (counter % 10 == 0)
            Q_EMIT newData();
}

void RoboyMotorCalibration::ADCvalue(const roboy_communication_middleware::ADCvalue::ConstPtr &msg) {
    ROS_INFO_THROTTLE(5, "receiving load_cell status");
    time.push_back(counter++);
    loadCellLoad.push_back(0.2659488846 + 0.0990372554/4.0*msg->load[ui.motor->value()]);
    if (loadCellLoad.size() > samples_per_plot) {
        loadCellLoad.pop_front();
    }

    loadCellValue.push_back(msg->adc_value[ui.motor->value()]);
    if (loadCellValue.size() > samples_per_plot) {
        loadCellValue.pop_front();
    }
    if (time.size() > samples_per_plot)
        time.pop_front();

    if (counter % 10 == 0)
            Q_EMIT newData();
}

void RoboyMotorCalibration::plotData() {
    ui.load->graph(0)->setData(time, loadCellLoad);
    ui.load->xAxis->rescale();
    ui.load->replot();

    ui.adc_value->graph(0)->setData(time, loadCellValue);
    ui.adc_value->xAxis->rescale();
    ui.adc_value->replot();

    ui.displacement->graph(0)->setData(time, motorData);
    ui.displacement->graph(0)->rescaleAxes();
    ui.displacement->replot();
}

PLUGINLIB_DECLARE_CLASS(roboy_motor_calibration, RoboyMotorCalibration, RoboyMotorCalibration, rqt_gui_cpp::Plugin)
