#include <roboy_motor_command/roboy_motor_command.hpp>

RoboyMotorCommand::RoboyMotorCommand()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("RoboyMotorCommand");
}

void RoboyMotorCommand::initPlugin(qt_gui_cpp::PluginContext &context) {
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

    for(uint fpga = 0; fpga<NUMBER_OF_FPGAS; fpga++) {
        control_mode[fpga] = DISPLACEMENT;
    }

    for(uint fpga = 0; fpga<NUMBER_OF_FPGAS; fpga++) {
        for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA + 1; motor++) {
            setpoint[fpga][motor] = 0;
        }
    }

    char str[100];
    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA+1; motor++) {
        sprintf(str,"motor_setPoint_slider_%d", motor);
        setpoint_slider_widget.push_back(widget_->findChild<QSlider *>(str));
    }

    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA+1; motor++) {
        sprintf(str,"motor_setPoint_%d", motor);
        setpoint_widget.push_back(widget_->findChild<QLineEdit *>(str));
    }

    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA+1; motor++) {
        sprintf(str,"motor_scale_%d", motor);
        scale_widget.push_back(widget_->findChild<QLineEdit *>(str));
    }

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "motor_command_rqt_plugin");
    }

    motorCommand = nh->advertise<roboy_communication_middleware::MotorCommand>("/roboy/middleware/MotorCommand", 1);
    motorControl = nh->serviceClient<roboy_communication_middleware::ControlMode>("/roboy/middleware/ControlMode");
    emergencyStop = nh->serviceClient<std_srvs::SetBool>("/roboy/middleware/EmergencyStop");

    ui.stop_button_all->setStyleSheet("background-color: green");
    QObject::connect(ui.stop_button_all, SIGNAL(clicked()), this, SLOT(stopButtonAllClicked()));

    for(uint motor = 0; motor<NUMBER_OF_MOTORS_PER_FPGA; motor++){
        QObject::connect(setpoint_slider_widget.at(motor), SIGNAL(valueChanged(int)), this, SLOT(setPointChanged(int)));
    }
    QObject::connect(setpoint_slider_widget.at(NUMBER_OF_MOTORS_PER_FPGA), SIGNAL(valueChanged(int)), this, SLOT(setPointAllChanged(int)));

    QObject::connect(ui.fpga, SIGNAL(valueChanged(int)), this, SLOT(fpgaChanged(int)));

    for(uint motor = 0; motor<NUMBER_OF_MOTORS_PER_FPGA; motor++){
        QObject::connect(scale_widget.at(motor), SIGNAL(editingFinished()), this, SLOT(scaleChanged()));
    }
    QObject::connect(scale_widget.at(NUMBER_OF_MOTORS_PER_FPGA), SIGNAL(editingFinished()), this, SLOT(scaleChangedAll()));

    QObject::connect(ui.pos, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
    QObject::connect(ui.vel, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
    QObject::connect(ui.dis, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
}

void RoboyMotorCommand::shutdownPlugin() {
    // unregister all publishers here
}

void RoboyMotorCommand::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void RoboyMotorCommand::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void RoboyMotorCommand::stopButtonAllClicked(){
    std_srvs::SetBool msg;
    if(ui.stop_button_all->isChecked()) {
        ui.stop_button_all->setStyleSheet("background-color: red");
        msg.request.data = 1;
        emergencyStop.call(msg);
        ui.pos->setEnabled(false);
        ui.vel->setEnabled(false);
        ui.dis->setEnabled(false);
    }else {
        ui.stop_button_all->setStyleSheet("background-color: green");
        msg.request.data = 0;
        emergencyStop.call(msg);
        ui.pos->setEnabled(true);
        ui.vel->setEnabled(true);
        ui.dis->setEnabled(true);
    }
}

void RoboyMotorCommand::setPointChanged(int){
    roboy_communication_middleware::MotorCommand msg;
    bool ok;
    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA + 1; motor++) {
        setpoint[ui.fpga->value()][motor] = setpoint_slider_widget[motor]->value()
                                            * scale_widget[motor]->text().toInt(&ok);
        setpoint_widget[motor]->setText(QString::number(setpoint[ui.fpga->value()][motor]));
        if(ok && motor<NUMBER_OF_MOTORS_PER_FPGA) {
            msg.motors.push_back(motor);
            msg.setPoints.push_back(setpoint[ui.fpga->value()][motor]);
        }
    }
    if(msg.motors.size()>0)
        motorCommand.publish(msg);
}

void RoboyMotorCommand::setPointAllChanged(int){
    roboy_communication_middleware::MotorCommand msg;
    bool ok;
    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA + 1; motor++) {
        setpoint[ui.fpga->value()][motor] = setpoint_slider_widget[NUMBER_OF_MOTORS_PER_FPGA]->value()
                                            * scale_widget[NUMBER_OF_MOTORS_PER_FPGA]->text().toInt(&ok);
        setpoint_widget[motor]->setText(QString::number(setpoint[ui.fpga->value()][motor]));
        setpoint_slider_widget[motor]->setValue(setpoint[ui.fpga->value()][motor]
                                                /scale_widget[NUMBER_OF_MOTORS_PER_FPGA]->text().toInt(&ok));
        if(ok && motor<NUMBER_OF_MOTORS_PER_FPGA) {
            msg.motors.push_back(motor);
            msg.setPoints.push_back(setpoint[ui.fpga->value()][motor]);
        }
    }
    if(msg.motors.size()>0)
        motorCommand.publish(msg);
}

void RoboyMotorCommand::fpgaChanged(int){
    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA + 1; motor++) {
        setpoint_widget[motor]->setText(QString::number(setpoint[ui.fpga->value()][motor]));
        setpoint_slider_widget[motor]->setValue(setpoint[ui.fpga->value()][motor]);
    }
}

void RoboyMotorCommand::scaleChanged(){
    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA + 1; motor++) {
        bool ok;
        scale[ui.fpga->value()][motor] = scale_widget[motor]->text().toInt(&ok);
    }
}

void RoboyMotorCommand::scaleChangedAll(){
    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA + 1; motor++) {
        bool ok;
        scale[ui.fpga->value()][motor] = scale_widget[NUMBER_OF_MOTORS_PER_FPGA]->text().toInt(&ok);
        scale_widget[motor]->setText(QString::number(scale[ui.fpga->value()][motor]));
    }
}

void RoboyMotorCommand::controlModeChanged(){
    roboy_communication_middleware::ControlMode msg;
    if(ui.pos->isChecked()) {
        control_mode[ui.fpga->value()] = POSITION;
        msg.request.control_mode = POSITION;
    }
    if(ui.vel->isChecked()) {
        control_mode[ui.fpga->value()] = VELOCITY;
        msg.request.control_mode = VELOCITY;
    }
    if(ui.dis->isChecked()) {
        control_mode[ui.fpga->value()] = DISPLACEMENT;
        msg.request.control_mode = DISPLACEMENT;
    }

    bool ok;
    msg.request.setPoint = setpoint_slider_widget[NUMBER_OF_MOTORS_PER_FPGA]->value()
                           * scale_widget[NUMBER_OF_MOTORS_PER_FPGA]->text().toInt(&ok);
    motorControl.call(msg);
}

PLUGINLIB_DECLARE_CLASS(roboy_motor_command, RoboyMotorCommand, RoboyMotorCommand, rqt_gui_cpp::Plugin)
