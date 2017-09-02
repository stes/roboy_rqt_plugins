#include <roboy_motor_command/roboy_motor_command.hpp>

RoboyMotorCommand::RoboyMotorCommand()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("RoboyMotorCommand");

    for(uint fpga = 0; fpga<NUMBER_OF_FPGAS; fpga++) {
        stopButton[fpga]= false;
    }

//    for(uint fpga = 0; fpga<NUMBER_OF_FPGAS; fpga++) {
//        for (uint motor = 0; motor < 15; motor++) {
//            stopButton[fpga][motor] = false;
//        }
//    }
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

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "motor_command_rqt_plugin");
    }

    motorCommand = nh->advertise<roboy_communication_middleware::MotorCommand>("/roboy/middleware/MotorCommand", 1);

    ui.stop_button->setStyleSheet("background-color: green");
    ui.stop_button_all->setStyleSheet("background-color: green");
    QObject::connect(ui.stop_button, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));
    QObject::connect(ui.stop_button_all, SIGNAL(clicked()), this, SLOT(stopButtonAllClicked()));
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

void RoboyMotorCommand::stopButtonClicked(){
    if(ui.stop_button->isChecked()) {
        ui.stop_button->setStyleSheet("background-color: red");
        stopButton[ui.fpga->value] = true;
    }else {
        ui.stop_button->setStyleSheet("background-color: green");
        stopButton[ui.fpga->value] = false;
    }
}

void RoboyMotorCommand::stopButtonAllClicked(){
    if(ui.stop_button_all->isChecked()) {
        ui.stop_button_all->setStyleSheet("background-color: red");
        for(uint fpga = 0; fpga<NUMBER_OF_FPGAS; fpga++) {
            stopButton[fpga]= true;
        }
    }else {
        ui.stop_button_all->setStyleSheet("background-color: green");
        for(uint fpga = 0; fpga<NUMBER_OF_FPGAS; fpga++) {
            stopButton[fpga]= false;
        }
    }
}

PLUGINLIB_DECLARE_CLASS(roboy_motor_command, RoboyMotorCommand, RoboyMotorCommand, rqt_gui_cpp::Plugin)
