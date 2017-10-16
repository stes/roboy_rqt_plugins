#include <pabiroboy_joint_control/pabiroboy_joint_control.hpp>

PaBiRoboyJointControl::PaBiRoboyJointControl()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("PaBiRoboyJointControl");
}

void PaBiRoboyJointControl::initPlugin(qt_gui_cpp::PluginContext &context) {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));

    text["joint_setpoint_0"] = widget_->findChild<QLineEdit *>("joint_setpoint_text_0");
    text["joint_setpoint_1"] = widget_->findChild<QLineEdit *>("joint_setpoint_text_1");
    text["joint_setpoint_2"] = widget_->findChild<QLineEdit *>("joint_setpoint_text_2");
    text["joint_setpoint_3"] = widget_->findChild<QLineEdit *>("joint_setpoint_text_3");
    text["joint_setpoint_all"] = widget_->findChild<QLineEdit *>("joint_setpoint_text_all");
    text["Kp"] = widget_->findChild<QLineEdit *>("Kp");
    text["Ki"] = widget_->findChild<QLineEdit *>("Ki");
    text["Kd"] = widget_->findChild<QLineEdit *>("Kd");

    flexor[0] = widget_->findChild<QLineEdit *>("flexor_0");
    flexor[1] = widget_->findChild<QLineEdit *>("flexor_1");
    flexor[2] = widget_->findChild<QLineEdit *>("flexor_2");
    flexor[3] = widget_->findChild<QLineEdit *>("flexor_3");
    extensor[0] = widget_->findChild<QLineEdit *>("extensor_0");
    extensor[1] = widget_->findChild<QLineEdit *>("extensor_1");
    extensor[2] = widget_->findChild<QLineEdit *>("extensor_2");
    extensor[3] = widget_->findChild<QLineEdit *>("extensor_3");

    slider["joint_setpoint_0"] = widget_->findChild<QSlider *>("joint_setpoint_0");
    slider["joint_setpoint_1"] = widget_->findChild<QSlider *>("joint_setpoint_1");
    slider["joint_setpoint_2"] = widget_->findChild<QSlider *>("joint_setpoint_2");
    slider["joint_setpoint_3"] = widget_->findChild<QSlider *>("joint_setpoint_3");
    slider["joint_setpoint_all"] = widget_->findChild<QSlider *>("joint_setpoint_all");

    button["activate"] = widget_->findChild<QPushButton *>("activate");
    button["activate"]->setStyleSheet("background-color: green");

    // add graphs to plot for setpoint an current angle
    ui.joint0->addGraph();
    ui.joint0->graph(0)->setPen(QPen(color_pallette[0]));
    ui.joint0->addGraph();
    ui.joint0->graph(1)->setPen(QPen(color_pallette[1]));
    ui.joint0->yAxis->setLabel("degrees");
    ui.joint0->yAxis->setRange(-360, 360);

    ui.joint1->addGraph();
    ui.joint1->graph(0)->setPen(QPen(color_pallette[0]));
    ui.joint1->addGraph();
    ui.joint1->graph(1)->setPen(QPen(color_pallette[1]));
    ui.joint1->yAxis->setLabel("degrees");
    ui.joint1->yAxis->setRange(-360, 360);

    ui.joint2->addGraph();
    ui.joint2->graph(0)->setPen(QPen(color_pallette[0]));
    ui.joint2->addGraph();
    ui.joint2->graph(1)->setPen(QPen(color_pallette[1]));
    ui.joint2->yAxis->setLabel("degrees");
    ui.joint2->yAxis->setRange(-360, 360);

    ui.joint3->addGraph();
    ui.joint3->graph(0)->setPen(QPen(color_pallette[0]));
    ui.joint3->addGraph();
    ui.joint3->graph(1)->setPen(QPen(color_pallette[1]));
    ui.joint3->yAxis->setLabel("degrees");
    ui.joint3->yAxis->setRange(-360, 360);

    // connect slider to callback
    QObject::connect(slider["joint_setpoint_0"], SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControl()));
    QObject::connect(slider["joint_setpoint_1"], SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControl()));
    QObject::connect(slider["joint_setpoint_2"], SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControl()));
    QObject::connect(slider["joint_setpoint_3"], SIGNAL(valueChanged(int)), this, SLOT(updateSetPointsJointControl()));
    QObject::connect(slider["joint_setpoint_all"], SIGNAL(valueChanged(int)), this,
                     SLOT(updateSetPointsJointControlAll()));

    QObject::connect(ui.activate, SIGNAL(clicked()), this, SLOT(activate()));

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "pabiroboy_joint_control_rqt_plugin");
    }

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    motorCommand = nh->advertise<roboy_communication_middleware::MotorCommand>("/roboy/middleware/MotorCommand", 1);
    jointStatus = nh->subscribe("/roboy/middleware/JointStatus", 1, &PaBiRoboyJointControl::JointStatus, this);

    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));
}

void PaBiRoboyJointControl::shutdownPlugin() {
    motorCommand.shutdown();
}

void PaBiRoboyJointControl::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                         qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void PaBiRoboyJointControl::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                            const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void PaBiRoboyJointControl::JointStatus(const roboy_communication_middleware::JointStatus::ConstPtr &msg) {
    ROS_INFO_THROTTLE(5, "receiving joint status");
    time.push_back(counter++);

    jointDataSetpoint[0].push_back(slider["joint_setpoint_0"]->value());
    jointDataSetpoint[1].push_back(slider["joint_setpoint_1"]->value());
    jointDataSetpoint[2].push_back(slider["joint_setpoint_2"]->value());
    jointDataSetpoint[3].push_back(slider["joint_setpoint_3"]->value());

    for (uint joint = 0; joint < NUMBER_OF_JOINTS; joint++) {
        jointData[joint].push_back(msg->absAngles[joint]/4096.0*360);
        if (jointData[joint].size() > samples_per_plot) {
            jointData[joint].pop_front();
        }
        if (jointDataSetpoint[joint].size() > samples_per_plot) {
            jointDataSetpoint[joint].pop_front();
        }
    }
    if (time.size() > samples_per_plot)
        time.pop_front();

    if (counter % 3 == 0)
            Q_EMIT newData();

    if(joint_control_active)
        JointController();
}

void PaBiRoboyJointControl::plotData() {
    ui.joint0->graph(0)->setData(time, jointData[0]);
    ui.joint1->graph(0)->setData(time, jointData[1]);
    ui.joint2->graph(0)->setData(time, jointData[2]);
    ui.joint3->graph(0)->setData(time, jointData[3]);

    ui.joint0->graph(1)->setData(time, jointDataSetpoint[0]);
    ui.joint1->graph(1)->setData(time, jointDataSetpoint[1]);
    ui.joint2->graph(1)->setData(time, jointDataSetpoint[2]);
    ui.joint3->graph(1)->setData(time, jointDataSetpoint[3]);

    ui.joint0->xAxis->rescale();
    ui.joint1->xAxis->rescale();
    ui.joint2->xAxis->rescale();
    ui.joint3->xAxis->rescale();

    ui.joint0->replot();
    ui.joint1->replot();
    ui.joint2->replot();
    ui.joint3->replot();
}

void PaBiRoboyJointControl::activate() {
    if (button["activate"]->isChecked()) {
        button["activate"]->setStyleSheet("background-color: red");
        joint_control_active = true;
    } else {
        button["activate"]->setStyleSheet("background-color: green");
        joint_control_active = false;
    }
}

void PaBiRoboyJointControl::updateSetPointsJointControl() {
    text["joint_setpoint_0"]->setText(QString::number(slider["joint_setpoint_0"]->value()));
    text["joint_setpoint_1"]->setText(QString::number(slider["joint_setpoint_1"]->value()));
    text["joint_setpoint_2"]->setText(QString::number(slider["joint_setpoint_2"]->value()));
    text["joint_setpoint_3"]->setText(QString::number(slider["joint_setpoint_3"]->value()));
}

void PaBiRoboyJointControl::updateSetPointsJointControlAll() {
    text["joint_setpoint_0"]->setText(QString::number(slider["joint_setpoint_all"]->value()));
    text["joint_setpoint_1"]->setText(QString::number(slider["joint_setpoint_all"]->value()));
    text["joint_setpoint_2"]->setText(QString::number(slider["joint_setpoint_all"]->value()));
    text["joint_setpoint_3"]->setText(QString::number(slider["joint_setpoint_all"]->value()));
    text["joint_setpoint_all"]->setText(QString::number(slider["joint_setpoint_all"]->value()));

    slider["joint_setpoint_0"]->setValue(slider["joint_setpoint_all"]->value());
    slider["joint_setpoint_1"]->setValue(slider["joint_setpoint_all"]->value());
    slider["joint_setpoint_2"]->setValue(slider["joint_setpoint_all"]->value());
    slider["joint_setpoint_3"]->setValue(slider["joint_setpoint_all"]->value());
}

void PaBiRoboyJointControl::JointController() {
    ROS_INFO_THROTTLE(5, "joint control active");
    float error[NUMBER_OF_JOINTS];
    float integral[NUMBER_OF_JOINTS];
    float integral_max = 360;
    const float smooth_distance = 50;
    const float offset = 20;
    static float error_previous[NUMBER_OF_JOINTS] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool ok;
    roboy_communication_middleware::MotorCommand msg;
    for (uint joint = 0; joint < NUMBER_OF_JOINTS; joint++) {
        error[joint] = jointDataSetpoint[joint].back() - jointData[joint].back();

        bool flexor_id_valid_ok, extensor_id_valid_ok;
        int flexor_muscle = flexor[joint]->text().toInt(&flexor_id_valid_ok);
        int extensor_muscle = extensor[joint]->text().toInt(&extensor_id_valid_ok);

        msg.motors.push_back(flexor_muscle);
        msg.motors.push_back(extensor_muscle);

        float pterm = text["Kp"]->text().toFloat(&ok) * error[joint];
        float dterm = text["Kd"]->text().toFloat(&ok) *
                      (error[joint] - error_previous[joint]);
        integral[joint] += text["Ki"]->text().toFloat(&ok) * error[joint];
        if (integral[joint] >= integral_max) {
            integral[joint] = integral_max;
        } else if (integral[joint] <= integral_max) {
            integral[joint] = -integral_max;
        }
        float result = pterm + dterm + integral[joint];
        if (result <= -smooth_distance) {
            msg.setPoints.push_back(offset - result);
            msg.setPoints.push_back(offset);
        } else if (result < smooth_distance) {
            msg.setPoints.push_back(offset + powf(result - smooth_distance, 2.0f) /
                                             (4.0f * smooth_distance));
            msg.setPoints.push_back(offset + powf(result + smooth_distance, 2.0f) /
                                             (4.0f * smooth_distance));
        } else {
            msg.setPoints.push_back(offset);
            msg.setPoints.push_back(offset + result);
        }
        error_previous[joint] = error[joint];
    }
    motorCommand.publish(msg);
}

PLUGINLIB_DECLARE_CLASS(pabiroboy_joint_control, PaBiRoboyJointControl, PaBiRoboyJointControl, rqt_gui_cpp::Plugin)
