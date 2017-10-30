#include <roboy_motor_calibration/roboy_motor_calibration.hpp>

map<int,vector<float>> RoboyMotorCalibration::coeffs;

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
    button["load_motor_config"] = widget_->findChild<QPushButton *>("load_motor_config");

    text["data_points"] = widget_->findChild<QLineEdit *>("data_points");
    text["timeout"] = widget_->findChild<QLineEdit *>("timeout");
    text["degree"] = widget_->findChild<QLineEdit *>("degree");
    text["displacement_min"] = widget_->findChild<QLineEdit *>("displacement_min");
    text["displacement_max"] = widget_->findChild<QLineEdit *>("displacement_max");
    text["motor_config_path"] = widget_->findChild<QLineEdit *>("motor_config_path");

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
    QObject::connect(button["load_config"], SIGNAL(clicked()), this, SLOT(loadConfig()));

    ui.load->addGraph();
    ui.load->graph(0)->setPen(QPen(color_pallette[0]));
    ui.load->addGraph();
    ui.load->graph(1)->setPen(QPen(color_pallette[3]));

    ui.adc_value->addGraph();
    ui.adc_value->graph(0)->setPen(QPen(color_pallette[1]));

    ui.displacement->addGraph();
    ui.displacement->graph(0)->setPen(QPen(color_pallette[2]));

    ui.calibration_data->addGraph();
    ui.calibration_data->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui.calibration_data->graph(0)->setScatterStyle(QCPScatterStyle::ssCircle);
    ui.calibration_data->graph(0)->setPen(QPen(color_pallette[0]));
    ui.calibration_data->addGraph();
    ui.calibration_data->graph(1)->setPen(QPen(color_pallette[1]));

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

    for(uint i=0;i<NUMBER_OF_MOTORS_PER_FPGA;i++){
        coeffs[i] = {0,0,0,0,0};
    }
    loadConfig();
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

bool RoboyMotorCalibration::readConfig(string filepath){
    YAML::Node config = YAML::LoadFile(filepath);
    vector<vector<float>> motor_polynomial_parameter =
            config["motor_polynomial_parameter"].as<vector<vector<float >>>();
    for (int i = 0; i < motor_polynomial_parameter.size(); i++) {
        vector<float> polynomial_parameters;
        int motor_id = 0;
        bool first_value = true;
        for(auto &val:motor_polynomial_parameter[i]){
            if(first_value) {
                motor_id = val;
                first_value = false;
            }else {
                polynomial_parameters.push_back(val);
            }
        }
        coeffs[motor_id] = polynomial_parameters;
        ROS_INFO_STREAM(motor_id << "\t" << coeffs[motor_id][0]<< "\t" << coeffs[motor_id][1]
                                << "\t" << coeffs[motor_id][2]<< "\t" << coeffs[motor_id][3]
                                 << "\t" << coeffs[motor_id][4]);
    }
    return true;
}

bool RoboyMotorCalibration::writeConfig(string filepath){
    std::ofstream fout(filepath);
    if (!fout.is_open()) {
        ROS_WARN_STREAM("Could not write config " << filepath);
        return false;
    }
    YAML::Node config;
    for (auto const &coefficients : coeffs) {
        stringstream str;
        str << "[";
        for(uint i=0;i<=coefficients.second.size();i++){
            if( i <coefficients.second.size())
                str << "0,";
            else
                str << "0]";
        }
        YAML::Node node = YAML::Load(str.str());
        // first number is the sensor id
        node[0] = coefficients.first;
        for (int i = 0; i < coefficients.second.size(); i++) {
            node[i+1] = coefficients.second[i];
        }
        config["motor_polynomial_parameter"].push_back(node);
        ROS_INFO_STREAM(coefficients.first << "\t" << coeffs[coefficients.first][0]<< "\t" << coeffs[coefficients.first][1]
                                 << "\t" << coeffs[coefficients.first][2]<< "\t" << coeffs[coefficients.first][3]
        << "\t" << coeffs[coefficients.first][4]);
    }

    fout << config;
    return true;
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
        msg.request.degree = text["degree"]->text().toInt();
        msg.request.numberOfDataPoints = text["data_points"]->text().toInt();
        msg.request.timeout = text["timeout"]->text().toInt();
        msg.request.displacement_min = text["displacement_min"]->text().toInt();
        msg.request.displacement_max = text["displacement_max"]->text().toInt();
        motorCalibration.call(msg);
        for (auto const &coefficients : coeffs) {
            ROS_INFO_STREAM(coefficients.first << "\t" << coeffs[coefficients.first][0]<< "\t" << coeffs[coefficients.first][1]
                                               << "\t" << coeffs[coefficients.first][2]<< "\t" << coeffs[coefficients.first][3]);
        }
        coeffs[ui.motor->value()] = msg.response.estimated_spring_parameters;

        for (auto const &coefficients : coeffs) {
            ROS_INFO_STREAM(coefficients.first << "\t" << coeffs[coefficients.first][0]<< "\t" << coeffs[coefficients.first][1]
                                               << "\t" << coeffs[coefficients.first][2]<< "\t" << coeffs[coefficients.first][3]);
        }


        QVector<double> load = QVector<double>::fromStdVector( msg.response.load );
        QVector<double> displacement = QVector<double>::fromStdVector( msg.response.displacement );

        ui.calibration_data->graph(0)->setData(displacement, load);

        QVector<double> load_graph;
        for(uint i=0;i<displacement.size();i++){
            double val = msg.response.estimated_spring_parameters[0];
            for(uint j=1;j<msg.response.estimated_spring_parameters.size();j++){
                val += msg.response.estimated_spring_parameters[j]*pow(displacement[i],(double)j);
            }
            load_graph.push_back(val);
        }

        ui.calibration_data->graph(1)->setData(displacement, load_graph);
        ui.calibration_data->graph(0)->rescaleAxes();
        ui.calibration_data->replot();

        writeConfig(text["motor_config_path"]->text().toStdString());
    }
}

void RoboyMotorCalibration::MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
    lock_guard<mutex> lock(mux);
    ROS_INFO_THROTTLE(5, "receiving motor status");
    timeMotorData.push_back(counter++);
    motorData.push_back(msg->displacement[ui.motor->value()]);
    if (motorData.size() > samples_per_plot) {
        motorData.pop_front();
    }

    float calibrated_value = coeffs[ui.motor->value()][0];
    for(uint coeff=1;coeff<coeffs[ui.motor->value()].size();coeff++){
        calibrated_value += coeffs[ui.motor->value()][coeff]*pow(motorData.back(),coeff);
    }

    motorDataCalibrated.push_back(calibrated_value);
    if (motorDataCalibrated.size() > samples_per_plot) {
        motorDataCalibrated.pop_front();
    }

    if (timeMotorData.size() > samples_per_plot)
        timeMotorData.pop_front();

    if (counter % 10 == 0)
            Q_EMIT newData();
}

void RoboyMotorCalibration::ADCvalue(const roboy_communication_middleware::ADCvalue::ConstPtr &msg) {
    ROS_INFO_THROTTLE(5, "receiving load_cell status");
    lock_guard<mutex> lock(mux);
    time.push_back(counter++);
    loadCellLoad.push_back(msg->load[0]); // TODO: ui.motor->value()
    if (loadCellLoad.size() > samples_per_plot) {
        loadCellLoad.pop_front();
    }

    loadCellValue.push_back(msg->adc_value[0]); // TODO: ui.motor->value()
    if (loadCellValue.size() > samples_per_plot) {
        loadCellValue.pop_front();
    }
    if (time.size() > samples_per_plot)
        time.pop_front();

    if (counter % 10 == 0)
            Q_EMIT newData();
}

void RoboyMotorCalibration::plotData() {
    lock_guard<mutex> lock(mux);
    ui.load->graph(0)->setData(time, loadCellLoad);
    ui.load->graph(1)->setData(time, motorDataCalibrated);
    ui.load->xAxis->rescale();

    ui.adc_value->graph(0)->setData(time, loadCellValue);
    ui.adc_value->xAxis->rescale();

    ui.displacement->graph(0)->setData(timeMotorData, motorData);
    ui.displacement->graph(0)->rescaleAxes();

    ui.load->replot();
    ui.adc_value->replot();
    ui.displacement->replot();
}

void RoboyMotorCalibration::loadConfig(){
    QFileInfo check_file(text["motor_config_path"]->text());
    if (check_file.exists() && check_file.isFile()) {
        ROS_INFO("reading motor config from %s", text["motor_config_path"]->text().toStdString().c_str());
        readConfig(text["motor_config_path"]->text().toStdString());
    }else{
        ROS_ERROR("file %s does not exist, check your path",text["motor_config_path"]->text().toStdString().c_str() );
    }

}

PLUGINLIB_DECLARE_CLASS(roboy_motor_calibration, RoboyMotorCalibration, RoboyMotorCalibration, rqt_gui_cpp::Plugin)
