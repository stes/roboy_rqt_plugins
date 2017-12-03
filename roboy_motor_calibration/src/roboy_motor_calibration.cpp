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
    button["load_motor_config"] = widget_->findChild<QPushButton *>("load_motor_config");
    button["fit_curve"] = widget_->findChild<QPushButton *>("fit_curve");

    button["fit_curve"]->setToolTip("estimates spring parameters read from csv file");

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

    ui.stop_button_all->setStyleSheet("background-color: green");
    QObject::connect(button["stop_button_all"], SIGNAL(clicked()), this, SLOT(stopButtonAllClicked()));
    QObject::connect(button["calibrate"], SIGNAL(clicked()), this, SLOT(MotorCalibration()));
    QObject::connect(button["load_config"], SIGNAL(clicked()), this, SLOT(loadConfig()));
    QObject::connect(button["fit_curve"], SIGNAL(clicked()), this, SLOT(fitCurve()));

    // myoMuscle TAB
    ui.load->addGraph();
    ui.load->graph(0)->setPen(QPen(color_pallette[0])); // measured
    ui.load->addGraph();
    ui.load->graph(1)->setPen(QPen(color_pallette[3])); // estimated from sensors

    ui.load->xAxis->setLabel("x");
    ui.load->yAxis->setLabel("Newton");
    ui.load->yAxis->setRange(0, 500);
    ui.load->replot();

    ui.adc_value->addGraph();
    ui.adc_value->graph(0)->setPen(QPen(color_pallette[1]));

    ui.adc_value->xAxis->setLabel("x");
    ui.adc_value->yAxis->setLabel("adc_value");
    ui.adc_value->yAxis->setRange(0, 4095);
    ui.adc_value->replot();

    ui.displacement->addGraph();
    ui.displacement->graph(0)->setPen(QPen(color_pallette[2]));
    ui.displacement->xAxis->setLabel("x");
    ui.displacement->yAxis->setLabel("displacement");
    ui.displacement->replot();

    // muscleMuscle TAB
    ui.load_2->addGraph();
    ui.load_2->graph(0)->setPen(QPen(color_pallette[0])); // measured
    ui.load_2->addGraph();
    ui.load_2->graph(1)->setPen(QPen(color_pallette[3])); // estimated from sensors

    ui.load_2->xAxis->setLabel("x");
    ui.load_2->yAxis->setLabel("Newton");
    ui.load_2->yAxis->setRange(0, 100);
    ui.load_2->replot();

    ui.motorAngle->addGraph();
    ui.motorAngle->graph(0)->setPen(QPen(color_pallette[1]));

    ui.motorAngle->xAxis->setLabel("x");
    ui.motorAngle->yAxis->setLabel("motor angle [degree]");
    ui.motorAngle->yAxis->setRange(0, 360);
    ui.motorAngle->replot();

    ui.motorPosition->addGraph();
    ui.motorPosition->graph(0)->setPen(QPen(color_pallette[2]));
    ui.motorPosition->xAxis->setLabel("x");
    ui.motorPosition->yAxis->setLabel("motor position [degree]");
    ui.motorPosition->replot();

    // estimated curva
    ui.displacement_force->addGraph();
    ui.displacement_force->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui.displacement_force->graph(0)->setScatterStyle(QCPScatterStyle::ssCircle);
    ui.displacement_force->graph(0)->setPen(QPen(color_pallette[0]));
    ui.displacement_force->addGraph();
    ui.displacement_force->graph(1)->setPen(QPen(color_pallette[1]));
    ui.displacement_force->xAxis->setLabel("displacement[ticks]");
    ui.displacement_force->yAxis->setLabel("force[N]");

    ui.force_displacement->addGraph();
    ui.force_displacement->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui.force_displacement->graph(0)->setScatterStyle(QCPScatterStyle::ssCircle);
    ui.force_displacement->graph(0)->setPen(QPen(color_pallette[0]));
    ui.force_displacement->addGraph();
    ui.force_displacement->graph(1)->setPen(QPen(color_pallette[1]));
    ui.force_displacement->xAxis->setLabel("force[N]");
    ui.force_displacement->yAxis->setLabel("displacement[ticks]");


    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));

    for(uint i=0;i<NUMBER_OF_MOTORS_PER_FPGA;i++){
        coeffs_displacement2force[i] = {0,0,0,0,0};
        coeffs_force2displacement[i] = {0,0,0,0,0};
    }
    loadConfig();

    motorStatus = nh->subscribe("/roboy/middleware/MotorStatus", 1, &RoboyMotorCalibration::MotorStatus, this);
    motorAngle = nh->subscribe("/roboy/middleware/MotorAngle", 1, &RoboyMotorCalibration::MotorAngle, this);
    loadCells = nh->subscribe("/roboy/middleware/LoadCells", 1, &RoboyMotorCalibration::ADCvalue, this);
    motorCalibration = nh->serviceClient<roboy_communication_middleware::MotorCalibrationService>("/roboy/middleware/MotorCalibration");
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
        msg.request.degree = text["degree"]->text().toInt();
        msg.request.numberOfDataPoints = text["data_points"]->text().toInt();
        msg.request.timeout = text["timeout"]->text().toInt();
        msg.request.displacement_min = text["displacement_min"]->text().toInt();
        msg.request.displacement_max = text["displacement_max"]->text().toInt();
        motorCalibration.call(msg);
        estimateSpringParameters(msg.response.load, msg.response.displacement,
                                 coeffs_displacement2force[msg.request.motor],
                                 coeffs_force2displacement[msg.request.motor]);

        writeConfig(text["motor_config_path"]->text().toStdString());
    }
}

void RoboyMotorCalibration::MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
    lock_guard<mutex> lock(mux);
    ROS_INFO_THROTTLE(5, "receiving motor status");
    timeMotorData[POSITION].push_back(counter);
    motorData[POSITION].push_back(msg->position[ui.motor->value()]/1024.0f/62.0f*360.0f); // 1024 ticks per turn / gear box ratio * 360 degrees
    if (motorData[POSITION].size() > samples_per_plot) {
        motorData[POSITION].pop_front();
    }

    timeMotorData[DISPLACEMENT].push_back(counter);
    motorData[DISPLACEMENT].push_back(msg->displacement[ui.motor->value()]);
    if (motorData[DISPLACEMENT].size() > samples_per_plot) {
        motorData[DISPLACEMENT].pop_front();
    }

    if (timeMotorData[DISPLACEMENT].size() > samples_per_plot)
        timeMotorData[DISPLACEMENT].pop_front();

    if (timeMotorData[POSITION].size() > samples_per_plot)
        timeMotorData[POSITION].pop_front();

    counter++;

    if (counter % 10 == 0)
            Q_EMIT newData();
}

void RoboyMotorCalibration::MotorAngle(const roboy_communication_middleware::MotorAngle::ConstPtr &msg) {
    lock_guard<mutex> lock(mux);
    ROS_INFO_THROTTLE(5, "receiving motor angle");
    timeMotorData[ANGLE].push_back(counter++);
    motorData[ANGLE].push_back(msg->angles[ui.motor->value()]);
    if (motorData[ANGLE].size() > samples_per_plot) {
        motorData[ANGLE].pop_front();
    }

    if (timeMotorData[ANGLE].size() > samples_per_plot)
        timeMotorData[ANGLE].pop_front();

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

void RoboyMotorCalibration::polynomialRegression(int degree, vector<double> &x, vector<double> &y,
                                      vector<float> &coeffs) {
    int N = x.size(), i, j, k;
    double X[2 * degree +
             1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    for (i = 0; i < 2 * degree + 1; i++) {
        X[i] = 0;
        for (j = 0; j < N; j++)
            X[i] = X[i] + pow(x[j],
                              i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    }
    double B[degree + 1][degree + 2], a[degree +
                                        1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
    for (i = 0; i <= degree; i++)
        for (j = 0; j <= degree; j++)
            B[i][j] = X[i +
                        j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
    double Y[degree +
             1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^degree*yi)
    for (i = 0; i < degree + 1; i++) {
        Y[i] = 0;
        for (j = 0; j < N; j++)
            Y[i] = Y[i] + pow(x[j], i) *
                          y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    }
    for (i = 0; i <= degree; i++)
        B[i][degree +
             1] = Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
    degree = degree +
             1;                //degree is made degree+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
    for (i = 0; i <
                degree; i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
        for (k = i + 1; k < degree; k++)
            if (B[i][i] < B[k][i])
                for (j = 0; j <= degree; j++) {
                    double temp = B[i][j];
                    B[i][j] = B[k][j];
                    B[k][j] = temp;
                }

    for (i = 0; i < degree - 1; i++)            //loop to perform the gauss elimination
        for (k = i + 1; k < degree; k++) {
            double t = B[k][i] / B[i][i];
            for (j = 0; j <= degree; j++)
                B[k][j] = B[k][j] - t *
                                    B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
        }
    for (i = degree - 1; i >= 0; i--)                //back-substitution
    {                        //x is an array whose values correspond to the values of x,y,z..
        a[i] = B[i][degree];                //make the variable to be calculated equal to the rhs of the last equation
        for (j = 0; j < degree; j++)
            if (j !=
                i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                a[i] = a[i] - B[i][j] * a[j];
        a[i] = a[i] /
               B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
    }
    for (i = 0; i < degree; i++)
        coeffs.push_back(a[i]);    //the values of x^0,x^1,x^2,x^3,....
}

void RoboyMotorCalibration::estimateSpringParameters(vector<double> &force,
                                                     vector<double> &displacement,
                                                     vector<float> &coefficients_displacement_force,
                                                     vector<float> &coefficients_force_displacement){
    polynomialRegression(ui.degree->text().toInt(), displacement, force, coefficients_displacement_force);

    QVector<double> load = QVector<double>::fromStdVector( force );
    QVector<double> dis = QVector<double>::fromStdVector( displacement );

    ui.displacement_force->graph(0)->setData(dis, load);

    QVector<double> load_graph;
    for(uint i=0;i<displacement.size();i++){
        double val = coefficients_displacement_force[0];
        for(uint j=1;j<coefficients_displacement_force.size();j++){
            val += coefficients_displacement_force[j]*pow(displacement[i],(double)j);
        }
        load_graph.push_back(val);
    }

    ui.displacement_force->graph(1)->setData(dis, load_graph);
    ui.displacement_force->graph(0)->rescaleAxes();
    ui.displacement_force->replot();

    polynomialRegression(ui.degree->text().toInt(), force, displacement, coefficients_force_displacement);

    ui.force_displacement->graph(0)->setData( load, dis );

    QVector<double> displacement_graph;
    for(uint i=0;i<displacement.size();i++){
        double val = coefficients_force_displacement[0];
        for(uint j=1;j<coefficients_force_displacement.size();j++){
            val += coefficients_force_displacement[j]*pow(force[i],(double)j);
        }
        displacement_graph.push_back(val);
    }

    ui.force_displacement->graph(1)->setData(load, displacement_graph);
    ui.force_displacement->graph(0)->rescaleAxes();
    ui.force_displacement->replot();
}

void RoboyMotorCalibration::plotData() {
    lock_guard<mutex> lock(mux);
    switch(ui.tabWidget->currentIndex()){
        case 0: {
            float calibrated_value = coeffs_displacement2force[ui.motor->value()][0];
            for (uint coeff = 1; coeff < coeffs_displacement2force[ui.motor->value()].size(); coeff++) {
                calibrated_value +=
                        coeffs_displacement2force[ui.motor->value()][coeff] * pow(motorData[POSITION].back(), coeff);
            }

            motorDataCalibrated[MYOMUSLCE].push_back(calibrated_value);
            if (motorDataCalibrated[MYOMUSLCE].size() > samples_per_plot) {
                motorDataCalibrated[MYOMUSLCE].pop_front();
            }
            ui.load->graph(0)->setData(time, loadCellLoad);
            ui.load->graph(1)->setData(time, motorDataCalibrated[MYOMUSLCE]);
            ui.load->xAxis->rescale();

            ui.adc_value->graph(0)->setData(time, loadCellValue);
//            ui.adc_value->graph(0)->rescaleAxes();
            ui.adc_value->xAxis->rescale();

            ui.displacement->graph(0)->setData(timeMotorData[DISPLACEMENT], motorData[DISPLACEMENT]);
            ui.displacement->graph(0)->rescaleAxes();

            ui.load->replot();
            ui.adc_value->replot();
            ui.displacement->replot();
            break;
        }
        case 1:{
            // TODO: how do motor angle and motor position result in the effective position

//            motorDataCalibrated[MYOMUSLCE].push_back(calibrated_value);
//            if (motorDataCalibrated[MYOMUSLCE].size() > samples_per_plot) {
//                motorDataCalibrated[MYOMUSLCE].pop_front();
//            }

            ui.load_2->graph(0)->setData(time, loadCellLoad);
//            ui.load_2->graph(1)->setData(time, motorDataCalibrated);
            ui.load_2->xAxis->rescale();

            ui.motorAngle->graph(0)->setData(timeMotorData[ANGLE], motorData[ANGLE]);
            ui.motorAngle->xAxis->rescale();
//            ui.motorAngle->graph(0)->rescaleAxes();

            ui.motorPosition->graph(0)->setData(timeMotorData[POSITION], motorData[POSITION]);
            ui.motorPosition->graph(0)->rescaleAxes();

            ui.load_2->replot();
            ui.motorAngle->replot();
            ui.motorPosition->replot();
            break;
        }
    }

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

void RoboyMotorCalibration::fitCurve(){
    char str[200];
    sprintf(str, "About to overwrite existing spring parameters for motor %d", ui.motor->value());
    QMessageBox::information(widget_, tr(str),
                             "Make sure you have selected the correct motor in the GUI");

    QString fileName = QFileDialog::getOpenFileName(widget_,tr("Open Calibration CSV file"), "",
                                                    tr("Calibration Data (*.csv);;All Files (*)"));
    if (fileName.isEmpty()) {
        ROS_ERROR("could not open file");
        return;
    }else {
        vector<double> dis, force;

        ifstream infile(fileName.toStdString().c_str());
        string line;
        while (getline(infile, line))
        {
            char *pt;
            pt = strtok ((char*)line.c_str(),",");
            bool displacement = true;
            float val0, val1;
            int counter = 0;
            while (pt != NULL) {
                counter++;
                std::istringstream istr(pt);
                istr.imbue(std::locale("C"));
                if(displacement) {
                    istr >> val0;
                    displacement = false;
                }else {
                    istr >> val1;
                }
                pt = strtok (NULL, ",");
            }
            if(counter==2){
                dis.push_back(val0);
                force.push_back(val1);
            }
        }
        coeffs_displacement2force[ui.motor->value()].clear();
        coeffs_force2displacement[ui.motor->value()].clear();
        estimateSpringParameters(force, dis,
                                 coeffs_displacement2force[ui.motor->value()],
                                 coeffs_force2displacement[ui.motor->value()]);
        writeConfig(text["motor_config_path"]->text().toStdString());
    }
}

PLUGINLIB_DECLARE_CLASS(roboy_motor_calibration, RoboyMotorCalibration, RoboyMotorCalibration, rqt_gui_cpp::Plugin)
