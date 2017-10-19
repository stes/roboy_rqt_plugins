#include <roboy_darkroom/roboy_darkroom.hpp>

tf::Transform RoboyDarkRoom::lighthouse1;
tf::Transform RoboyDarkRoom::lighthouse2;
tf::Transform RoboyDarkRoom::tf_world;

RoboyDarkRoom::RoboyDarkRoom()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("RoboyDarkRoom");
}

RoboyDarkRoom::~RoboyDarkRoom(){
    publish_transform = false;
    for (auto const &object:trackedObjects) {
        lock_guard<mutex>(object.second->mux);
        object.second->calibrating = false;
        object.second->distances = false;
        object.second->particle_filtering = false;
        object.second->poseestimating = false;
        object.second->tracking = false;
    }
}

void RoboyDarkRoom::initPlugin(qt_gui_cpp::PluginContext &context) {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    text["lighthouse2_x"] = widget_->findChild<QLineEdit *>("lighthouse2_x_text");
    text["lighthouse2_y"] = widget_->findChild<QLineEdit *>("lighthouse2_y_text");
    text["lighthouse2_z"] = widget_->findChild<QLineEdit *>("lighthouse2_z_text");
    text["lighthouse2_roll"] = widget_->findChild<QLineEdit *>("lighthouse2_roll_text");
    text["lighthouse2_pitch"] = widget_->findChild<QLineEdit *>("lighthouse2_pitch_text");
    text["lighthouse2_yaw"] = widget_->findChild<QLineEdit *>("lighthouse2_yaw_text");
    text["broadcast_ip"] = widget_->findChild<QLineEdit *>("broadcast_ip");
    text["broadcast_port"] = widget_->findChild<QLineEdit *>("broadcast_port");

    slider["lighthouse2_x"] = widget_->findChild<QSlider *>("lighthouse2_x");
    slider["lighthouse2_y"] = widget_->findChild<QSlider *>("lighthouse2_y");
    slider["lighthouse2_z"] = widget_->findChild<QSlider *>("lighthouse2_z");
    slider["lighthouse2_roll"] = widget_->findChild<QSlider *>("lighthouse2_roll");
    slider["lighthouse2_pitch"] = widget_->findChild<QSlider *>("lighthouse2_pitch");
    slider["lighthouse2_yaw"] = widget_->findChild<QSlider *>("lighthouse2_yaw");

    button["triangulate"] = widget_->findChild<QPushButton *>("triangulate");
    button["show_rays"] = widget_->findChild<QPushButton *>("show_rays");
    button["show_distances"] = widget_->findChild<QPushButton *>("show_distances");
    button["pose_correction_sensor_cloud"] = widget_->findChild<QPushButton *>("pose_correction_sensor_cloud");
    button["pose_correction_particle_filter"] = widget_->findChild<QPushButton *>("pose_correction_particle_filter");
    button["position_estimation_relativ_sensor_distances"] = widget_->findChild<QPushButton *>("position_estimation_relativ_sensor_distances");
    button["reset_lighthouse_poses"] = widget_->findChild<QPushButton *>("reset_lighthouse_poses");
    button["switch_lighthouses"] = widget_->findChild<QPushButton *>("switch_lighthouses");
    button["calibrate_relative_distances"] = widget_->findChild<QPushButton *>("calibrate_relative_distances");
    button["record"] = widget_->findChild<QPushButton *>("record");
    button["connect_roboy"] = widget_->findChild<QPushButton *>("connect_roboy");
    button["connect_object"] = widget_->findChild<QPushButton *>("connect_object");
    button["clear_all"] = widget_->findChild<QPushButton *>("clear_all");

    QObject::connect(text["lighthouse2_x"], SIGNAL(editingFinished()), this, SLOT(resetLighthousePoses()));
    QObject::connect(text["lighthouse2_y"], SIGNAL(editingFinished()), this, SLOT(resetLighthousePoses()));
    QObject::connect(text["lighthouse2_z"], SIGNAL(editingFinished()), this, SLOT(resetLighthousePoses()));
    QObject::connect(text["lighthouse2_roll"], SIGNAL(editingFinished()), this, SLOT(resetLighthousePoses()));
    QObject::connect(text["lighthouse2_pitch"], SIGNAL(editingFinished()), this, SLOT(resetLighthousePoses()));
    QObject::connect(text["lighthouse2_yaw"], SIGNAL(editingFinished()), this, SLOT(resetLighthousePoses()));

    QObject::connect(slider["lighthouse2_x"], SIGNAL(valueChanged(int)), this, SLOT(resetLighthousePoses()));
    QObject::connect(slider["lighthouse2_y"], SIGNAL(valueChanged(int)), this, SLOT(resetLighthousePoses()));
    QObject::connect(slider["lighthouse2_z"], SIGNAL(valueChanged(int)), this, SLOT(resetLighthousePoses()));
    QObject::connect(slider["lighthouse2_roll"], SIGNAL(valueChanged(int)), this, SLOT(resetLighthousePoses()));
    QObject::connect(slider["lighthouse2_pitch"], SIGNAL(valueChanged(int)), this, SLOT(resetLighthousePoses()));
    QObject::connect(slider["lighthouse2_yaw"], SIGNAL(valueChanged(int)), this, SLOT(resetLighthousePoses()));

    QObject::connect(button["triangulate"], SIGNAL(clicked()), this, SLOT(startTriangulation()));
    QObject::connect(button["show_rays"], SIGNAL(clicked()), this, SLOT(showRays()));
    QObject::connect(button["show_distances"], SIGNAL(clicked()), this, SLOT(showDistances()));
    QObject::connect(button["pose_correction_sensor_cloud"], SIGNAL(clicked()), this, SLOT(startPoseEstimationSensorCloud()));
    QObject::connect(button["pose_correction_particle_filter"], SIGNAL(clicked()), this, SLOT(startPoseEstimationParticleFilter()));
    QObject::connect(button["position_estimation_relativ_sensor_distances"], SIGNAL(clicked()), this, SLOT(startEstimateSensorPositionsUsingRelativeDistances()));
    QObject::connect(button["reset_lighthouse_poses"], SIGNAL(clicked()), this, SLOT(resetLighthousePoses()));
    QObject::connect(button["calibrate_relative_distances"], SIGNAL(clicked()), this, SLOT(startCalibrateRelativeSensorDistances()));
    QObject::connect(button["switch_lighthouses"], SIGNAL(clicked()), this, SLOT(switchLighthouses()));
    QObject::connect(button["record"], SIGNAL(clicked()), this, SLOT(record()));
    QObject::connect(button["connect_roboy"], SIGNAL(clicked()), this, SLOT(connectRoboy()));
    QObject::connect(button["connect_object"], SIGNAL(clicked()), this, SLOT(connectObject()));
    QObject::connect(button["clear_all"], SIGNAL(clicked()), this, SLOT(clearAll()));

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "darkroom_rqt_plugin");
    }

    pose_correction_sub = nh->subscribe("/roboy/middleware/DarkRoom/LighthousePoseCorrection", 1,
                                        &RoboyDarkRoom::correctPose, this);
    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    resetLighthousePoses();

    publish_transform = true;
    if(transform_thread==nullptr){
        transform_thread = boost::shared_ptr<std::thread>(new std::thread(&RoboyDarkRoom::transformPublisher, this));
        transform_thread->detach();
    }
}

void RoboyDarkRoom::shutdownPlugin() {
}

void RoboyDarkRoom::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void RoboyDarkRoom::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}


void RoboyDarkRoom::connectRoboy(){
    ROS_INFO("connect roboy clicked");
    TrackedObjectPtr newObject = TrackedObjectPtr(new TrackedObject());
    newObject->connectRoboy();
    trackedObjects[object_counter] = newObject;
    object_counter++;
}

void RoboyDarkRoom::connectObject(){
    ROS_INFO("connect object clicked");
    TrackedObjectPtr newObject = TrackedObjectPtr(new TrackedObject());
    bool ok;
    newObject->connectObject(text["broadcast_ip"]->text().toStdString().c_str(), text["broadcast_port"]->text().toInt(&ok));
    trackedObjects[object_counter] = newObject;
    object_counter++;
}

void RoboyDarkRoom::clearAll(){
    ROS_INFO("clear all clicked");
    for (auto const &object:trackedObjects) {
        object.second->clearAll();
    }
}

void RoboyDarkRoom::resetLighthousePoses() {
    ROS_INFO("reset lighthouse poses clicked");
    text["lighthouse2_x"]->setText(QString::number(slider["lighthouse2_x"]->value()/100.0*5.0));
    text["lighthouse2_y"]->setText(QString::number(slider["lighthouse2_y"]->value()/100.0*5.0));
    text["lighthouse2_z"]->setText(QString::number(slider["lighthouse2_z"]->value()/100.0*5.0));
    text["lighthouse2_roll"]->setText(QString::number(slider["lighthouse2_roll"]->value()));
    text["lighthouse2_pitch"]->setText(QString::number(slider["lighthouse2_pitch"]->value()));
    text["lighthouse2_yaw"]->setText(QString::number(slider["lighthouse2_yaw"]->value()));

    tf_world.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion quat;
    quat.setRPY(0, 0, 0);
    tf_world.setRotation(quat);
    quat.setRPY(0, 0, 0);
    lighthouse1.setOrigin(tf::Vector3(0, 0, 0));
    quat.setRPY(0,0,0);
    lighthouse1.setRotation(quat);
    bool ok;
    quat.setRPY(
            text["lighthouse2_roll"]->text().toFloat(&ok)*M_PI/180.0,
            text["lighthouse2_pitch"]->text().toFloat(&ok)*M_PI/180.0,
            text["lighthouse2_yaw"]->text().toFloat(&ok)*M_PI/180.0
    );
    lighthouse2.setRotation(quat);
    lighthouse2.setOrigin(tf::Vector3(
            text["lighthouse2_x"]->text().toFloat(&ok),
            text["lighthouse2_y"]->text().toFloat(&ok),
            text["lighthouse2_z"]->text().toFloat(&ok)
    ));
    lighthouse2.setRotation(quat);
}

void RoboyDarkRoom::record() {
    ROS_INFO("record clicked");
    if (trackedObjects.empty())
        button["record"]->setChecked(false);
    for (auto const &object:trackedObjects) {
        lock_guard<mutex> (object.second->mux);
        object.second->record(button["record"]->isChecked());
    }
}

void RoboyDarkRoom::showRays() {
    ROS_INFO("show rays clicked");
    for (auto const &object:trackedObjects) {
        lock_guard<mutex> (object.second->mux);
        object.second->rays = button["show_rays"]->isChecked();
    }
}

void RoboyDarkRoom::showDistances() {
    ROS_INFO("show distances clicked");
    for (auto const &object:trackedObjects) {
        lock_guard<mutex> (object.second->mux);
        object.second->distances = object.second->rays = button["show_distances"]->isChecked();
    }
}

void RoboyDarkRoom::switchLighthouses() {
    ROS_INFO("switch lighthouses clicked");
    for (auto const &object:trackedObjects) {
        lock_guard<mutex> (object.second->mux);
        object.second->switchLighthouses(button["switch_lighthouses"]->isChecked());
    }
}

void RoboyDarkRoom::startCalibrateRelativeSensorDistances() {
    ROS_INFO("calibrate_relative_distances clicked");
    for (uint i=0; i<trackedObjects.size(); i++) {
        lock_guard<mutex> (trackedObjects[i]->mux);
        if (button["calibrate_relative_distances"]->isChecked()) {
            ROS_INFO("starting calibration thread");
            trackedObjects[i]->calibrating = true;
            trackedObjects[i]->calibrate_thread = boost::shared_ptr<boost::thread>(
                    new boost::thread(
                            [this, i](){this->trackedObjects[i]->calibrateRelativeSensorDistances();}
                    ));
            trackedObjects[i]->calibrate_thread->detach();
        } else {
            if (trackedObjects[i]->calibrate_thread != nullptr) {
                trackedObjects[i]->calibrating = false;
                if (trackedObjects[i]->calibrate_thread->joinable()) {
                    ROS_INFO("Waiting for calibration thread to terminate");
                    trackedObjects[i]->calibrate_thread->join();
                }
            }
        }
    }
}

void RoboyDarkRoom::startTriangulation() {
    ROS_INFO("triangulate clicked");
    for (uint i=0; i<trackedObjects.size(); i++) {
        lock_guard<mutex> (trackedObjects[i]->mux);
        if (button["triangulate"]->isChecked()) {
            ROS_INFO("starting tracking thread");
            trackedObjects[i]->tracking = true;
            trackedObjects[i]->tracking_thread = boost::shared_ptr<boost::thread>(
                    new boost::thread(
                            [this, i](){this->trackedObjects[i]->triangulateSensors();}
                    ));
            trackedObjects[i]->tracking_thread->detach();
        } else {
            if (trackedObjects[i]->tracking_thread != nullptr) {
                ROS_INFO("stopping tracking thread");
                trackedObjects[i]->tracking = false;
                if (trackedObjects[i]->tracking_thread->joinable()) {
                    ROS_INFO("Waiting for tracking thread to terminate");
                    trackedObjects[i]->tracking_thread->join();
                }
            }
        }
    }
}

void RoboyDarkRoom::startPoseEstimationSensorCloud() {
    ROS_INFO("pose_correction_sensor_cloud clicked");
    for (uint i=0; i<trackedObjects.size(); i++) {
        lock_guard<mutex> (trackedObjects[i]->mux);
        ROS_INFO("starting pose estimation thread");
        trackedObjects[i]->poseestimating = true;
        trackedObjects[i]->poseestimation_thread = boost::shared_ptr<boost::thread>(
                new boost::thread([this, i](){
                    this->trackedObjects[i]->poseEstimationSensorCloud();
                }));
        trackedObjects[i]->poseestimation_thread->detach();
    }
}

void RoboyDarkRoom::startEstimateSensorPositionsUsingRelativeDistances() {
    ROS_INFO("position_estimation_relativ_sensor_distances clicked");
    for (uint i=0; i<trackedObjects.size(); i++) {
        lock_guard<mutex> (trackedObjects[i]->mux);
        ROS_INFO("starting relativ distance thread for lighthouse 1");
        trackedObjects[i]->distance_thread_1 = boost::shared_ptr<boost::thread>(
                new boost::thread([this, i](){
                    this->trackedObjects[i]->estimateSensorPositionsUsingRelativeDistances(LIGHTHOUSE_A, trackedObjects[i]->calibrated_sensors);
                }));
        trackedObjects[i]->distance_thread_1->detach();
        ROS_INFO("starting relativ distance thread for lighthouse 2");
        trackedObjects[i]->distance_thread_2 = boost::shared_ptr<boost::thread>(
                new boost::thread([this, i](){
                    this->trackedObjects[i]->estimateSensorPositionsUsingRelativeDistances(LIGHTHOUSE_B, trackedObjects[i]->calibrated_sensors);
                }));
        trackedObjects[i]->distance_thread_2->detach();
    }
}

void RoboyDarkRoom::startPoseEstimationParticleFilter() {
    ROS_INFO("pose_correction_particle_filter clicked");
    for (uint i=0; i<trackedObjects.size(); i++) {
        lock_guard<mutex> (trackedObjects[i]->mux);
        ROS_INFO("starting particle filter thread");
        trackedObjects[i]->particle_filtering = true;
        trackedObjects[i]->particlefilter_thread = boost::shared_ptr<boost::thread>(
                new boost::thread(
                        [this, i](){this->trackedObjects[i]->poseEstimationParticleFilter();}
                ));
        trackedObjects[i]->particlefilter_thread->detach();
    }
}

void RoboyDarkRoom::transformPublisher() {
    ros::Rate rate(10);
    while (publish_transform) {
        lock_guard<mutex> lock(mux);
        tf_broadcaster.sendTransform(tf::StampedTransform(lighthouse1, ros::Time::now(), "world", "lighthouse1"));
        tf_broadcaster.sendTransform(tf::StampedTransform(lighthouse2, ros::Time::now(), "world", "lighthouse2"));
//        tf::Matrix3x3 rot(lighthouse2.getRotation());
//
//        ROS_INFO_STREAM("rot: \n" << rot.getRow(0).getX() << "\t" << rot.getRow(0).getY() << "\t" << rot.getRow(0).getZ() << endl <<
//                                rot.getRow(1).getX() << "\t" << rot.getRow(1).getY() << "\t" << rot.getRow(1).getZ() << endl <<
//                                rot.getRow(2).getX() << "\t" << rot.getRow(2).getY() << "\t" << rot.getRow(2).getZ() << endl);
        rate.sleep();
    }
}

void RoboyDarkRoom::correctPose(const roboy_communication_middleware::LighthousePoseCorrection &msg) {
    tf::Transform tf;
    tf::transformMsgToTF(msg.tf, tf);
    if (msg.id == LIGHTHOUSE_A) {
        if (msg.type == 0) // relativ
            lighthouse1 = tf * lighthouse1;
        else    // absolut
            lighthouse1 = tf;
    } else {
        if (msg.type == 0) // relativ
            lighthouse2 = tf * lighthouse2;
        else    // absolut
            lighthouse2 = tf;
    }
}

PLUGINLIB_DECLARE_CLASS(roboy_darkroom, RoboyDarkRoom, RoboyDarkRoom, rqt_gui_cpp::Plugin)
