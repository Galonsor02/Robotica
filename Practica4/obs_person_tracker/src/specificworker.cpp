/*
 *    Copyright (C) 2024 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <cppitertools/enumerate.hpp>
#include <cppitertools/range.hpp>
#include <cppitertools/sliding_window.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    std::locale::global(std::locale("C"));
    this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
}
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    return true;
}
void SpecificWorker::initialize()
{
   // std::cout << "Initialize worker" << std::endl;
    if(this->startup_check_flag)
    {
        std::cout << "check flag" << std::endl;

        this->startup_check();
    }
    else
    {
        // Viewer
        std::cout << "viewer" << std::endl;

        viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
        auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
        robot_draw = r;
        viewer->setStyleSheet("background-color: lightGray;");
        this->resize(800, 700);

        // Initialize the plot
        plot = new QCustomPlot(frame_dist);
        plot->resize(frame_dist->size());
        plot->addGraph();
        plot->graph(0)->setPen(QPen(QColor(0, 0, 255)));
        plot->xAxis->setLabel("Time");
        plot->yAxis->setLabel("Dist. to person");
        plot->xAxis->setRange(0, 50);
        plot->yAxis->setRange(-1000, 1000);
        plot->replot();
        plot->show();

        // connect stop button in UI with a lambda function
        connect(pushButton_stop, &QPushButton::clicked, [this]()
        {
            try
            { omnirobot_proxy->setSpeedBase(0, 0, 0); }
            catch (const Ice::Exception &e)
            { std::cout << e << std::endl; }
            pushButton_stop->setText(pushButton_stop->isChecked() ? "Track" : "Stop");
        });
        viewer->show();
        std::cout << "late viewer" << std::endl;

        this->setPeriod(STATES::Compute, 100);
    }
}
void SpecificWorker::compute()
{
    /// Check if there is new YOLO data in buffer
    std::cout << "compute worker" << std::endl;

    std::expected<RoboCompVisualElementsPub::TObject, std::string> tp_person = std::unexpected("No person found");
    auto [data_] = buffer.read_first();
    if(data_.has_value())
        tp_person = find_person_in_data(data_.value().objects);
    else return;

    std::vector<Eigen::Vector2f> path;
    if(tp_person.has_value())
    {
        std::cout << "has value" << std::endl;

        float x = std::stof(tp_person.value().attributes.at("x_pos"));
        float y = std::stof(tp_person.value().attributes.at("y_pos"));

        try
        {
            auto res = grid2d_proxy->getPaths(RoboCompGrid2D::TPoint{0.f, 0.f, 250}, RoboCompGrid2D::TPoint{x, y, 250});
            path.clear(); // Ensure the vector is empty before filling it
            for (const auto &p : res.path)
                path.emplace_back(p.x, p.y); // Convert RoboCompGrid2D::TPoint to Eigen::Vector2f
            draw_path(res.path);
        }
        catch (const Ice::Exception &e) { qDebug() << "Error calling Grid2D_getPaths:" << e.what(); }
    }

    if (path.empty())
    {
        qWarning() << "Empty path. Changing to SEARCH state.";
        state = STATE::SEARCH;
        const auto &[adv,rot]=state_machine(path); // Call the state machine with an empty path
        try{ omnirobot_proxy->setSpeedBase(0.f, adv, rot); }
        catch(const Ice::Exception &e){std::cout << e << std::endl;}
        return;
    }
    else
    {
        const auto &[adv, rot] = state_machine(path);
        // Plot on UI
        if(tp_person)
        {
            float d = std::hypot(std::stof(tp_person.value().attributes.at("x_pos")),
                                 std::stof(tp_person.value().attributes.at("y_pos")));
            plot_distance(running_average(d) - params.PERSON_MIN_DIST);
            lcdNumber_dist_to_person->display(d);
            lcdNumber_angle_to_person->display(atan2(std::stof(tp_person.value().attributes.at("x_pos")),
                                                     std::stof(tp_person.value().attributes.at("y_pos"))));
        }
        lcdNumber_adv->display(adv);
        lcdNumber_rot->display(rot);

        // Move the robot
        try{ omnirobot_proxy->setSpeedBase(0.f, adv, rot); }
        catch(const Ice::Exception &e){std::cout << e << std::endl;}
    }
}

std::expected<RoboCompVisualElementsPub::TObject, std::string> SpecificWorker::find_person_in_data(const std::vector<RoboCompVisualElementsPub::TObject> &objects)
{
    if(objects.empty())
        return std::unexpected("Empty objects in method <find_person_in_data>");
    if(auto p_ = std::ranges::find_if(objects, [](auto &a)
            { return a.id == 0 and std::stof(a.attributes.at("score")) > 0.6;}); p_ == std::end(objects))
        return std::unexpected("No person found in method <find_person_in_data>");
    else
        return *p_;
}

void SpecificWorker::draw_path(const std::vector<RoboCompGrid2D::TPoint> &path)
{
    // Limpiar elementos gráficos de la ruta anterior
    for (auto item : path_items)
    {
        viewer->scene.removeItem(item);
        delete item;
    }
    path_items.clear();

    if (path.empty())
    {
        qDebug() << "La ruta está vacía. No hay nada que dibujar.";
        return;
    }

    auto color = QColor(128, 0, 128); // Color púrpura
    auto brush = QBrush(color);

    for (const auto &point : path)
    {
        // Dibujar un rectángulo para cada punto en la ruta
        auto item = viewer->scene.addRect(-50, -50, 100, 100, QPen(color), brush);
        item->setPos(point.x, point.y); // Usar directamente las coordenadas de TPoint
        path_items.push_back(item);
    }
}
//////////////////////////////////////////////////////////////////
/// STATE MACHINE
//////////////////////////////////////////////////////////////////
SpecificWorker::RobotSpeed SpecificWorker::state_machine(const TPath &path)
{
    RetVal res;
    if(pushButton_stop->isChecked())    // Stop if button is pressed
        state = STATE::STOP;

    switch(state)
    {
        case STATE::TRACK:
            res = track(path);
            label_state->setText("TRACK");
            break;
        case STATE::WAIT:
            res = wait(path);
            label_state->setText("WAIT");
            break;
        case STATE::SEARCH:
            res = search(path);
            label_state->setText("SEARCH");
            break;
        case STATE::STOP:
            res = stop();
            label_state->setText("STOP");
            break;
    }
    auto &[st, speed, rot] = res;
    state = st;
    return {speed, rot};
}

SpecificWorker::RetVal SpecificWorker::track(const TPath &path)
{
    cout << "Tracking..." << endl;
    static float ant_angle_error = 0.0;
    //qDebug() << __FUNCTION__;
    // variance of the gaussian function is set by the user giving a point xset where the function must be yset, and solving for s
    auto gaussian_break = [](float x) -> float
    {
        // gaussian function where x is the rotation speed -1 to 1. Returns 1 for x = 0 and 0.4 for x = 0.5
        const double xset = 0.5;
        const double yset = 0.73;
        //compute the variance s so the function is yset for x = xset
        float s = -xset*xset/(log(yset));
        return (float)exp(-x*x/s);
    };

    if(path.empty())
    { /* qWarning() << __FUNCTION__ << "No person found"; */ return RetVal(STATE::SEARCH, 0.f, 0.f);
    }

    auto distance = path.back().norm()+100.f;
    lcdNumber_dist_to_person->display(distance);

    // check if the distance to the person is lower than a threshold
    if(distance < params.PERSON_MIN_DIST) {

        qWarning() << __FUNCTION__ << "Distance to person lower than threshold"; return RetVal(STATE::WAIT, 0.f, 0.f);
    }

    // angle error is the angle between the robot and the person. It has to be brought to zero
    float angle_error = atan2(path[1].x(), path[1].y());
    float rot_speed = params.k1 * angle_error + params.k2 * (angle_error-ant_angle_error);
    ant_angle_error = angle_error;
    // rot_brake is a value between 0 and 1 that decreases the speed when the robot is not facing the person
    float rot_brake = gaussian_break(rot_speed);
    // acc_distance is the distance given to the robot to reach again the maximum speed
    float acc_distance = params.acc_distance_factor * params.ROBOT_WIDTH;
    // advance brake is a value between 0 and 1 that decreases the speed when the robot is too close to the person
    float adv_brake = std::clamp(distance * 1.f/acc_distance - (params.PERSON_MIN_DIST / acc_distance), 0.f, 1.f);
    return RetVal(STATE::TRACK, params.MAX_ADV_SPEED * rot_brake * adv_brake, rot_speed);
}

SpecificWorker::RetVal SpecificWorker::wait(const TPath &path)
{
    cout << "Waiting..." << endl;
    if(path.empty())
        return RetVal(STATE::SEARCH, 0.f, 0.f);

    if(std::hypot(path[0].x(), path[0].y()) > params.PERSON_MIN_DIST + 100)
        return RetVal(STATE::TRACK, 0.f, 0.f);

    return RetVal(STATE::WAIT, 0.f, 0.f);
}

SpecificWorker::RetVal SpecificWorker::search(const TPath &path)
{
    cout << "Searching..." << endl;
    if(not path.empty())
        return RetVal(STATE::TRACK, 0.f, 0.f);

    return RetVal(STATE::SEARCH, 0.f, params.SEARCH_ROT_SPEED);
}

SpecificWorker::RetVal SpecificWorker::stop()
{
    cout << "Stopping..." << endl;
    if(not pushButton_stop->isChecked())
        return RetVal(STATE::TRACK, 0.f, 0.f);

    return RetVal(STATE::STOP, 0.f, 0.f);
}



void SpecificWorker::plot_distance(double distance)
{
    // add value to plot
    static int key = 0;
    plot->graph(0)->addData(key++, distance);
    // Remove data points if there are more than X
    if (plot->graph(0)->dataCount() > params.MAX_DIST_POINTS_TO_SHOW)
        plot->graph(0)->data()->removeBefore(key - params.MAX_DIST_POINTS_TO_SHOW);
    // plot
    plot->rescaleAxes();  plot->replot();
}
float SpecificWorker::running_average(float dist)
{
    static float avg = 0;
    static int count = 0;
    avg = (avg * count + dist) / (count + 1);
    count++;
    return avg;
}

//////////////////////////////////////////////////////////////////
/// SUBSCRIPTIONS (runs in a different thread)
//////////////////////////////////////////////////////////////////
//SUBSCRIPTION to setVisualObjects method from VisualElementsPub interface. This is called in a different thread.
void SpecificWorker::VisualElementsPub_setVisualObjects(RoboCompVisualElementsPub::TData data)
{
    // std::cout << "VisualElements_setVisualObjects" << std::endl;
    //    for(auto object : data.objects)
    //        std::cout << "Object type: " << object.id << std::endl;
    //    qDebug() << "Size: " << data.objects.size();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    buffer.put<0>(std::move(data), timestamp); // inserts the laser data value to the queue 0.
}
//////////////////////////////////////////////////////////////////
/// AUXILIARY FUNCTIONS
//////////////////////////////////////////////////////////////////
void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
	//computeCODE
	//
	//if (SUCCESSFUL)
    //  emmit goToRestore()
}
//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
	//computeCODE
	//Restore emergency component

}
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}



/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)
// this->lidar3d_proxy->getLidarDataArrayProyectedInImage(...)
// this->lidar3d_proxy->getLidarDataProyectedInImage(...)
// this->lidar3d_proxy->getLidarDataWithThreshold2d(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// this->omnirobot_proxy->correctOdometer(...)
// this->omnirobot_proxy->getBasePose(...)
// this->omnirobot_proxy->getBaseState(...)
// this->omnirobot_proxy->resetOdometer(...)
// this->omnirobot_proxy->setOdometer(...)
// this->omnirobot_proxy->setOdometerPose(...)
// this->omnirobot_proxy->setSpeedBase(...)
// this->omnirobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

/**************************************/
// From the RoboCompVisualElements you can call this methods:
// this->visualelements_proxy->getVisualObjects(...)
// this->visualelements_proxy->setVisualObjects(...)

/**************************************/
// From the RoboCompVisualElements you can use this types:
// RoboCompVisualElements::TRoi
// RoboCompVisualElements::TObject
// RoboCompVisualElements::TObjects

// Instantiate the random number generator and distribution
//    static std::mt19937 gen(rd());
//    static std::uniform_int_distribution<int> dist(0, 1);
//    static bool first_time = true;
//    static int sign = 1;