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
    std::cout << "Initialize worker" << std::endl;
    if(this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
        // Viewer
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

        this->setPeriod(STATES::Compute, 100);
    }
}
void SpecificWorker::compute()
{
    /// Check if there is new YOLO data in buffer
    std::expected<RoboCompVisualElementsPub::TObject, std::string> tp_person = std::unexpected("No person found");
    auto [data_] = buffer.read_first();
    if(data_.has_value())
        tp_person = find_person_in_data(data_.value().objects);
    else return;

    std::vector<Eigen::Vector2f> path;
    if(tp_person.has_value())
    {
        float x = std::stof(tp_person.value().attributes.at("x_pos"));
        float y = std::stof(tp_person.value().attributes.at("y_pos"));

        try
        {
            auto res = grid2d_proxy->getPaths(RoboCompGrid2D::TPoint{0.f, 0.f, 250}, RoboCompGrid2D::TPoint{x, y, 250});
            path.clear(); // Ensure the vector is empty before filling it
            for (const auto &p : res.path)
                path.emplace_back(p.x, p.y); // Convert RoboCompGrid2D::TPoint to Eigen::Vector2f
        }
        catch (const Ice::Exception &e) { qDebug() << "Error calling Grid2D_getPaths:" << e.what(); }
    }

    if (path.empty())
    {
        qWarning() << "Empty path. Changing to SEARCH state.";
        state_machine(path); // Call the state machine with an empty path
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

//////////////////////////////////////////////////////////////////
/// STATE MACHINE
//////////////////////////////////////////////////////////////////
SpecificWorker::RobotSpeed SpecificWorker::state_machine(const Tpath &path)
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

SpecificWorker::RetVal SpecificWorker::track(const Tpath &path)
{
    static float ant_angle_error = 0.0;
    auto gaussian_break = [](float x) -> float
    {
        const double xset = 0.5;
        const double yset = 0.73;
        float s = -xset*xset/(log(yset));
        return (float)exp(-x*x/s);
    };

    if(path.empty())
        return RetVal(STATE::SEARCH, 0.f, 0.f);

    auto distance = path.back().norm();
    lcdNumber_dist_to_person->display(distance);

    if(distance < params.PERSON_MIN_DIST)
        return RetVal(STATE::WAIT, 0.f, 0.f);

    float angle_error = atan2(path[3].x(), path[3].y());
    float rot_speed = params.k1 * angle_error + params.k2 * (angle_error-ant_angle_error);
    ant_angle_error = angle_error;
    float rot_brake = gaussian_break(rot_speed);
    float acc_distance = params.acc_distance_factor * params.ROBOT_WIDTH;
    float adv_brake = std::clamp(distance * 1.f/acc_distance - (params.PERSON_MIN_DIST / acc_distance), 0.f, 1.f);
    return RetVal(STATE::TRACK, params.MAX_ADV_SPEED * rot_brake * adv_brake, rot_speed);
}

SpecificWorker::RetVal SpecificWorker::wait(const Tpath &path)
{
    if(path.empty())
        return RetVal(STATE::TRACK, 0.f, 0.f);

    if(std::hypot(path[0].x(), path[0].y()) > params.PERSON_MIN_DIST + 100)
        return RetVal(STATE::TRACK, 0.f, 0.f);

    return RetVal(STATE::WAIT, 0.f, 0.f);
}

SpecificWorker::RetVal SpecificWorker::search(const Tpath &path)
{
    if(not path.empty())
        return RetVal(STATE::TRACK, 0.f, 0.f);

    return RetVal(STATE::SEARCH, 0.f, params.SEARCH_ROT_SPEED);
}

SpecificWorker::RetVal SpecificWorker::stop()
{
    if(not pushButton_stop->isChecked())
        return RetVal(STATE::TRACK, 0.f, 0.f);

    return RetVal(STATE::STOP, 0.f, 0.f);
}