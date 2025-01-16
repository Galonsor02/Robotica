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

/**
	\brief
	@author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#define HIBERNATION_ENABLED

#include <genericworker.h>
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"
#include <expected>
#include <random>
#include <doublebuffer_sync/doublebuffer_sync.h>
#include <Eigen/Eigen>
#include <locale>
#include <qcustomplot/qcustomplot.h>

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);
        void VisualElementsPub_setVisualObjects(RoboCompVisualElementsPub::TData data);

    public slots:
        void initialize();
        void compute();
        void emergency();
        void restore();
        int startup_check();
        void draw_path(const std::vector<RoboCompGrid2D::TPoint> &path);

    private:
        bool startup_check_flag;
        struct Params
        {
            float ROBOT_WIDTH = 460;  // mm
            float ROBOT_LENGTH = 480;  // mm
            float MAX_ADV_SPEED = 700; // mm/s
            float MAX_ROT_SPEED = 2; // rad/s
            float SEARCH_ROT_SPEED = 1; // rad/s
            float PERSON_MIN_DIST = 1000; // mm
            int MAX_DIST_POINTS_TO_SHOW = 300; // points to show in plot
            QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};
            float acc_distance_factor = 2;
            float k1 = 1.1;  // proportional gain for the angle error;
            float k2 = 0.5; // proportional gain for derivative of the angle error;
        };
        Params params;

        // state machine
        enum class STATE
        {
            TRACK, STOP, WAIT, SEARCH
        };
        STATE state = STATE::TRACK;
        using RetVal = std::tuple<STATE, float, float>;
        using RobotSpeed = std::tuple<float, float>;
        using TPath = std::vector<Eigen::Vector2f>;
        using TPerson = std::expected<RoboCompVisualElementsPub::TObject, std::string>;
        RetVal track(const TPath &path);
        RetVal wait(const TPath &path);
        RetVal search(const TPath &path);
        RetVal stop();
        RobotSpeed state_machine( const TPath &path);
        std::vector<QGraphicsItem*> path_items; // Ítems gráficos para la ruta
        bool flag=true;
        RoboCompGrid2D::Result onePath;
        // draw
        AbstractGraphicViewer *viewer;
        QGraphicsPolygonItem *robot_draw;
        void draw_person(RoboCompVisualElementsPub::TObject &person, QGraphicsScene *scene) const;

        // person
        std::expected<RoboCompVisualElementsPub::TObject, std::string> find_person_in_data(const std::vector<RoboCompVisualElementsPub::TObject> &objects);

        // DoubleBufferSync to synchronize the subscription thread with compute
        BufferSync<InOut<RoboCompVisualElementsPub::TData, RoboCompVisualElementsPub::TData>> buffer;

        // QCustomPlot object
        QCustomPlot *plot;
        void plot_distance(double distance);

        float running_average(float dist);
};
#endif
