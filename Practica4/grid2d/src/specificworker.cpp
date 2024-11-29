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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
	// Uncomment if there's too many debug messages
	// but it removes the possibility to see the messages
	// shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
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
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }
	

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
        viewer->show();

		// grid
		QPen pen(QColor("blue"), 20);
		for (const auto &[i, row] : grid | iter::enumerate)
		{
			for (const auto &[j, cell] : row | iter::enumerate)
			{
				cell.State = STATE::UNKNOWN;
				cell.item = viewer->scene.addRect(-cellSize/2, -cellSize/2, cellSize, cellSize, pen);
				cell.item->setPos(real_to_index(i, j));

			}
		}

		this->setPeriod(STATES::Compute, 100);
		//this->setPeriod(STATES::Emergency, 500);

	}

}

void SpecificWorker::compute()
{
    std::cout << "Compute worker" << std::endl;

    //read bpearl (lower) lidar and draw
    auto ldata_bpearl = read_lidar_bpearl();
    if(ldata_bpearl.empty()) { qWarning() << __FUNCTION__ << "Empty bpearl lidar data"; return; };
    draw_lidar(ldata_bpearl, &viewer->scene);

	// update grid
	update_grid(ldata_bpearl);
}

//READ LIDAR BPEARL AND HELIOS
std::vector<Eigen::Vector2f> SpecificWorker::read_lidar_bpearl()
{
    try
    {
        auto ldata =  lidar3d_proxy->getLidarData("bpearl", 0, 2*M_PI, 1);
        // filter points according to height and distance
        std::vector<Eigen::Vector2f>  p_filter;

        for(const auto &a: ldata.points)
        {
            if(a.z < 500 and a.distance2d > 200)
                p_filter.emplace_back(a.x, a.y);
        }
        return p_filter;
    }
    catch(const Ice::Exception &e){std::cout << e << std::endl;}
    return {};
}

void SpecificWorker::update_grid(std::vector<Eigen::Vector2f> bpearl)
{
	QPen pen(QColor("red"), 20);
	for (const auto &a: bpearl)
	{
		grid[a.x()][a.y()].item=viewer->scene.addRect(-cellSize/2, -cellSize/2, cellSize, cellSize, pen);
		grid[a.x()][a.y()].item->setPos(real_to_index(a.x(),a.y()));
	}
}

QPointF SpecificWorker::real_to_index(float x, float y)
{
	auto i= ((dimension/gridScale)*x) - dimension/2;
	auto j= ((dimension/gridScale)*y) - dimension/2;

	return QPointF(i, j);
}

QPointF SpecificWorker::index_to_real(float i, float j)
{
	auto x= ((i+((dimension*gridScale)/2))/dimension);
	auto y= ((j+((dimension*gridScale)/2))/dimension);
	QPointF result;
	result.setX(x);
	result.setY(y);
	return result;
}

/**
 * Draws LIDAR points onto a QGraphicsScene.
 *
 * This method clears any existing graphical items from the scene, then iterates over the filtered
 * LIDAR points to add new items. Each LIDAR point is represented as a colored rectangle. The point
 * with the minimum distance is highlighted in red, while the other points are drawn in green.
 *
 * @param filtered_points A collection of filtered points to be drawn, each containing the coordinates
 *                        and distance.
 * @param scene A pointer to the QGraphicsScene where the points will be drawn.
 */
void SpecificWorker::draw_lidar(auto &filtered_points, QGraphicsScene *scene)
{
    static std::vector<QGraphicsItem*> items;   // store items so they can be shown between iterations

    // remove all items drawn in the previous iteration
    for(auto i: items)
    {
        scene->removeItem(i);
        delete i;
    }
    items.clear();

    auto color = QColor(Qt::darkGreen);
    auto brush = QBrush(QColor(Qt::darkGreen));
    for(const auto &p : filtered_points)
    {
        auto item = scene->addRect(-50, -50, 100, 100, color, brush);
        item->setPos(p.x(), p.y());
        items.push_back(item);
    }
}




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


RoboCompGrid2D::Result SpecificWorker::Grid2D_getPaths(RoboCompGrid2D::TPoint source, RoboCompGrid2D::TPoint target)
{
#ifdef HIBERNATION_ENABLED
	hibernation = true;
#endif
//implementCODE

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
// From the RoboCompGrid2D you can use this types:
// RoboCompGrid2D::TPoint
// RoboCompGrid2D::Result

