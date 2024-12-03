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
				cell.item->setPos(index_to_real(i, j));
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
	reset_grid();
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

void SpecificWorker::update_grid(std::vector<Eigen::Vector2f> lidar_points)
{
	// Itera sobre los puntos LiDAR
	for (const auto& point : lidar_points)
	{
		// Calcula la distancia al punto LiDAR (módulo del vector) (hipotenusa)
		float distance = std::sqrt(point.x() * point.x() + point.y() * point.y());

		// Calcula el número de pasos (S) y el delta
		float Steps = (distance / cellSize);  // S es el número de pasos de 100mm(TAMAÑO DE CELDA)

		// Itera sobre los pasos desde 0 a 1
		for (const auto &step: iter::range(0.f, 1.f, Steps))
		{
			// Convierte las coordenadas a índice de la celda
			auto q = point * step;

			QPoint cell_index = real_to_index(q.x(), q.y());
			std::cout <<q.x()<<" "<<q.y()<<" " << cell_index.x() << " " << cell_index.y() << std::endl;

			// Encuentra la celda correspondiente en el grid (redondear a la posición más cercana)
			auto cell = grid[cell_index.x()][cell_index.y()];

			// Cambia el estado de la celda a blanco (Estado conocido)
			cell.State = STATE::FREE;
			cell.item->setBrush(QBrush(QColor("white")));
		}

		// Marca la última celda en rojo (lo más cercano al punto LiDAR)

		QPoint last_cell_index = real_to_index(point.x(), point.y());
		std::cout <<point.x() <<" "<<point.y()<<" " << last_cell_index.x() << " " << last_cell_index.y() << std::endl;

		// Encuentra la última celda y marca su estado a rojo (Estado detectado)
		auto last_cell = grid[last_cell_index.x()][last_cell_index.y()];
		last_cell.State = STATE::OCCUPIED;
		last_cell.item->setBrush(QBrush(QColor("red")));
	}
}

void SpecificWorker::reset_grid()
{
	for (const auto &[i, row] : grid | iter::enumerate)
	{
		for (const auto &[j, cell] : row | iter::enumerate)
		{
			cell.State = STATE::UNKNOWN;
			cell.item->setBrush(QBrush(QColor("lightGray")));
		}
	}
}


QPointF SpecificWorker::index_to_real(int i, int j)
{
	auto x= ((dimension*2/gridScale)*i)-dimension;
	auto y= -((dimension*2/gridScale)*j)+dimension;
	return QPointF(x, y);
}

QPoint SpecificWorker::real_to_index(float x, float y)
{
	auto i= ((x + dimension) / ((dimension * 2) / gridScale));
	auto j= ((y - dimension) / -((dimension * 2) / gridScale));
	QPoint result(std::clamp(i,0,-1), j);
	result.setX(i);
	result.setY(j);
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

