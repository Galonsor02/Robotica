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

#include <queue>
#include <ranges>
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
		std::cout << "startup check" << std::endl;
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
        viewer->show();

		// grid
		QPen pen(QColor("blue"),15);
		for (const auto &[i, row] : grid | iter::enumerate)
		{
			for (const auto &[j, cell] : row | iter::enumerate)
			{
				cell.State = STATE::UNKNOWN;
				cell.item = viewer->scene.addRect(-cellSize/2.f, -cellSize/2.f, cellSize, cellSize, pen);
				cell.item->setPos(index_to_real(i, j));
			}
		}

		connect(viewer, SIGNAL(new_mouse_coordinates(QPointF)), this, SLOT(viewerSlot(QPointF)));

		this->setPeriod(STATES::Compute, 100);
		//this->setPeriod(STATES::Emergency, 500);
	}
}

void SpecificWorker::compute()
{
    reset_grid();

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

void SpecificWorker::update_grid(std::vector<Eigen::Vector2f> lidar_points)
{
    // Itera sobre los puntos LiDAR

    for (const auto& point : lidar_points)
    {
        // Calcula la distancia al punto LiDAR (módulo del vector) (hipotenusa)
        float distance = std::sqrt(point.x() * point.x() + point.y() * point.y());

        // Calcula el número de pasos (S) y el delta
        float Steps = (distance / cellSize);  // S es el número de pasos de 100mm(TAMAÑO DE CELDA)

        // Asegúrate de que Steps no sea cero (evitar división por cero)
        if (Steps == 0) {
            std::cout << "Steps is zero, skipping point." << std::endl;
            continue; // Si Steps es 0, no procesamos este punto
        }

        // Itera sobre los pasos desde 0 a 1
        for (float step = 0.f; step < 1.f; step += 1.f / Steps) // Ajusta el incremento para que no se pase de 1
        {
            // Convierte las coordenadas a índice de la celda
            auto q = point * step;

            QPoint cell_index = real_to_index(q.x(), q.y());

            // Verifica si los índices están dentro de los límites del grid
            if (cell_index.x() < 0 || cell_index.x() >= gridSize || cell_index.y() < 0 || cell_index.y() >= gridSize) {
                std::cout << "Cell index out of bounds: " << cell_index.x() << ", " << cell_index.y() << std::endl;
                continue; // Si el índice está fuera de los límites, saltamos este paso
            }

            std::cout << q.x() << " " << q.y() << " " << cell_index.x() << " " << cell_index.y() << std::endl;

            // Encuentra la celda correspondiente en el grid (redondear a la posición más cercana)
            auto& cell = grid[cell_index.x()][cell_index.y()];

            // Cambia el estado de la celda a blanco (Estado conocido)
            cell.State = STATE::FREE;
            cell.item->setBrush(QBrush(QColor("white")));
        }

        // Marca la última celda en rojo (lo más cercano al punto LiDAR)
        QPoint last_cell_index = real_to_index(point.x(), point.y());
        std::cout << point.x() << " " << point.y() << " " << last_cell_index.x() << " " << last_cell_index.y() << std::endl;

        // Verificación de límites antes de acceder a las celdas cercanas
        for (int i = last_cell_index.x() - 1; i <= last_cell_index.x() + 1; ++i)
        {
            if (i < 0 || i >= gridSize) continue; // Asegurarse de no estar fuera del rango

            for (int j = last_cell_index.y() - 2; j <= last_cell_index.y() + 2; ++j)
            {
                if (j < 0 || j >= gridSize) continue; // Asegurarse de no estar fuera del rango

                // Encuentra la última celda y marca su estado a rojo (Estado detectado)
                auto& last_cell = grid[i][j];
                last_cell.State = STATE::OCCUPIED;
                last_cell.item->setBrush(QBrush(QColor("red")));
            }
        }
    }

}

void SpecificWorker::reset_grid()
{
	for (const auto &[i, row] : grid | iter::enumerate)
		for (const auto &[j, cell] : row | iter::enumerate)
		{
			cell.State = STATE::UNKNOWN;
			cell.item->setBrush(QBrush(QColor("LightGray")));
		}

}

QPointF SpecificWorker::index_to_real(int i, int j)
{
	const auto x= i*dimension/gridSize-(dimension/2);
	const auto y= -j*dimension/gridSize + (dimension/2);
	return QPointF(x, y);
}

QPoint SpecificWorker::real_to_index(float x, float y)
{
	int i= (((dimension/2)+x)*gridSize)/dimension;
	int j= -((y-(dimension/2))*gridSize)/dimension;
	QPoint result(std::clamp(i,0,dimension-1), std::clamp(j,0,dimension-1));
	return {result.x(), result.y()};
}


bool SpecificWorker::grid_index_valid(const QPoint& index) {
	return index.x() >= 0 && index.x() < gridSize && index.y() >= 0 && index.y() < gridSize;
}
// La función de Dijkstra

std::vector<QPointF> SpecificWorker::dijkstra(QPointF start_, QPointF goal_)
{
	const auto start = real_to_index(start_.x(), start_.y());
	const auto goal = real_to_index(goal_.x(), goal_.y());
    // Mapa para almacenar el costo mínimo de cada celda
    std::unordered_map<QPoint, float, QPointHash> distance_map;
    // Mapa para almacenar la celda anterior en el camino
    std::unordered_map<QPoint, QPoint, QPointHash> previous_map;

    // Cola de prioridad para procesar las celdas con menor costo primero
    std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> pq;
    pq.push({0.0f, start});  // Comenzamos con el punto de inicio con un costo de 0
    distance_map[start] = 0.0f;

    // Direcciones de los vecinos: arriba, abajo, izquierda, derecha
    std::vector<QPoint> directions = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};

    while (!pq.empty()) {
        Cell current = pq.top();
        pq.pop();

        // Si llegamos al objetivo, reconstruimos el camino
        if (current.position == goal) {
        	std::vector<QPoint> path;
        	while (previous_map.find(current.position) != previous_map.end()) {
        		path.push_back(current.position);
        		current.position = previous_map[current.position];
        	}
        	std::reverse(path.begin(), path.end());  // Invertir el camino para que vaya de inicio a objetivo
        	std::vector<QPointF> path_real;
        	std::ranges::transform(path, std::back_inserter(path_real), [this](const auto& p)
        			{ return index_to_real(p.x(), p.y()); });
        	return path_real;
        }

        // Explorar los vecinos
        for (const auto& dir : directions) {
            QPoint neighbor = current.position + dir;

            // Comprobar si el vecino está dentro de los límites del grid
            if (grid_index_valid(neighbor)) {
                // Obtener el costo de la celda vecina (ya sea libre o un obstáculo)
                float neighbor_cost = grid[neighbor.x()][neighbor.y()].State == STATE::OCCUPIED ? INF : 1.0f;

                // Calcular el costo total de llegar al vecino
                float new_cost = current.cost + neighbor_cost;

                // Si encontramos un camino más corto al vecino, actualizamos la distancia
                if (distance_map.find(neighbor) == distance_map.end() || new_cost < distance_map[neighbor]) {
                    distance_map[neighbor] = new_cost;
                    previous_map[neighbor] = current.position;
                    pq.push({new_cost, neighbor});
                }
            }
        }
    }

    return {};  // Si no encontramos un camino, devolvemos un vector vacío
}

// // Función para encontrar y mostrar el camino en el grid
// std::vector<QPointF> SpecificWorker::find_and_display_path(QPointF start, QPointF goal)
// {
// 	// Obtener el camino más corto usando Dijkstra
// 	std::vector<QPointF> path = dijkstra(start, goal);
// 	if (not path.empty())
// 		{
//
// 	}
// 	// // Si hay un camino, lo visualizamos
// 	// if (!path.empty()) {
// 	// 	for (const auto& cell : path) {
// 	// 		// Marcar cada celda en el camino (por ejemplo, ponerla de color azul)
// 	// 		grid[cell.x()][cell.y()].item->setBrush(QBrush(QColor("blue")));
// 	// 	}
// 	// } else {
// 	// 	std::cout << "No path found!" << std::endl;
// 	// }
// }

void SpecificWorker::draw_path(const std::vector<QPointF> &path, QGraphicsScene *scene)
{
	static std::vector<QGraphicsItem*> items;   // store items so they can be shown between iterations
	// remove all items drawn in the previous iteration
	for(auto i: items)
	{ scene->removeItem(i); delete i; }
	items.clear();

	const auto color = QColor(Qt::darkYellow);
	const auto brush = QBrush(QColor(Qt::darkYellow));
	for(const auto &p : path)
	{
		auto item = scene->addRect(-50, -50, 100, 100, color, brush);
		item->setPos(p.x(), p.y());
		items.push_back(item);
	}
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

void SpecificWorker::viewerSlot(QPointF p)
{
	qDebug() << "Coordenadas reales clicadas:" << p;

	const QPoint index = real_to_index(p.x(), p.y());
	int goalX = index.x();
	int goalY = index.y();

	if (goalX < 0 || goalX >= gridSize || goalY < 0 || goalY >= gridSize)
	{
		qDebug() << "El punto está fuera del grid";
		return;
	}

	qDebug() << "Índices de cuadrícula objetivo:" << goalX << goalY;

	int startX = gridSize / 2;
	int startY = gridSize / 2;
	std::cout << "before dijkstra" << std::endl;

	auto path = dijkstra(QPointF(0, 0), p);
	std::cout << "after dijkstra" << std::endl;

	draw_path(path, &viewer->scene);
}

RoboCompGrid2D::Result SpecificWorker::Grid2D_getPaths(RoboCompGrid2D::TPoint source, RoboCompGrid2D::TPoint target)
{
	qDebug() << "Coordenadas reales clicadas:" << target.x << target.y;
	const QPoint index = real_to_index(target.x, target.y);
	const int goalX = index.x();
	const int goalY = index.y();
	if (goalX < 0 || goalX >= gridSize || goalY < 0 || goalY >= gridSize)
	{
		qDebug() << "El punto está fuera del grid";
		return {};
	}

	qDebug() << "Índices de cuadrícula objetivo:" << goalX << goalY;
	int startX = gridSize / 2;
	int startY = gridSize / 2;

	auto path = dijkstra(QPointF(source.x, source.y), QPointF(target.x, target.y));
	RoboCompGrid2D::Result result;
	std::ranges::transform(path, std::back_inserter(result.path), [](const auto& p)
			{ return RoboCompGrid2D::TPoint{p.x(), p.y()}; });
	result.timestamp = QDateTime::currentMSecsSinceEpoch();
	result.valid = !path.empty();
	return result;
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

