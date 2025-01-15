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

// #define HIBERNATION_ENABLED

#include <genericworker.h>
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"
#include "Lidar3D.h"
#include <Eigen/Dense>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	RoboCompGrid2D::Result Grid2D_getPaths(RoboCompGrid2D::TPoint source, RoboCompGrid2D::TPoint target);


public slots:
	void initialize();
	void compute();
	void emergency();
	void restore();
	int startup_check();


	void viewerSlot(QPointF);

private:
	std::vector<QPointF> path;

	struct Params
	{
		float ROBOT_WIDTH = 460;  // mm
		float ROBOT_LENGTH = 480;  // mm
		float MAX_ADV_SPEED = 1900; // mm/s
		float MAX_ROT_SPEED = 40; // rad/s
		float SEARCH_ROT_SPEED = 0.9; // rad/s
		float STOP_THRESHOLD = 700; // mm
		float ADVANCE_THRESHOLD = ROBOT_WIDTH * 3; // mm
		float LIDAR_FRONT_SECTION = 0.2; // rads, aprox 12 degrees
		// person
		float PERSON_MIN_DIST = 1000; // mm
		int MAX_DIST_POINTS_TO_SHOW = 300; // points to show in plot
		// lidar
		std::string LIDAR_NAME_LOW = "bpearl";
		std::string LIDAR_NAME_HIGH = "helios";
		QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};
		// control track
		float acc_distance_factor = 2;
		float k1 = 1.1;  // proportional gain for the angle error;
		float k2 = 0.5; // proportional gain for derivative of the angle error;
	};

	Params params;


	enum class STATE {
		UNKNOWN, FREE, OCCUPIED
	};
	struct TCell
	{
		STATE State = STATE::UNKNOWN;
		QGraphicsRectItem *item;
	};
	TCell cell;
	struct QPointHash {
		size_t operator()(const QPoint& p) const {
			return std::hash<int>()(p.x()) ^ std::hash<int>()(p.y());
		}
	};
	// Definir la estructura Cell que usaremos en Dijkstra
	struct Cell {
		float cost;  // Costo de la celda (1 para libre, INF para ocupado)
		QPoint position;  // Posición de la celda en el grid

		bool operator>(const Cell& other) const {
			return cost > other.cost;
		}
	};
	static constexpr int cellSize = 100;
	static constexpr int dimension = 10000;
	static constexpr int gridSize = dimension / cellSize;
	std::array<std::array<TCell, gridSize>, gridSize> grid;
	const float INF = std::numeric_limits<float>::infinity();

	bool startup_check_flag;

	// read lidar
	std::vector<Eigen::Vector2f> read_lidar_bpearl();
	QPointF index_to_real(int i, int j);
	QPoint real_to_index(float i, float j);
	//update grid
	void update_grid(std::vector<Eigen::Vector2f> bpearl);
	//reset grid
	void reset_grid();
	//camino
	bool grid_index_valid(const QPoint& index);

std::vector<QPointF> dijkstra(QPointF start, QPointF goal);

std::vector<QPointF> find_and_display_path(QPointF start, QPointF goal);
    //draw lidar
	AbstractGraphicViewer *viewer;
	void draw_lidar(auto &filtered_points, QGraphicsScene *scene);
	QGraphicsPolygonItem *robot_draw;
	//draw path
	void draw_path(const std::vector<QPointF> &path, QGraphicsScene *scene);

};

#endif
