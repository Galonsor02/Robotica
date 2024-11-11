//
// Created by pbustos on 21/10/24.
//

#include "dbscan.h"
#include <mlpack/methods/dbscan/dbscan.hpp>
#include <mlpack/core.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/range.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <QVector2D>

namespace rc
{
    QPolygonF enlarge_polygon(const QPolygonF &polygon, qreal amount)
    {
        QPolygonF enlargedPolygon;
        QPolygonF polygonCopy = polygon;
        polygonCopy << QPointF(polygon[0]);
        polygonCopy << QPointF(polygon[1]);
        for(const auto vs : iter::sliding_window(polygonCopy, 3))
        {

        }


        return enlargedPolygon;
    }
    std::vector<QPolygonF> dbscan(const std::vector<Eigen::Vector2f> &points,
                                  float eps, int min_points,
                                  float robot_width)
    {
        if(points.empty()) {std::cout << __FUNCTION__ << " No points" << std::endl; return {};}
        arma::mat arma_data(2, points.size());
        for (const auto &[i, p]: points | iter::enumerate)
        {
            arma_data(0, i) = p.x();
            arma_data(1, i) = p.y();
        }
        arma::Row<size_t> assignments;
        mlpack::dbscan::DBSCAN<> dbscan(eps, min_points);
        dbscan.Cluster(arma_data, assignments);
        std::map<size_t, std::vector<cv::Point2f>> clustersMap;
        for (const auto i: iter::range(assignments.n_elem))
        {
            size_t label = assignments[i];
            if (label != (size_t) -1)
                clustersMap[label].emplace_back(points[i].x(), points[i].y());  // -1 indicates noise
        }

        // compute polygons
        std::vector<QPolygonF> list_poly;
        std::vector<cv::Point2f> hull;
        for (const auto &pair: clustersMap)
        {
            // Calculate the convex hull of the cluster
            std::vector<cv::Point2f> hull;
            cv::convexHull(pair.second, hull);

            // Convert the convex hull to a QPolygonF
            QPolygonF poly;
            for (const auto &p: hull)
                poly << QPointF(p.x, p.y);

            // enlarge the polygon to account for the robot size

            list_poly.emplace_back(enlarge_polygon(poly,robot_width));
        }
        return list_poly;
    };
}