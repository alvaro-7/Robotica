/*
 *    Copyright (C) 2023 by YOUR NAME HERE
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

#include <genericworker.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <ranges>

class SpecificWorker : public GenericWorker
{

Q_OBJECT
public:
	struct Velocidad{
		float velx;
		float vely;
		float giro;
	};
    	enum class Estado{TURN, IDLE, FOLLOW_WALL, STRAIGHT_LINE, SPIRAL};
	using result = std::tuple<Estado,Velocidad>;
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);



public slots:
	void compute();
	int startup_check();
	void initialize(int period);
private:
	bool startup_check_flag;

    //
    std::tuple<Estado, Velocidad> Idle(const RoboCompLidar3D::TPoints &points);
    std::tuple<Estado, Velocidad> Turn(const RoboCompLidar3D::TPoints &points);
    std::tuple<Estado, Velocidad> FollowWall(const RoboCompLidar3D::TPoints &points);
    std::tuple<Estado, Velocidad> StraightLine(const RoboCompLidar3D::TPoints &points);
    std::tuple<Estado, Velocidad> Spiral(const RoboCompLidar3D::TPoints &points);
    //

    AbstractGraphicViewer *viewer;
    float distancia_al_punto_mas_cercano(RoboCompLidar3D::TPoints filtered_points);
    bool colision_inminente(const RoboCompLidar3D::TPoints filtered_points);
    float lateral_distance_to_wall(RoboCompLidar3D::TPoints filtered_points);
    void draw_lidar(const RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer);
    void print_estado(Estado e);
    Estado estado = Estado::STRAIGHT_LINE;
};

#endif
