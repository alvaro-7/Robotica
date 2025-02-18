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
#include <tuple>
#include <chrono>

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);

    public slots:
        void compute();
        int startup_check();
        void initialize(int period);

    private:
        bool startup_check_flag;
        AbstractGraphicViewer *viewer;

        //CONSTANTES
        const int LIMITE_CONTADOR_PUERTAS = 30;
        const float CONST_ROT = 0.4;
        const float VEL_MOVE = 200;
        const float CONST_DIST = 900;
        const float LIMITE_ANGULO = 0.01;
        const float VEL_GO = 450;
        const int TIEMPO_LIMITE = 14000;
        const float THRES_PEAK = 1000;
        const float LIMITE_LINES_BAJO = 1000;
        const float LIMITE_LINES_ALTO = 2000;
        const float LIMITE_DOORS_BAJO = 500;
        const float LIMITE_DOORS_ALTO = 1400;

        struct Door
        {
            RoboCompLidar3D::TPoint left, right, middle;
            const float THRESHOLD = 500;
            Door(){ left = right = middle = RoboCompLidar3D::TPoint(0,0,0);};
            Door(const RoboCompLidar3D::TPoint &left_, const RoboCompLidar3D::TPoint &right_) : left(left_), right(right_)
            {
                middle.x = (left.x + right.x)/2;
                middle.y = (left.y + right.y)/2;
            };
            bool operator==(const Door &d) const
            {
                return std::hypot(d.middle.x - middle.x, d.middle.y - middle.y) < THRESHOLD;
            };
            Door& operator=(const Door &d)
            {
                left = d.left;
                right = d.right;
                middle = d.middle;
                return *this;
            };
            void print() const{
                cout << "Puntos middle: " << middle.x << ", " << middle.y << endl;
            };
            float angulo_robot() const{
                return atan2(middle.x, middle.y);
            }
        };
        using Doors = std::vector<Door>;
        Door door_target;

        int contador_cambio_puerta = 0;

        struct Velocidad{
            float velx;
            float vely;
            float giro;
        };

		std::chrono::steady_clock::time_point tiempo_inicio;
    	bool timer_inicializado = false;


        enum class Estado{IDLE, SEARCH_DOOR, ORIENT, MOVE, GO_THROUGH};
        Estado estado = Estado::SEARCH_DOOR;

        std::tuple<SpecificWorker::Estado, SpecificWorker::Velocidad> func_search_door(SpecificWorker::Doors doors);
        std::tuple<SpecificWorker::Estado, SpecificWorker::Velocidad> func_move();
        std::tuple<SpecificWorker::Estado, SpecificWorker::Velocidad> func_orient();
        std::tuple<SpecificWorker::Estado, SpecificWorker::Velocidad> func_go_through();
        RoboCompLidar3D::TPoint perpPoint();

        struct Lines
        {
            RoboCompLidar3D::TPoints low, middle, high;
        };

        void draw_lidar(const RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer);
        Lines extract_lines(const RoboCompLidar3D::TPoints &points);

        SpecificWorker::Lines extract_peaks(const Lines &peaks);
        void draw_doors(const Doors &doors, AbstractGraphicViewer *viewer, QColor = QColor("green"));

        std::tuple<Doors, Doors, Doors> get_doors(const Lines &lines);

        Doors filter_doors(const std::tuple<Doors, Doors, Doors> &doors);
        Doors doors_extractor(const RoboCompLidar3D::TPoints &filtered_points);

        float break_adv(float rot);
        float break_rot(float rot);
        float move_robot(float rot);        
};


#endif