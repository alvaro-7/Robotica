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
#include "specificworker.h"
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>


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
    return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;
    this->Period = period;
    if(this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
        // Inicializaciones personales
        viewer = new AbstractGraphicViewer(this, QRectF(-5000,-5000,10000,10000));
        viewer->add_robot(460,480,0,100,QColor("Blue"));
        viewer->show();
        viewer->activateWindow();

        timer.start(Period);
    }
}

void SpecificWorker::compute()
{
    RoboCompLidar3D::TData ldata;
    ldata = lidar3d_proxy->getLidarData("helios", 0, 360, 1);
//    ldata =  lidar3d_proxy->getLidarData("bpearl", 0, 2*M_PI, 1);
    const auto &points = ldata.points;
    if (points.empty()) return;
//    std::cout << __FUNCTION__ << " " << ldata.points.size() << std::endl;
//
//    RoboCompLidar3D::TPoints filtered_points;
//    for(const auto &p : ldata.points)
//        if(p.z < 500 and p.z > 200 )
//            filtered_points.push_back(RoboCompLidar3D::TPoint{.x=p.x*1000, .y=p.y*1000, .z=p.z*1000});
//
//    std::cout << __FUNCTION__ << " " << filtered_points.size() << std::endl;

    auto doors = doors_extractor(points);
    //auto doors = doors_extractor(filtered_points);

    auto res = std::ranges::find(doors, door_target);
    if (res != doors.end()) {
        door_target = *res;
        // qInfo() << "door_target = " << door_target;
        std::cout << "Puerta fijada" << endl;
    } else {
        qInfo() << "No door detected";
    }


    std::tuple<SpecificWorker::Estado, SpecificWorker::Velocidad> mov;
    switch(estado)
    {
        case Estado::IDLE:
            std::cout << "Estado IDLE" << endl;
            {
                SpecificWorker::Velocidad vel = {0, 0, 0};
                mov = {SpecificWorker::Estado::SEARCH_DOOR, vel};
            }
            break;

        case Estado::SEARCH_DOOR:
            std::cout << "Estado SEARCH_DOOR" << endl;
            mov = func_search_door(doors);
            break;

        case Estado::MOVE:
            std::cout << "Estado MOVE" << endl;
            mov = func_move();
            break;

        case Estado::ORIENT:
            std::cout << "Estado ORIENT" << endl;
            mov = func_orient();
            break;

        case Estado::GO_THROUGH:
            std::cout << "Estado GO_THROUGH" << endl;
            mov = func_go_through();
            break;
    }
    try {
        auto vel = std::get<1>(mov);
        omnirobot_proxy->setSpeedBase(vel.velx, vel.vely, vel.giro);
        estado = std::get<0>(mov);
        printf("Velocidad: %f %f - \n", vel.velx, vel.giro);
    } catch(const Ice::Exception &e)
    {  std::cout << "Error reading from Camera" << e << std::endl; 	}
}
///////////////////////////////////////////////////////////////////////////////

std::tuple<SpecificWorker::Estado, SpecificWorker::Velocidad> SpecificWorker::func_search_door(SpecificWorker::Doors doors){
    if(!doors.empty()){
        for(const auto &d: doors){
            if(d.angulo_robot() < door_target.angulo_robot()){
                door_target = d;
            }
        }
        SpecificWorker::Velocidad vel = {0, 0, 0};
        return {SpecificWorker::Estado::MOVE, vel};
    }
    else{
        SpecificWorker::Velocidad vel = {0.3, 0, 0.5};
        std::cout << "No se han detectado puertas, se mueve el robot para buscar" << endl;
        return {SpecificWorker::Estado::SEARCH_DOOR, vel};
    }
}

std::tuple<SpecificWorker::Estado, SpecificWorker::Velocidad> SpecificWorker::func_move(){
    RoboCompLidar3D::TPoint& p_objetivo = door_target.middle;

    float const_vel = 200;  //ESTAS VARIABLES HABRA QUE IR PROBANDO CON VARIOS VALORES PARA ENCONTRAR UNOS BUENOS
    float const_giro = -0.4; //UNA VEZ QUE SE HAYAN ENCONTRADO BUENOS VALORES SE HACEN CONSTANTES DEL .H

    float objetivo_x = p_objetivo.x;
    float objetivo_y = p_objetivo.y;
    float distancia = sqrt(pow(objetivo_x, 2) + pow(objetivo_y, 2));

    if(distancia < 150){ //LA DISTANCIA CUANDO SE ENCUENTRE UNA CON LA QUE FUNCIONA CORRECTAMENTE HAY QUE HACERLA CONSTANTE EN EL .H
        std::cout << "Distancia a la puerta adecuada, toca orientarse a ella" << endl;
        return {SpecificWorker::Estado::ORIENT, {0, 0, 0}};
    }
    else{
        float rot  = const_giro * (atan2(objetivo_y, objetivo_x));
        float velx = const_vel * (1.0 - fabs(rot));
        return {SpecificWorker::Estado::MOVE, {velx, 0, rot}};
    }
}

std::tuple<SpecificWorker::Estado, SpecificWorker::Velocidad>  SpecificWorker::func_orient(){
    float const_rot = -0.4;
    if(door_target.angulo_robot() < 0.05 || door_target.angulo_robot() > -0.05){ //UNA VEZ ENCONTRADO UNOS BUENOS PARAMETROS, HACERLOS UNA CONSTANTE
        return{SpecificWorker::Estado::GO_THROUGH, {0,0,0}};
    }
    else{
        return {SpecificWorker::Estado::ORIENT, {0,0,const_rot * door_target.angulo_robot()}};
    }
}

std::tuple<SpecificWorker::Estado, SpecificWorker::Velocidad> SpecificWorker::func_go_through(){
    if(!SpecificWorker::timer_inicializado){
        SpecificWorker::timer_inicializado = true;
        SpecificWorker::tiempo_inicio = std::chrono::steady_clock::now();
        return {SpecificWorker::Estado::GO_THROUGH, {450, 0, 0}};
    }
    else if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - SpecificWorker::tiempo_inicio).count() > SpecificWorker::tiempo_limite){
        SpecificWorker::timer_inicializado = false;
        return {SpecificWorker::Estado::SEARCH_DOOR, {0, 0, 0}};
    }
    return {SpecificWorker::Estado::GO_THROUGH, {450, 0, 0}};
}

SpecificWorker::Doors
SpecificWorker::doors_extractor(const RoboCompLidar3D::TPoints  &points)
{
    auto lines = extract_lines(points);
    auto peaks = extract_peaks(lines);
    auto doors = get_doors(peaks);
    auto final_doors = filter_doors(doors);

    draw_lidar(lines.middle, viewer); //NO SE SI SE DEBE PINTAR AQUI, O DESPUES DE LOS ESTADOS
    draw_doors(final_doors, viewer);
    return final_doors;
}

SpecificWorker::Lines SpecificWorker::extract_lines(const RoboCompLidar3D::TPoints &points)
{
    Lines lines;
    for(const auto &p: points)
    {
        if(p.z > LOW_LOW and p.z < LOW_HIGH)
            lines.low.push_back(p);
        if(p.z > MIDDLE_LOW and p.z < MIDDLE_HIGH)
            lines.middle.push_back(p);
        if(p.z > HIGH_LOW and p.z < HIGH_HIGH)
            lines.high.push_back(p);
    }
    return lines;
}

SpecificWorker::Lines SpecificWorker::extract_peaks(const SpecificWorker::Lines &lines) {
    Lines peaks;
    const float THRES_PEAK = 1000; //ESTO CUANDO SE VAYA A HACER LA IMPLEMENTACION FINAL HAY QUE PONERLO EN EL .H

    for (const auto &both: iter::sliding_window(lines.low, 2))
        if (fabs(both[1].r - both[0].r) > THRES_PEAK) {
            if (both[0].r < both[1].r) peaks.low.push_back(both[0]);
            else peaks.low.push_back(both[1]);
        }

    for (const auto &both: iter::sliding_window(lines.middle, 2))
        if (fabs(both[1].r - both[0].r) > THRES_PEAK) {
            if (both[0].r < both[1].r) peaks.middle.push_back(both[0]);
            else peaks.middle.push_back(both[1]);
        }

    for(const auto &both: iter::sliding_window(lines.high, 2))
        if(fabs(both[1].r - both[0].r) > THRES_PEAK) {
            if (both[0].r < both[1].r) peaks.high.push_back(both[0]);
            else peaks.high.push_back(both[1]);
        }

    return peaks;
}

std::tuple<SpecificWorker::Doors, SpecificWorker::Doors, SpecificWorker::Doors>
SpecificWorker::get_doors(const SpecificWorker::Lines &peaks) {

    Doors doors_low, doors_middle, doors_high;

    auto dist = [](auto a, auto b){
        return std::hypot(a.x-b.x, a.y-b.y);
    };

    const float THRES_DOOR = 500; //ESTO CUANDO SE VAYA A HACER LA IMPLEMENTACION FINAL HAY QUE PONERLO EN EL .H

    auto near_door = [dist, THRES_DOOR](auto &doors, auto d){
        for(auto &&old: doors)
        {
            // qInfo() << dist(old.left, d.left) << dist(old.right, d.right) << dist(old.right, d.left) << dist(old.left, d.right);
            if( dist(old.left, d.left) < THRES_DOOR or
                dist(old.right, d.right) < THRES_DOOR or
                dist(old.right, d.left) < THRES_DOOR or
                dist(old.left, d.right) < THRES_DOOR)
                return true;
        }
        return false;
    };

    for(auto &par : peaks.low | iter::combinations(2)){
        if(dist(par[0], par[1]) < 1400 && dist(par[0], par[1]) > 500){
            auto door = Door(par[0], par[1]);
            if(!near_door(doors_low, door)) {
                doors_low.emplace_back(par[0], par[1]);
            }
        }
    }
    for(auto &par : peaks.middle | iter::combinations(2)){
        if(dist(par[0], par[1]) < 1400 && dist(par[0], par[1]) > 500){
            auto door = Door(par[0], par[1]);
            if(!near_door(doors_middle, door)) {
                doors_middle.emplace_back(par[0], par[1]);
            }
        }
    }
    for(auto &par : peaks.high | iter::combinations(2)){
        if(dist(par[0], par[1]) < 1400 && dist(par[0], par[1]) > 500){
            auto door = Door(par[0], par[1]);
            if(!near_door(doors_high, door)) {
                doors_high.emplace_back(par[0], par[1]);
            }
        }
    }

    return std::make_tuple(doors_low, doors_middle, doors_high);
}

SpecificWorker::Doors
SpecificWorker::filter_doors(const tuple<SpecificWorker::Doors, SpecificWorker::Doors, SpecificWorker::Doors> &doors)
{
    Doors final_doors;

    auto &[dlow, dmiddle, dhigh] = doors;
    for(auto &dl: dlow)
    {
        std::apply([dl](auto&&... args)
            {((std::ranges::find(args, dl) != args.end()), ...);}, doors);
    }
    for(auto &dl: dlow)
    {
        bool equal_middle = std::ranges::find(dmiddle, dl) != dmiddle.end();
        bool equal_high = std::ranges::find(dhigh, dl) != dhigh.end();

        if (equal_middle and equal_high)
            final_doors.push_back(dl);
    }
    return dmiddle;
}


int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer)
{
    static std::vector<QGraphicsItem*> borrar;
    for(auto &b : borrar) {
        viewer->scene.removeItem(b);
        delete b;
    }
    borrar.clear();

    for(const auto &p : points)
    {
        auto point = viewer->scene.addRect(-50,-50,100, 100, QPen(QColor("Blue")), QBrush(QColor("Blue")));
        point->setPos(p.x, p.y);
        borrar.push_back(point);
    }
}

void SpecificWorker::draw_doors(const Doors &doors, AbstractGraphicViewer *viewer, QColor color)
{
    static std::vector<QGraphicsItem *> borrar;
    for (auto &b: borrar) {
        viewer->scene.removeItem(b);
        delete b;
    }
    borrar.clear();

    printf("%ld\n", doors.size());
    for (const auto &d: doors) {
        auto point = viewer->scene.addRect(-50, -50, 100, 100, QPen(color), QBrush(color));
        point->setPos(d.left.x, d.left.y);
        borrar.push_back(point);
        point = viewer->scene.addRect(-50, -50, 100, 100, QPen(color), QBrush(color));
        point->setPos(d.right.x, d.right.y);
        borrar.push_back(point);
        auto line = viewer->scene.addLine(d.left.x, d.left.y, d.right.x, d.right.y, QPen(color, 50));
        borrar.push_back(line);
    }
}

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData

