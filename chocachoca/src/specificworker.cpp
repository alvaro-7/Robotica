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


#define MAX_VEL 500
#define MAX_VEL_ANG 1
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
    viewer = new AbstractGraphicViewer(this, QRectF(-5000, -5000, 10000, 10000));
    viewer->add_robot(460, 480, 0, 100, QColor("Blue"));
    viewer->show();
    viewer->activateWindow();

		timer.start(Period);
	}

}

static float velocidad_de_giro_actual;
void SpecificWorker::compute()
{
    struct timespec ts = { .tv_sec = 0, .tv_nsec = 400'000'000'0}; // 0.4 segundos
	int delta = 10;
    //RoboCompLidar3D::TData ldata = lidar3d_proxy->getLidarData("bpearl", 0, 2*M_PI, 1);
    RoboCompLidar3D::TData ldata = lidar3d_proxy->getLidarData("helios", 0, 360, 1);
    std::cout << __FUNCTION__ << " " << ldata.points.size() << std::endl;

    RoboCompLidar3D::TPoints filtered_points;
    for(const auto &p : ldata.points)
        if(p.z < 1.7 and p.z > -1.3 )
            filtered_points.push_back(RoboCompLidar3D::TPoint{.x=p.x*1000, .y=p.y*1000, .z=p.z*1000});

    std::cout << __FUNCTION__ << " " << ldata.points.size() << std::endl;
    draw_lidar(filtered_points, viewer);

	result res;
    print_estado(estado);
    switch(estado)
    {
		case Estado::TURN:
			res = Turn(filtered_points); 
			estado = std::get<0>(res);
			break;
        case Estado::IDLE:
            res = Idle(filtered_points);
            estado = std::get<0>(res);
            break;
        case Estado::FOLLOW_WALL:
            res = FollowWall(filtered_points);
            estado = std::get<0>(res);
			break;
        case Estado::STRAIGHT_LINE:
            res = StraightLine(filtered_points);
            estado = std::get<0>(res);
			break;
        case Estado::SPIRAL:
            res = Spiral(filtered_points);
            estado = std::get<0>(res);
            break;
    }
	try {
		auto vel = std::get<1>(res);
		//printf("%f %f\n", vel.velx, vel.giro);
        velocidad_de_giro_actual = vel.giro;
		omnirobot_proxy->setSpeedBase(vel.velx, vel.vely, vel.giro);
	} catch(const Ice::Exception &e)
	{  std::cout << "Error reading from Camera" << e << std::endl; 	}
}

////////////////////////////////////////////////////////////////////////////////////////////


bool SpecificWorker::colision_inminente(const RoboCompLidar3D::TPoints filtered_points)
{
    //printf("%ld\n", filtered_points.size());
    int offset = filtered_points.size()/2.40;

    auto min_elem = std::min_element(filtered_points.begin() + offset, filtered_points.end() - offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);});

    const float MIN_DISTANCE = 500;

    printf("%f", std::hypot(min_elem->x, min_elem->y));
    if(std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE){
        printf("entro\n");
        return true;
    } else{
        printf("no entro\n");
        return false;
    }
}

/*
bool SpecificWorker::colision_inminente(const RoboCompLidar3D::TPoints filtered_points)
{
    printf("%ld\n", filtered_points.size());
    int offset = filtered_points.size()/2.40;

    auto min_elem = std::min_element(filtered_points.begin() + offset, filtered_points.end() - offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);});

    const float MIN_DISTANCE = 500;

    printf("MIN_ELEM: %f\n", std::hypot(min_elem->x, min_elem->y));
    if(std::hypot(min_elem->x, min_elem->y) < MIN_DISTANCE){
        printf("entro\n");
        return true;
    } else{
        printf("no entro\n");
        return false;
    }
}
 */
/*
bool SpecificWorker::colision_inminente(RoboCompLidar3D::TPoints filtered_points){
    RoboCompLidar3D::TPoints points_in_x_equal_0;

    std::ranges::remove_copy_if(filtered_points, std::back_inserter(points_in_x_equal_0), [](auto &p){ return (p.x > 425 || p.x < -425) || p.y < 0;});
    if(points_in_x_equal_0.empty()){
        printf("NO HAY PUNTOS\n");
        return true;
    }
    auto min_elem = std::min_element(points_in_x_equal_0.begin(), points_in_x_equal_0.end(),
                                     [](auto a, auto b) { return std::hypot(a.y, a.x) < std::hypot(b.y, b.x);});

    printf("\n\n%f\n\n", min_elem->y);
    return abs(std::hypot(min_elem->y, min_elem->x)) < 525;
}*/
float SpecificWorker::distancia_al_punto_mas_cercano(RoboCompLidar3D::TPoints filtered_points){
    auto min_elem = std::min_element(filtered_points.begin(), filtered_points.end(),
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);});
    return std::hypot(min_elem->x, min_elem->y);
}
// distancia minima por los 2 lados a muro.
// Distancia palante es y
//
// Si devuelve 0, es debido a que no ve ningún muro por los dos lados.
float SpecificWorker::lateral_distance_to_wall(RoboCompLidar3D::TPoints filtered_points){
    RoboCompLidar3D::TPoints points_in_x_equal_0;
    int offset = filtered_points.size()*3/8;
    std::ranges::remove_copy_if(filtered_points, std::back_inserter(points_in_x_equal_0), [](auto &p){ return (p.y > 10 || p.y < -10);});
    /*
    for (const auto& e : points_in_x_equal_0)
        printf("n_elems: %ld, x: %f y: %f\n", points_in_x_equal_0.size(), e.x, e.y);
    */
    if(points_in_x_equal_0.empty()){
        printf("NO HAY PUNTOS\n");
        return 0.0;
    }
    auto min_elem = std::min_element(points_in_x_equal_0.begin(), points_in_x_equal_0.end(),
                                     [](auto a, auto b) { return std::abs(a.x) < std::abs(b.x);});

    return min_elem->x;
}
void SpecificWorker::print_estado(SpecificWorker::Estado e){
    switch(e){
        case Estado::TURN:
            printf("TURN\n");
            break;
        case Estado::IDLE:
            printf("IDLE\n");
            break;/*
        case Estado::FOLLOW_WALL:
            printf("FOLLOW_WALL\n");
            break;*/
        case Estado::STRAIGHT_LINE:
            printf("STRAIGHT_LINE\n");
            break;
        case Estado::SPIRAL:
            printf("SPIRAL\n");
            break;
    }

}
std::tuple<SpecificWorker::Estado, SpecificWorker::Velocidad> SpecificWorker::Idle(const RoboCompLidar3D::TPoints &points){
    printf("%f\n", lateral_distance_to_wall(points));
	return {Estado::IDLE, {0, 0, 0}};
}
// Gira para el mejor lado un tiempo aleatorio hasta no tener nada delante.
std::tuple<SpecificWorker::Estado, SpecificWorker::Velocidad> SpecificWorker::Turn(const RoboCompLidar3D::TPoints &points){
    struct timespec ts = {.tv_nsec = 0};
    if(colision_inminente(points))
    {

        ts.tv_sec = (random() % 2 + 1);
        if(velocidad_de_giro_actual > 0.1)
            nanosleep(&ts, NULL);
        bool a = true;//random() %2;
        if(a)
            return {Estado::TURN, {0, 0, MAX_VEL_ANG}};
        else
            return {Estado::TURN, {0, 0, -MAX_VEL_ANG}};

    }
    return {Estado::STRAIGHT_LINE, {500, 0, 0.0}};
}
#define REFERENCE_DISTANCE 360
std::tuple<SpecificWorker::Estado, SpecificWorker::Velocidad> SpecificWorker::FollowWall(const RoboCompLidar3D::TPoints &points){
    float a = lateral_distance_to_wall(points);
	if(a < 0) {
        if (a > -REFERENCE_DISTANCE)
            return {Estado::STRAIGHT_LINE, {MAX_VEL, 0, 0}};
        else {
            return {Estado::FOLLOW_WALL, {0, 0, -0.2}};
        }
    }
    if(a >= 0) {
        if (a < REFERENCE_DISTANCE)
            return {Estado::STRAIGHT_LINE, {MAX_VEL, 0, 0}};
        else {
            return {Estado::FOLLOW_WALL, {0, 0, 0.2}};
        }
    }
}
std::tuple<SpecificWorker::Estado, SpecificWorker::Velocidad> SpecificWorker::StraightLine(const RoboCompLidar3D::TPoints &points){
    static bool espiral_hecha = false;
    if(colision_inminente(points))
        return {Estado::TURN, {0, 0, 0}};
    if(distancia_al_punto_mas_cercano(points) > 2000 && !espiral_hecha) {
        espiral_hecha = true;
        return {Estado::SPIRAL, {0, 0, 0}};
    }
    return {Estado::STRAIGHT_LINE, {MAX_VEL, 0, 0.0}};
}
//tambien podemos cambiar lo que variamos de la espiral a la velocidad angular para que la haga + rapido
std::tuple<SpecificWorker::Estado, SpecificWorker::Velocidad> SpecificWorker::Spiral(const RoboCompLidar3D::TPoints &points){

	static float v = 0;
	float v_last = v;
	v += 0.00325;
    if(colision_inminente(points)){
	    v = 0;
        return {Estado::TURN, {0, 0, 0}};
    }
    return {Estado::SPIRAL, {v_last, 0, 1}};
    //return {Estado::SPIRAL, {MAX_VEL, 0, v_last}};
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
    for(auto &b: borrar)
    {
        viewer->scene.removeItem(b);
        delete b;
    }

    borrar.clear();

    for(const auto &p: points)
    {
        auto point = viewer->scene.addRect(-50, -50, 100, 100, QPen(QColor("blue")), QBrush(QColor("blue")));
        //auto point2 = viewer->scene.addRect(-50, -50, 100, 100, QPen(QColor("yellow")), QBrush(QColor("yellow")));
	//if(p)
        	point->setPos(p.x, p.y);
	//else
        //	point2->setPos(p.x, p.y);

        //
        borrar.push_back(point);
    }
}

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData

