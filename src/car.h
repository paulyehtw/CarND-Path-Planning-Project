#ifndef CAR_H
#define CAR_H

#include <vector>

struct Car
{
    int id;
    double x;
    double y;
    double s;
    double d;
    double v;
    double yaw;
    Car(){};
    Car(double car_x, double car_y, double car_s, double car_d, double car_v, double car_yaw)
    {
        x = car_x;
        y = car_y;
        s = car_s;
        d = car_d;
        v = car_v;
        yaw = car_yaw;
    };
    Car(int id, double car_x, double car_y, double car_s, double car_d, double car_v, double car_yaw)
    {
        id = id;
        x = car_x;
        y = car_y;
        s = car_s;
        d = car_d;
        v = car_v;
        yaw = car_yaw;
    };
};

// The data format for each car is: [ id, x, y, vx, vy, s, d]. 
// The id is a unique identifier for that car. The x, y values are in global map coordinates, 
// and the vx, vy values are the velocity components, also in reference to the global map. 
// Finally s and d are the Frenet coordinates for that car.
struct CarDetected
{
    int id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
    CarDetected(int car_id, double car_x, double car_y, double car_vx, double car_vy, double car_s, double car_d)
    {
        id = car_id;
        x = car_x;
        y = car_y;
        vx = car_vx;
        vy = car_vy;
        s = car_s;
        d = car_d;
    };
};

#endif // CAR_H