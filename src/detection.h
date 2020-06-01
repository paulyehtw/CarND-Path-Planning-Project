#ifndef DETECTION_H
#define DETECTION_H

#include <vector>

// The data format for each detected car is: [ id, x, y, vx, vy, s, d]
struct Detection
{
    int id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
    Detection(int car_id, double car_x, double car_y, double car_vx, double car_vy, double car_s, double car_d)
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

#endif // DETECTION_H