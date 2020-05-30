#ifndef CAR_H
#define CAR_H

struct Car
{
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
};

#endif // CAR_H