#include "refpoint.h"

refpoint::refpoint(){}

refpoint::refpoint(float X, float Y) : y(Y), x(X){}

void refpoint::setCart(float xIn, float yIn){
        x = xIn;
        y = yIn;
}
float refpoint::getX(){
        return x;
}
float refpoint::getY(){
        return y;
}
float refpoint::findAngle(){
        float angle;
        float t = atan2(y, x) * 180 / 3.14159;
        angle = t>0 ? t : t + 360; // polar vals range 0:360
        return angle;
}
float refpoint::findRad(){
        float rad;
        rad = pow(x*x + y*y, 0.5);
        return rad;
}
void refpoint::clear(){
        x = 0;
        y = 0;
}
