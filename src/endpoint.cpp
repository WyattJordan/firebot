#include "endpoint.h"


void endpoint::setCart(float xIn, float yIn){
        x = xIn;
        y = yIn;
}
float endpoint::getX(){
        return x;
}
float endpoint::getY(){
        return y;
}
float endpoint::findAngle(){
        float angle;
        float t = atan2(y, x) * 180 / 3.14159;
        angle = t>0 ? t : t + 360; // polar vals range 0:360
        return angle;
}
float endpoint::findRad(){
        float rad;
        rad = pow(x*x + y*y, 0.5);
        return rad;
}
void endpoint::clear(){
        x = 0;
        y = 0;
}
