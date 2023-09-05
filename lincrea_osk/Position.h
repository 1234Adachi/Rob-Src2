#ifndef POSITION
#define POSITION

#include <stdio.h>

class Position {

public:
    // X座標
    double x;
    // Y座標
    double y;
    // 向いている方向　ラジアン
    double angle;

public:
    Position();

    Position(const Position& src);

    void set(const Position& src);

    void addDistance(double leftMeter,double rightMeter);


    // 表示文字列用
    char buffer[256];
    inline
    char* toString(){
        sprintf(buffer, "X:%2.3lf, Y:%2.3lf, Angle:%2.3lf", x, y, angle * 180 / 3.141592 );
        return buffer;
    }

};

#endif // POSITION