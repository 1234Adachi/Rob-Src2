
#include "Position.h"
#include "RoboSpec.h"

#include "math.h"
#include "float.h"

/** 車輪間の距離 */
#define AXLE_LENGTH RoboSpec::AXLE_LENGTH

Position::Position(){
    x = 0.0;
    y= 0.0;
    angle = 0.0;
}

Position::Position(const Position& src){
    set(src);
}

void 
Position::set(const Position& src){
    x = src.x;
    y = src.y;
    angle = src.angle;
}

void 
Position::addDistance(double leftMeter,double rightMeter){
    double wheelDiff = rightMeter - leftMeter;
	
    double diffX;
	double diffY;

	double deltaAngle;

	if(wheelDiff < -DBL_EPSILON || DBL_EPSILON < wheelDiff){
		double radius = leftMeter * AXLE_LENGTH / wheelDiff + 0.5 * AXLE_LENGTH;
		deltaAngle = wheelDiff / AXLE_LENGTH;

		diffX = radius * sin(deltaAngle);
		diffY = radius * (1.0 - cos(deltaAngle));

	} else {
		// 直進したケース
		deltaAngle = 0;

		diffX = rightMeter;
		diffY = 0;
	}

	double cosAngle = cos(angle);
	double sinAngle = sin(angle);

	x += (diffX * cosAngle - diffY * sinAngle);
	y += (diffX * sinAngle + diffY * cosAngle);

	angle += deltaAngle;
}
