
#ifndef ROBO_BODY
#define ROBO_BODY

#include "Motor.h"       
#include "ColorSensor.h"
#include "TouchSensor.h" 
#include "Position.h"

/**
 * 車輪クラス
 **/
class Wheel{
public:
    Wheel(ePortM port);

    int32_t getDelta();

    inline 
    void setPower(int8_t power){
        motor.setPWM(power);
    };

    inline
    void reset(){
        lastCount = 0;
        motor.reset();
    }
    
public:
    ev3api::Motor motor;
    int32_t lastCount;    
};


/**
 * ロボット本体
 **/
class RoboBody{
public:
    RoboBody();

    const Position& calcNewPosition();

    inline
    Position& getPosition(){ return position; }


    void setPower(int8_t leftPower, int8_t rightPower);

    inline 
    void getColor(rgb_raw_t& rawColor){
        colorSensor.getRawColor(rawColor);
    }

    inline 
    colorid_t getColorNumber(){
        return colorSensor.getColorNumber();
    }

    inline
    bool isButtonPushed(){
        return touchSensor.isPressed();
    }

protected:

    Position position;

    Wheel left;
    Wheel right;
    ev3api::ColorSensor colorSensor;
    ev3api::TouchSensor touchSensor;

};

#endif // ROBO_BODY