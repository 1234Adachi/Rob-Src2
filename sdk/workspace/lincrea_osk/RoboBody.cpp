#include "RoboBody.h"
#include "RoboSpec.h"

/**********************************************************************
 * Wheel関連メソッド
 **********************************************************************/

Wheel::Wheel(ePortM port) : motor(port) {
    lastCount = 0;
}


int32_t 
Wheel::getDelta(){
    int32_t count = motor.getCount();
    int32_t delta = count - lastCount;
    lastCount = count;
    return delta;
}


/**********************************************************************
 * RoboBody関連メソッド
 **********************************************************************/
RoboBody::RoboBody() : left(PORT_C),right(PORT_B), colorSensor(PORT_3), touchSensor(PORT_1) {
        // Do Nothing
}

/**
 * モーターの回転数から新しい位置を計算する。
 */
const Position&
RoboBody::calcNewPosition(){
    double leftMeter = (double)left.getDelta() * RoboSpec::LEFT_WHEEL / 360.0;
    double rightMeter = (double)right.getDelta() * RoboSpec::RIGHT_WHEEL / 360.0;

    position.addDistance(leftMeter,rightMeter);

    return position;
}

void
RoboBody::setPower(int8_t leftPower, int8_t rightPower){
    calcNewPosition();
    left.setPower(leftPower);
    right.setPower(rightPower);
}
