#include "Tracer.h"

#include "Env.h"

Tracer::Tracer(LineSide side){
    lineSide = side;
    grayBrightness = GRAY_COLOR;
    lastBrightness = -1; // 負の値にして有効でないことを示す 
    prevDiff = -1;   
}

// オンオフ制御
void
Tracer::calcOnOff(int8_t& leftPower, int8_t& rightPower,int8_t maxPower,const rgb_raw_t& rawColor){

    int brightness = getBrightness(rawColor);

    // 右エッジをトレースするつもりで計算
    if(grayBrightness < brightness){
        // 白フロア上にいる
        leftPower = 0;
        rightPower = maxPower;
    } else {
        // 黒線上にいる
        leftPower = maxPower;
        rightPower = 0;
    }
    
    // 左エッジトレースなら、左右のパワーをスワップ
    if(lineSide == LEFT){
        int8_t power = leftPower;
        leftPower = rightPower;
        rightPower = power;
    }

    lastBrightness = brightness;

    return;
}

// P制御
#define COLOR_WIDTH 100.0
void
Tracer::calcPID(int8_t& leftPower, int8_t& rightPower,int8_t maxPower,const rgb_raw_t& rawColor){

    int brightness = getBrightness(rawColor);
    //printf("brightness: %d\n", brightness);

    // 右エッジをトレースするつもりで計算
    if(grayBrightness < brightness){
        double diff = brightness - grayBrightness;
        // 白フロア上にいる
        leftPower = (double)maxPower * (1.0 - diff / COLOR_WIDTH);
        rightPower = maxPower;
    } else {
        double diff = grayBrightness - brightness;
        // 黒線上にいる
        leftPower = maxPower;
        rightPower =  (double)maxPower * (1.0 - diff / COLOR_WIDTH);
    }
    
    // 左エッジトレースなら、左右のパワーをスワップ
    if(lineSide == LEFT){
        int8_t power = leftPower;
        leftPower = rightPower;
        rightPower = power;
    }

    lastBrightness = brightness;
    
    return;
}

 // PD制御
 void
 Tracer::calcPD(int8_t& leftPower, int8_t& rightPower,int8_t maxPower,const rgb_raw_t& rawColor,int& prevDiff){
     double  Kp  = 0.8;//Pゲイン
     double  Kd  = 0.2;//Dゲイン
     double diff = 0.0;
     double diffSpan = 0.0;
     int brightness = getBrightness(rawColor);

     // 右エッジをトレースする場合
     if(grayBrightness < brightness){
         diff = brightness - grayBrightness;
         diffSpan = brightness - grayBrightness - prevDiff;
         if(diffSpan < 0){
             diffSpan = diffSpan * -1;
         }
         float turn = Kp * diff  + Kd * diffSpan;
         // 白フロア上にいる
         leftPower = (double)maxPower * (1.0 - turn / COLOR_WIDTH);
         rightPower = (double)maxPower;
     } else {
         diff = grayBrightness - brightness;
         diffSpan = grayBrightness - brightness - prevDiff;
         
         if(diffSpan < 0){
             diffSpan = diffSpan * -1;
         }
         float turn = Kp * diff  + Kd * diffSpan;
         // 黒線上にいる
         leftPower = (double)maxPower;
         rightPower = (double)maxPower * (1.0 - turn / COLOR_WIDTH);
     }

     // 左エッジトレースなら、左右のパワーをスワップ
     if(lineSide == LEFT){
         int8_t power = leftPower;
         leftPower = rightPower;
         rightPower = power;
     }

     lastBrightness = brightness;
     prevDiff = diff;
     return;
 }

void 
Tracer::moveRoboOnOf(RoboBody& robo, int8_t maxPower){
    rgb_raw_t rawColor;
    robo.getColor(rawColor);

    int8_t leftPower, rightPower;
    calcOnOff(leftPower,rightPower,maxPower,rawColor);
    robo.setPower(leftPower, rightPower);
}

// P制御
void 
Tracer::moveRoboPID(RoboBody& robo, int8_t maxPower,int8_t edge){
    rgb_raw_t rawColor;
    robo.getColor(rawColor);

    int8_t leftPower, rightPower;
    if(edge == 0){
      calcPID(leftPower,rightPower,maxPower,rawColor);
      //calcPD(leftPower,rightPower,maxPower,rawColor, prevDiff);
      robo.setPower(leftPower, rightPower);
    }else if(edge == 1){
      calcPID(rightPower,leftPower,maxPower,rawColor);
      //calcPD(rightPower,leftPower,maxPower,rawColor, prevDiff);
      robo.setPower(leftPower, rightPower);
    }
}
