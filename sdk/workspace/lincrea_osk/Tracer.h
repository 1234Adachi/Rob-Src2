#ifndef TRACER_H
#define TRACER_H

#include "ColorSensor.h" 
#include "RoboBody.h"

enum LineSide{
    LEFT,
    RIGHT
};

class Tracer{
public:
    Tracer(LineSide side = RIGHT);

    void calcOnOff(int8_t& leftPower, int8_t& rightPower,int8_t maxPower,const rgb_raw_t& rawColor);

    void calcPID(int8_t& leftPower, int8_t& rightPower,int8_t maxPower,const rgb_raw_t& rawColor);

    void moveRoboOnOf(RoboBody& robo, int8_t maxPower);

    void moveRoboPID(RoboBody& robo,int8_t maxPower,int8_t edge);
    
    inline
    static int getBrightness(const rgb_raw_t& rawColor){
        // 青線部分の影響を排除する為、、blueは足さない
        return (rawColor.r + rawColor.g);
    }
    
    inline
    static int getBlue(const rgb_raw_t& rawColor){
        return (rawColor.b);
    }

    inline
    int getLastBrightness(){
        return lastBrightness;
    }
    
    inline
    static bool isBlue(const rgb_raw_t& rawColor){
        
        bool blue = false;
        
        // rgbのうち最大・最小を判定
        int arrayRgb[] = {rawColor.r, rawColor.g, rawColor.b};
        double maxColor = INT16_MIN;
        double minColor = INT16_MAX;
        int maxColorType = 0; // 0:赤 1:緑 2:青
        int h; //色相
        double s; //彩度
        int v; //明度
        int count = 0;
        for (int i: arrayRgb) {
            if (i < minColor) {
                minColor = i;
            }
            if (i > maxColor) {
                maxColor = i;
                maxColorType = count;
            }
            count++;
            //printf("count : %d\n", count);
        }

        // 色相(h)を求める
        if(maxColorType == 0){
            h = 60*((rawColor.g - rawColor.b) / (maxColor - minColor));
        }else if(maxColorType == 1){
            h = 60*((rawColor.b - rawColor.r) / (maxColor - minColor)) + 120;
        }else if(maxColorType == 2){
            h = 60*((rawColor.r - rawColor.g) / (maxColor - minColor)) + 240;
        }

        // 彩度(s)を求める
        //printf("max: %d, min :%d\n", maxColor, minColor);
        s = (maxColor - minColor) / maxColor;

        // 明度(v)を求める
        v = maxColor;
        //printf("r: %d, g: %d, b: %d\n", rawColor.r, rawColor.g, rawColor.b);
        //printf("h: %d, s: %lf, v: %d\n", h, s, v);
        if (180 < h < 300 && 0.8 < s){
            blue = true;
        }
        
        return blue;
    }

protected:
    LineSide lineSide;
    int grayBrightness;

    int lastBrightness;
};

#endif // TRACER_H
