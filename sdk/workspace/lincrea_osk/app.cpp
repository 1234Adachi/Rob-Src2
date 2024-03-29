#include "app.h"
#include "util.h"
#include "Clock.h"
#include "RoboBody.h"
#include "Tracer.h"
#include "Env.h"
#include "ColorSensor.h"
#define EV3CPPAPI_SENSOR_H_

bool moveRobo(RoboBody& robo, Tracer& tracer, ev3api::Clock& clock);
// 今、コースのどの区間にいるのかを表す変数
static int phase = 0;
// ループ数
int loopCount = 0;
// 動作継続フラグ
bool doContinue = true;
// 直進開始
int startTime = 0;
// 右コース
int rightCrs = 0;
// 右エッジフラグ
int rightEg = 0;
// 左エッジフラグ
int leftEg = 1;
// エッジ判定
int edge = 0;
// 青色反射光の強さ
bool isBlue = false;

int colorNumber= 0;

double tempAngle = 0;


void main_task(intptr_t unused) {

  // 処理待ち時間（最大２秒?待つ）
  tslp_tsk(200000);

  syslog(LOG_NOTICE,"MAIN TASK START************************");

  ev3api::Clock clock;

  RoboBody robo;
  Tracer tracer;

  // ボタンが押されるのを待つ
  do{
    int isPushed = ev3_button_is_pressed(ENTER_BUTTON);
    if(isPushed == 1){
      break;
    }

    clock.sleep(100000);

  }while(true);

  // ロボット始動
  syslog(LOG_NOTICE,"ROBOT HAS STARTED！************************");
  do{
    doContinue = moveRobo(robo, tracer, clock);
    
    if(!doContinue){
      break;
    }

    loopCount++;
    
    if(loopCount % 100 == 0){
      int brightness = tracer.getLastBrightness();
      syslog(LOG_NOTICE,"Brightness:%d",brightness);
      //青色反射光の強さ
      rgb_raw_t rawColor;
      robo.getColor(rawColor);
      isBlue = tracer.isBlue(rawColor);
      if (isBlue){
            printf("blue\n");
      }
      Position& position = robo.getPosition();
      syslog(LOG_NOTICE,position.toString());
    }
    

    clock.sleep(4000);

  }while(true);

  ext_tsk();
}


bool moveRobo(RoboBody& robo, Tracer& tracer, ev3api::Clock& clock){
  //座標
  const Position& position = robo.calcNewPosition();
  
  //現在の時刻
  int endTime = clock.now();

  bool doContinue = true;
  // 以下 positionにより、出力と終了判定を行う。
  ONCE_MORE:
  switch(phase){
    case 0: // スタート直後
      if(0.5 < position.x){ 
        syslog(LOG_NOTICE,"switch to case 1 ************************");
        phase = 1;
        goto ONCE_MORE;
      }
      tracer.moveRoboPID(robo,60,edge); // スタートは若干遅め
      break;

    case 1: // スタートから第１カーブまで
      if(2.5 < position.x){ 
        syslog(LOG_NOTICE,"switch to case 2 ************************");
        phase = 2;
        goto ONCE_MORE;
      }
      tracer.moveRoboPID(robo,60,edge); // 全速力で走る
      break;

    case 2: // 第１カーブ
      if(-0.8 > position.y){ 
        robo.setPower(40,60);
        clock.sleep(200000);
        edge = leftEg; // 左ラインエッジに変更
        syslog(LOG_NOTICE,"switch to case 3 ************************");
        phase = 3;
        goto ONCE_MORE;
      }else if(0.8 < position.y){
        syslog(LOG_NOTICE,"switch to case 3 ************************");
        syslog(LOG_NOTICE,"RIGHT ***********************************");
        rightCrs = 1;
        phase = 3;
        goto ONCE_MORE;
      }
      tracer.moveRoboPID(robo,60,edge); // ちょっとスピードを抑える 
      break;
    
    case 3: // 第１カーブから第２カーブまで
      if(-1.8 > position.y || 1.8 < position.y){ // 
        syslog(LOG_NOTICE,"***************************** switch to case 4 ************************");
        phase = 4;
        goto ONCE_MORE;
      }
      tracer.moveRoboPID(robo,60,edge); // ちょっとスピードを抑える
      break;

    case 4: // 第２カーブからダブルループ入るまで
      if(isBlue && rightCrs == 1){ 
        printf("loop start\n");
        printf("tempAngle: %lf\n", tempAngle);
        syslog(LOG_NOTICE,"***************************** switch to case 48 ************************");
        phase = 48;
        goto ONCE_MORE;
      }else if(isBlue){
        tempAngle = position.angle;
        printf("tempAngle: %lf\n", tempAngle);
        while (true){
          
          robo.setPower(60, 30);
          if (abs(position.angle - tempAngle) > 0.2){
            break;
          }
        }   
        printf("angle_now: %lf\n", position.angle); 
        syslog(LOG_NOTICE,"***************************** switch to case 5 ************************");
        phase = 5;
        goto ONCE_MORE;
      }
      tracer.moveRoboPID(robo,60,edge); // ちょっとスピードを抑える
      break;

    case 5: // 2つ目のループ侵入・2つ目のループ内にいることを確認
      if((-1 < position.y && position.y < 0)
         || (0 < position.y && position.y < 1)){ // 
        syslog(LOG_NOTICE,"***************************** switch to case 6 ************************");
        phase = 6;
        goto ONCE_MORE;
      }
      tracer.moveRoboPID(robo,60,edge); // ちょっとスピードを抑える
      break;
  
    case 6:// 1つ目のループに戻る

      if(isBlue){ // 
        if(rightCrs == 1){ // 右コースの場合
          edge = leftEg; // 左ラインエッジに変更
        }else{             // 左コースの場合
          edge = rightEg; // 右ラインエッジに変更
        }
        syslog(LOG_NOTICE,"**************************** switch to case 7 ************************");
        phase = 7;
        goto ONCE_MORE;
      }
      tracer.moveRoboPID(robo,60,edge); // ちょっとスピードを抑える
      break;

    case 7:// 1つ目のループ内にいることを確認
      if(-1.5 > position.y || position.y > 1.5){ // 
        syslog(LOG_NOTICE,"switch to case 8 ************************");
        phase = 8;
        goto ONCE_MORE;
      }
      tracer.moveRoboPID(robo,60,edge); // ちょっとスピードを抑える
      break;

    case 8:// 180度旋回まで
      if(2.5 > position.x && isBlue){ // 
        syslog(LOG_NOTICE,"switch to case 9 ************************");
        phase = 9;
        goto ONCE_MORE;
      }
      tracer.moveRoboPID(robo,60,edge); // ちょい抑えめ
      break;

    case 9: // 180度旋回
      if(rightCrs == 1) {
        robo.setPower(70,-70);
      }else{
        robo.setPower(-30,30);
      }
      clock.sleep(1400000);
      syslog(LOG_NOTICE,"switch to case 10 ************************");
      phase =10;
      break; 

    case 10:// ダブルループを出る
      if(isBlue){ // 
        syslog(LOG_NOTICE,"switch to case 11 ************************");
        phase = 11;
        goto ONCE_MORE;
      }
      tracer.moveRoboPID(robo,60,edge); // ちょい抑える
      break;

    case 11: // 180度旋回
      if(colorNumber == COLOR_BLACK && rightCrs == 1){
        robo.setPower(30,-30);
        clock.sleep(1600000);
        edge = rightEg; // 右ラインエッジに変更
        syslog(LOG_NOTICE,"switch to case 12 ************************");
        phase =12;
        goto ONCE_MORE;
      }else if(colorNumber == COLOR_BLACK) {  
        robo.setPower(-30,30);
        clock.sleep(1600000);
        edge = leftEg; // 左ラインエッジに変更
        syslog(LOG_NOTICE,"switch to case 12 ************************");
        phase =12;
        goto ONCE_MORE;
      }
      tracer.moveRoboPID(robo,60,edge); // ちょい抑える
      break;

    case 12: // トレジャー de ハンター開始
      if(colorNumber == COLOR_RED){ // 
        syslog(LOG_NOTICE,"switch to case 13 ************************");
        phase = 13;
        goto ONCE_MORE;
      }
      tracer.moveRoboPID(robo,70,edge); // 全力で走る
      break;

    case 13: // 90度旋回して難所へ
      if(rightCrs == 1){
        robo.setPower(-30,30);
      }else{
        robo.setPower(30,-30);
      }
      clock.sleep(800000);
      startTime = clock.now();
      syslog(LOG_NOTICE,"switch to case 14 ************************");
      phase = 14;
      goto ONCE_MORE;
    break;
    
    case 14: // 90度旋回後、ゴールへ
      if(endTime-startTime > 9000000){ 
        if(colorNumber == COLOR_BLACK){
          syslog(LOG_NOTICE,"switch to case 15 ************************");
          phase = 15;
          goto ONCE_MORE;
        }
        robo.setPower(50,50);
      }
      robo.setPower(50,50);
      break;

    case 15: // 90度旋回してゴールへ
      if(rightCrs == 1){
        robo.setPower(30,-30);
      }else{
        robo.setPower(-30,30);
      }
      clock.sleep(800000);
      syslog(LOG_NOTICE,"switch to case 16 ************************");
      phase = 16;
      goto ONCE_MORE;
      break;

    case 16: // ゴールへ
      if(colorNumber == COLOR_BLUE){ // 
        syslog(LOG_NOTICE,"switch to case 50 ************************");
        phase = 50;
        goto ONCE_MORE;
      }
      tracer.moveRoboPID(robo,50,edge); // ちょい抑える
      break;

    case 48: // ゴールへ
      if(rightCrs == 1) {
        edge = leftEg; // 左ラインエッジに変更
        robo.setPower(70,-70);
      }else{
        edge = rightEg; // 右ラインエッジに変更
        robo.setPower(-70,70);
      }
      clock.sleep(500000);
      syslog(LOG_NOTICE,"switch to case 49 ************************");
      phase =49;
      goto ONCE_MORE;
      break; 

    case 49:// 直帰
      if(0.0 > position.x && colorNumber == COLOR_BLUE){ // 
        syslog(LOG_NOTICE,"switch to case 13 ************************");
        phase = 50;
        goto ONCE_MORE;
      }
      tracer.moveRoboPID(robo,60,edge); // ちょい抑えめ
      break;

    case 50: // 停止
      robo.setPower(0,0);
      doContinue = false;
      break;

  }  
  return doContinue;
}

// end::main_task[]
