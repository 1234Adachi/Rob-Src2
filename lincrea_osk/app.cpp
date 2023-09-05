#include "app.h"
#include "util.h"
#include "Clock.h"
#include "RoboBody.h"
#include "Tracer.h"
#include "Env.h"
#define EV3CPPAPI_SENSOR_H_

bool moveRobo(RoboBody& robo, Tracer& tracer, ev3api::Clock& clock);
// 今、コースのどの区間にいるのかを表す変数
static int phase = 0;
// ループ数
int loopCount = 0;
// 動作継続フラグ
bool doContinue = true;
// 90度旋回
int turnloop = 0;
// 直進開始
int startTime = 0;


void main_task(intptr_t unused) {

  // 処理待ち時間（最大２秒?待つ）
  tslp_tsk(200000);

  syslog(LOG_NOTICE,"MAIN TASK START************************");

  ev3api::Clock clock;

  RoboBody robo;
  Tracer tracer;

  // ボタンが押されるのを待つ
  do{
    bool isPushed = robo.isButtonPushed();
    if(isPushed){
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
  //色識別
  colorid_t colorNumber = robo.getColorNumber();
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
      tracer.moveRoboPID(robo,50); // スタートは若干遅め
      //robo.setPower(70,70);
      break;


    case 1: // スタートから第１カーブまで
      if(3.0 < position.x){ 
        syslog(LOG_NOTICE,"switch to case 2 ************************");
        phase = 2;
        goto ONCE_MORE;
      }
      tracer.moveRoboPID(robo,75); // 全速力で走る
      
      break;

    case 2: // 第１カーブ
      if(-0.35 > position.y){ 
        syslog(LOG_NOTICE,"switch to case 3 ************************");
        phase = 3;
        goto ONCE_MORE;
      }
      tracer.moveRoboPID(robo,50); // ちょっとスピードを抑える 
      break;
    
    case 3: // 第１カーブから第２カーブまで
      if(-2.0 > position.y ){ // 
        syslog(LOG_NOTICE,"switch to case 4 ************************");
        phase = 4;
        goto ONCE_MORE;
      }
      tracer.moveRoboPID(robo,70); // 全速力で走る
      //robo.setPower(80,80);
      break;

    case 4: // 第２カーブ
      if(3.4 > position.x){ 
        syslog(LOG_NOTICE,"switch to case 5 ************************");
        phase = 6;
        goto ONCE_MORE;
      }
      tracer.moveRoboPID(robo,50); // ちょっとスピードを抑える
      break;

    // case 5: // LAPまで
    //   if(3.0 > position.x ){ // 
    //     syslog(LOG_NOTICE,"switch to case 6 ************************");
    //     phase = 6;
    //     goto ONCE_MORE;
    //   }
    //   tracer.moveRoboPID(robo,70); // 全速力で走る
    //   break;

    case 6: // 1つ目のループに入る
    
    if(2.5 > position.x && colorNumber == COLOR_BLUE){ // 
      syslog(LOG_NOTICE,"switch to case 7 ************************");
      phase = 7;
      goto ONCE_MORE;
    }
    tracer.moveRoboPID(robo,60); // ちょい抑えめ
    break;

    case 7: // 2つ目のループに入る
    
    if(-0.7 < position.y){ // 
      syslog(LOG_NOTICE,"switch to case 8 ************************");
      phase = 8;
      goto ONCE_MORE;
    }

    tracer.ChangemoveRoboPID(robo,60); // 全速力で走る
    break;
  
  case 8:// 1つ目のループに戻る

  if(2.5 > position.x && colorNumber == COLOR_BLUE){ // 
    syslog(LOG_NOTICE,"switch to case 9 ************************");
    phase = 9;
    goto ONCE_MORE;
  }
  tracer.ChangemoveRoboPID(robo,60); // ちょい抑えめ
  break;

  case 9:// 1つ目のループに戻る

  if(-1.5 > position.y){ // 
    syslog(LOG_NOTICE,"switch to case 10 ************************");
    phase = 10;
    goto ONCE_MORE;
  }
  tracer.moveRoboPID(robo,60); // ちょい抑えめ
  break;

  case 10:// 1つ目のループに戻る

  if(2.5 > position.x && colorNumber == COLOR_BLUE){ // 
    syslog(LOG_NOTICE,"switch to case 11 ************************");
    phase = 11;
    goto ONCE_MORE;
  }
  tracer.moveRoboPID(robo,60); // ちょい抑えめ
  break;

  case 11: // 180度旋回
       
  robo.setPower(-30,30);
  clock.sleep(1400000);
  syslog(LOG_NOTICE,"switch to case 12 ************************");
  phase =12;

  break; 

  case 12:// 直帰

  if(3.4 < position.x){ // 
    syslog(LOG_NOTICE,"switch to case 13 ************************");
    phase = 13;
    goto ONCE_MORE;
  }
  tracer.moveRoboPID(robo,70); // ちょい抑えめ

  break;

  case 13: // 180度旋回
       
  robo.setPower(-30,30);
  clock.sleep(1600000);
  syslog(LOG_NOTICE,"switch to case 14 ************************");
  phase =14;

  break;

  case 14: // 配置のやつ開始
       
  if(colorNumber == COLOR_RED){ // 
    syslog(LOG_NOTICE,"switch to case 15 ************************");
    phase = 15;
    goto ONCE_MORE;
  }
  tracer.ChangemoveRoboPID(robo,60); // ちょい抑えめ

  break;

  case 15: // 90度旋回して難所へ

  if(turnloop == 0){
    syslog(LOG_NOTICE,"90度旋回 ************************");
    robo.setPower(30,-30);
    clock.sleep(840000);
    startTime = clock.now();
  }else if(endTime-startTime > 8000000){ 
    if(colorNumber == COLOR_BLACK){
      turnloop = 0;
      syslog(LOG_NOTICE,"switch to case 16 ************************");
      phase = 16;
      goto ONCE_MORE;
    }
    robo.setPower(50,50);
  }

  robo.setPower(50,50);
  turnloop++;
  break;

  case 16: // 90度旋回して難所へ

  // if(colorNumber == COLOR_BLACK){ // 
  //   // robo.setPower(0,30);
  //   syslog(LOG_NOTICE,"switch to case 17 ************************");
  //   phase = 17;
  //   goto ONCE_MORE;
  // }
  robo.setPower(-30,30);
  clock.sleep(800000);
  syslog(LOG_NOTICE,"switch to case 16 ************************");
  phase = 16;
  break;

  case 17: // 90度旋回してゴールへ
  if(turnloop == 0){
    syslog(LOG_NOTICE,"90度旋回 ************************");
    robo.setPower(30,-30);
    clock.sleep(840000);
    startTime = clock.now();

  }else if(colorNumber == COLOR_BLUE){ // 
    syslog(LOG_NOTICE,"switch to case 50 ************************");
    phase = 50;
    goto ONCE_MORE;
  }
  tracer.ChangemoveRoboPID(robo,50); // ちょい抑えめ
  turnloop++;

  break;








  case 49:// 直帰

  if(-0.1 > position.x){ // 
    syslog(LOG_NOTICE,"switch to case 13 ************************");
    phase = 50;
    goto ONCE_MORE;
  }
  tracer.moveRoboPID(robo,70); // ちょい抑えめ
  break;


case 50: // 停止
// 今回はここで止まる…あとは任せた！
robo.setPower(0,0);
doContinue = false;
break;

  }  

  return doContinue;
}

// end::main_task[]
