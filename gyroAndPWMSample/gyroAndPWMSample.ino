#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include <PCA9685.h>            //PCA9685用ヘッダーファイル（秋月電子通商作成）
#include <math.h>

/* This driver reads raw data from the BNO055
    2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

const double toRad= PI / 180.0;

#define DCMIN 0 // This is the 'minimum' pulse length count (out of 4096)
#define DCMAX 4095 // This is the 'maximum' pulse length count (out of 4096)
#define PWM_FREQ 50

#define SERVOMIN 150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600 // This is the 'maximum' pulse length count (out of 4096)

#define DC0_PHASE 3
#define DC0_ENBL  2
#define DC1_PHASE 1
#define DC1_ENBL  0

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

PCA9685 pwm = PCA9685(0x40);    //PCA9685のアドレス指定（アドレスジャンパ未接続時）

int defaultXdeg;
const double margin=0.1;
bool setClockwise=false;
bool setUnClockwise=false;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void){
    Serial.begin(115200);
    Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

    /* Initialise the sensor */
    if(!bno.begin()){
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }

    delay(1000);

    pwm.begin();
    pwm.setPWMFreq(PWM_FREQ);

    /* Display the current temperature */
    int8_t temp = bno.getTemp();
    Serial.print("Current Temperature: ");
    Serial.print(temp);
    Serial.println(" C");
    Serial.println("");

    bno.setExtCrystalUse(true);

    Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
    
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    defaultXdeg=euler.x();
}

//pwmモジュール経由のDCモタドラと9軸を用いて，起動時の角度に戻ろうとするサンプル
void loop(void){
    
    //角度読み出し
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    Serial.print("default:");
    Serial.println(defaultXdeg);
    Serial.print("X: ");
    Serial.println(euler.x());

    //yawを-179~180°に直す
    if(180<defaultXdeg){
        defaultXdeg=-1*(360-defaultXdeg);
    }

    //現在ベクトルを回転させて目標ベクトルに向かう際の差分角度(半時計が正)
    double now=euler.x();
    double targetX=cos(defaultXdeg*toRad);
    double targetY=sin(defaultXdeg*toRad);
    double nowX=cos(now*toRad);
    double nowY=sin(now*toRad);
    double rotate=nowX*targetY-targetX*nowY;

    Serial.println(rotate);

    int output=30;

    //目標と現在角度にマージン以上の開きがある場合，モータを動かして元に戻る
    if(abs(rotate)>margin){//回るべきか？
        Serial.print("rotate ");
        if(rotate>0.0){//反時計回りか？
            Serial.println("UnClockwise");
            if(!setUnClockwise){//すでに命令したか？
                Serial.println("go");
                dcMotorSet(0,output);
                dcMotorSet(1,-1*output);
                setClockwise=false;
                setUnClockwise=true;
            }
        }else{
            Serial.println("Clockwise");
            if(!setClockwise){//すでに命令したか？
                Serial.println("go");
                dcMotorSet(0,-1*output);
                dcMotorSet(1,output);
                setUnClockwise=false;
                setClockwise=true;
            }
        }
    }else{
        Serial.println("stop");
        if(setClockwise || setUnClockwise){
            setClockwise=false;
            setUnClockwise=false;
            dcMotorSet(0,0);
            dcMotorSet(1,0);
        }
    }
    Serial.println("");
    delay(100);
}

//DCモータのパワーを決定する
//ch:0 or 1  それぞれ右と左のモータに対応
//power:-100 ~ 100  逆転 100%～正転100%に対応
//実行状態や引数の範囲に応じて返り値あり
int dcMotorSet(int ch,int power){
    const int powerMax=100;
    const int powerMin=-100;
    const int stop=0;
    if(ch<0 || 1<ch){return -1;}
    if(power<powerMin || powerMax<power){return -1;}

    if(ch==0){
        if(0<power){
            power = map(power, stop+1, powerMax, DCMIN+1, DCMAX);
            pwm.setPWM(DC0_ENBL, 0, power);
            pwm.setPWM(DC0_PHASE, 0, stop);
        }else if(power<0){
            power = map(power, stop-1, powerMin, DCMIN+1, DCMAX);
            pwm.setPWM(DC0_ENBL, 0, power);
            pwm.setPWM(DC0_PHASE, 0,DCMAX);
        }else{
            pwm.setPWM(DC0_ENBL, 0, stop);
            pwm.setPWM(DC0_PHASE, 0,stop);
        }
        Serial.print("ch0:");
        Serial.println(power);
    }else{
        if(0<power){
            power = map(power, stop+1, powerMax, DCMIN+1, DCMAX);
            pwm.setPWM(DC1_ENBL, 0, power);
            pwm.setPWM(DC1_PHASE, 0, stop);
        }else if(power<0){
            power = map(power, stop-1, powerMin, DCMIN+1, DCMAX);
            pwm.setPWM(DC1_ENBL, 0, power);
            pwm.setPWM(DC1_PHASE, 0,DCMAX);
        }else{
            pwm.setPWM(DC1_ENBL, 0, stop);
            pwm.setPWM(DC1_PHASE, 0,stop);
        }
        Serial.print("ch1:");
        Serial.println(power);
    }

    return 0;
}

void servoWrite(int ch, int ang){ //動かすサーボチャンネルと角度を指定
    ang = map(ang, 0, 180, SERVOMIN, SERVOMAX); //角度（0～180）をPWMのパルス幅に変換
    pwm.setPWM(ch, 0, ang);
}
