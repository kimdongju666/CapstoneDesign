#include <Wire.h >
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7,8);
const byte address[6] = "00001";
double joystick[7];
const double PITCH = 0.00;
const double ROLL = 0.00;
const double YAW = 0.00;

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN); // 거리가 먼 경우 GND와 3.3v에 바이패스 커패시터 사용 

  radio.startListening();

  Wire.begin();
  Wire.setClock(400000);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x6b);
  Wire.write(0x0);
  Wire.endTransmission(true);
}

int throttle =0;
void loop() {
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);
  int16_t GyXH = Wire.read();  
  int16_t GyXL = Wire.read();
  int16_t GyYH = Wire.read();  
  int16_t GyYL = Wire.read();
  int16_t GyZH = Wire.read();  
  int16_t GyZL = Wire.read();
  int16_t GyX = GyXH <<8 |GyXL;
  int16_t GyY = GyYH <<8 |GyYL;
  int16_t GyZ = GyZH <<8 |GyZL;

  static int32_t GyXSum =0, GyYSum =0, GyZSum =0;
  static double GyXOff =0.0, GyYOff =0.0, GyZOff =0.0;
  static int cnt_sample =1000;
  if(cnt_sample >0) {
  GyXSum += GyX, GyYSum += GyY, GyZSum += GyZ;
  cnt_sample --;
  if(cnt_sample ==0) {      
    GyXOff = GyXSum /1000.0;      
    GyYOff = GyYSum /1000.0;      
    GyZOff = GyZSum /1000.0;
  }
  delay(1);
  return;    
  }
  double GyXD = GyX - GyXOff;
  double GyYD = GyY - GyYOff;
  double GyZD = GyZ - GyZOff;
  double GyXR = GyXD /131;
  double GyYR = GyYD /131;
  double GyZR = GyZD /131;

  static unsigned long t_prev =0;
  unsigned long t_now = micros();
  double dt = (t_now - t_prev)/1000000.0;
  t_prev = t_now;

  static double AngleX =0.0, AngleY =0.0, AngleZ =0.0;
  AngleX += GyXR *dt;
  AngleY += GyYR *dt;
  AngleZ += GyZR *dt;
  if(throttle ==0) AngleX =AngleY =AngleZ =0.0;

  static double tAngleX =0.0, tAngleY =0.0, tAngleZ =0.0;
  double eAngleX = tAngleX - AngleX;
  double eAngleY = tAngleY - AngleY;
  double eAngleZ = tAngleZ - AngleZ;
  double Kp =1.0;
  double BalX = Kp *eAngleX;
  double BalY = Kp *eAngleY;
  double BalZ = Kp *eAngleZ;

  double Kd =1.0;
  BalX += Kd *-GyXR;
  BalY += Kd *-GyYR;
  BalZ += Kd *-GyZR;
  if(throttle ==0) BalX =BalY =BalZ =0.0;

  double Ki =1.0;
  static double ResX =0.0, ResY =0.0, ResZ =0.0;
  ResX += Ki *eAngleX *dt;
  ResY += Ki *eAngleY *dt;
  ResZ += Ki *eAngleZ *dt;  
  if(throttle ==0) ResX =ResY =ResZ =0.0;
  BalX += ResX;
  BalY += ResY;
  BalZ += ResX; 

  if (radio.available()) {
    radio.read(joystick, sizeof(joystick));

    double pitch = joystick[0] - PITCH;
    double roll = joystick[1] - ROLL;
    double yaw = joystick[2] - YAW;
    if (joystick[5] == 1) {
    int b = joystick[4];
    int altitude = joystick[6];
    switch(b) {
      case 0:
        Serial.print("PITCH : ");
        Serial.print(joystick[0]);
        Serial.print(", ");
        Serial.print("ROLL : ");
        Serial.print(joystick[1]);
        Serial.print(", ");
        Serial.print("YAW : ");
        Serial.print(joystick[2]);
        if ((pitch <= 10.00 && pitch >= -10.00) && (roll <= 10.00 && roll >= -10.00)
        && (yaw <= 10.00 && yaw >= -10.00) && (altitude <= 600 && altitude >= 400)) {
          analogWrite(6, 50);
          analogWrite(10, 50);
          analogWrite(9, 50);
          analogWrite(5, 50);
        } // 제자리 비행

        // 드론 앞 뒤 움직임 속도 제어
       else if (pitch >= -30.00 && pitch <= -10.00) {
          analogWrite(6, 25);
          analogWrite(10, 25);
          analogWrite(9, 75);
          analogWrite(5, 75);
        } // 전진 속도 1단계
        else if ((pitch >= -30.00 && pitch <= -10.00) && (roll >= -30.00 && roll <= -10.00)) {
          analogWrite(6, 25);
          analogWrite(10, 75);
          analogWrite(9, 75);
          analogWrite(5, 75);
        }// 전진 + 좌측 방향 속도 1단계
        else if ((pitch >= -30.00 && pitch <= -10.00) && (roll <= 30.00 && roll >= 10.00)) {
          analogWrite(6, 75);
          analogWrite(10, 25);
          analogWrite(9, 75);
          analogWrite(5, 75);
        } // 전진 + 우측 방향 속도 1단계
       else if (pitch >= -50.00 && pitch <= -30.00) {
          analogWrite(6, 50);
          analogWrite(10, 50);
          analogWrite(9, 150);
          analogWrite(5, 150);
       } // 전진 속도 2단계
       else if ((pitch >= -50.00 && pitch <= -30.00) && (roll >= -50.00 && roll <= -30.00)) {
          analogWrite(6, 50);
          analogWrite(10, 150);
          analogWrite(9, 150);
          analogWrite(5, 150);
       } // 전진 + 좌측 방향 속도 2단계
       else if ((pitch >= -50.00 && pitch <= -30.00) && (roll <= 50.00 && roll >= 30.00)) {
          analogWrite(6, 150);
          analogWrite(10, 50);
          analogWrite(9, 150);
          analogWrite(5, 150);
       } // 전진 + 우측 방향 속도 2단계
       else if (pitch >= -70.00 && pitch <= -50.00) {
          analogWrite(6, 75);
          analogWrite(10, 75);
          analogWrite(9, 225);
          analogWrite(5, 225);
        } // 전진 속도 3단계
        else if ((pitch >= -70.00 && pitch <= -50.00) && (roll >= -70.00 && roll <= -50.00)) {
          analogWrite(6, 75);
          analogWrite(10, 225);
          analogWrite(9, 225);
          analogWrite(5, 225);
        } // 전진 + 좌측 방향 속도 3단계
        else if ((pitch >= -70.00 && pitch <= -50.00) && (roll >= -70.00 && roll <= -50.00)) {
          analogWrite(6, 225);
          analogWrite(10, 75);
          analogWrite(9, 225);
          analogWrite(5, 225);
        } // 전진 + 우측 방향 속도 3단계
        
       else if (pitch <= 30.00 && pitch >= 10.00) {
          analogWrite(6, 75);
          analogWrite(10, 75);
          analogWrite(9, 25);
          analogWrite(5, 25);
       } // 후진 속도 1단계
       else if ((pitch <= 30.00 && pitch >= 10.00) && (roll >= -30.00 && roll <= -10.00)) {
          analogWrite(6, 75);
          analogWrite(10, 75);
          analogWrite(9, 75);
          analogWrite(5, 25);
       } // 후진 + 좌측 속도 1단계
       else if ((pitch <= 30.00 && pitch >= 10.00) && (roll <= 30.00 && roll >= 10.00)) {
          analogWrite(6, 75);
          analogWrite(10, 75);
          analogWrite(9, 25);
          analogWrite(5, 75);
       } // 후진 + 우측 속도 1단계
       else if (pitch <= 50.00 && pitch >= 30.00) {
           analogWrite(6, 150);
          analogWrite(10, 150);
          analogWrite(9, 50);
          analogWrite(5, 50);
       } //후진 속도 2단계
       else if ((pitch <= 50.00 && pitch >= 30.00) && (roll >= -50.00 && roll <= -30.00)) {
           analogWrite(6, 150);
          analogWrite(10, 150);
          analogWrite(9, 150);
          analogWrite(5, 50);
       } // 후진 + 좌측 방향 속도 2단계
       else if ((pitch <= 50.00 && pitch >= 30.00) && (roll <= 50.00 && roll >= 30.00)) {
           analogWrite(6, 150);
          analogWrite(10, 150);
          analogWrite(9, 50);
          analogWrite(5, 150);
       } // 후진 + 우측 방향 속도 2단계
        else if (pitch <= 70.00 && pitch >= 50.00) {
          analogWrite(6, 225);
          analogWrite(10, 225);
          analogWrite(9, 75);
          analogWrite(5, 75);
        } //후진 속도 3단계
        else if ((pitch <= 70.00 && pitch >= 50.00) && (roll >= -70.00 && roll <= -50.00)) {
          analogWrite(6, 225);
          analogWrite(10, 225);
          analogWrite(9, 225);
          analogWrite(5, 75);
        } //후진 + 좌측 방향 속도 3단계
        else if ((pitch <= 70.00 && pitch >= 50.00) && (roll <= 70.00 && roll >= 50.00)) {
          analogWrite(6, 225);
          analogWrite(10, 225);
          analogWrite(9, 75);
          analogWrite(5, 225);
        } //후진 + 우측 방향 속도 3단계
        
        // 드론 좌 우 움직임 속도 제어 
        else if (roll >= -30.00 && roll <= -10.00) {
          analogWrite(6, 25);
          analogWrite(10, 75);
          analogWrite(9, 75);
          analogWrite(5, 25);
        } // 드론 좌측 방향 속도 1단계
        else if (roll >= -50.00 && roll <= -30.00) {
          analogWrite(6, 50);
          analogWrite(10, 150);
          analogWrite(9, 150);
          analogWrite(5, 50);
       } // 드론 좌측 방향 속도 2단계
        else if (roll >= -70.00 && roll <= -50.00) {
          analogWrite(6, 75);
          analogWrite(10, 225);
          analogWrite(9, 225);
          analogWrite(5, 75);
       } // 드론 좌측 방향 속도 3단계
        else if (roll <= 30.00 && roll >= 10.00) {
          analogWrite(6, 75);
          analogWrite(10, 25);
          analogWrite(9, 25);
          analogWrite(5, 75);
       } // 드론 우측 방향 속도 1단계
        else if (roll <= 50.00 && roll >= 30.00) {
          analogWrite(6, 150);
          analogWrite(10, 50);
          analogWrite(9, 50);
          analogWrite(5, 150);
       } // 드론 우측 방향 속도 1단계
        else if (roll <= 70.00 && roll >= 50.00) {
          analogWrite(6, 225);
          analogWrite(10, 75);
          analogWrite(9, 75);
          analogWrite(5, 225);
        } // 드론 우측 방향 속도 1단계
        
        // 드론 좌 우 수평 회전 속도 제어
       else if (yaw >= -40.00 && yaw <= -20.00) {
          analogWrite(6, 75);
          analogWrite(10, 25);
          analogWrite(9, 75);
          analogWrite(5, 25);
       }
       else if (yaw >= -60.00 && yaw <= -40.00) {
          analogWrite(6, 150);
          analogWrite(10, 50);
          analogWrite(9, 150);
          analogWrite(5, 50);
       }
       else if (yaw >= -80.00 && yaw <= -60.00) {
          analogWrite(6, 225);
          analogWrite(10, 75);
          analogWrite(9, 225);
          analogWrite(5, 75);
       }
       else if (yaw <= 40.00 && yaw >= 20.00) {
          analogWrite(6, 25);
          analogWrite(10, 75);
          analogWrite(9, 25);
          analogWrite(5, 75);
       }
        else if (yaw <= 60.00 && yaw >= 40.00) {
          analogWrite(6, 50);
          analogWrite(10, 150);
          analogWrite(9, 50);
          analogWrite(5, 150);
       }
       else if (yaw <= 80.00 && yaw >= 60.00) {
          analogWrite(6, 75);
          analogWrite(10, 225);
          analogWrite(9, 75);
          analogWrite(5, 225);
       }
       else if (altitude > 600 && altitude <= 800) {
        analogWrite(6,100);
        analogWrite(10,100);
        analogWrite(9,100);
        analogWrite(5,100);
       }
       else if (altitude > 800 && altitude <= 1000) {
        analogWrite(6,200);
        analogWrite(10,200);
        analogWrite(9,200);
        analogWrite(5,200);
       }
       else if (altitude >= 200 && altitude < 400) {
        analogWrite(6,25);
        analogWrite(10,25);
        analogWrite(9,25);
        analogWrite(5,25);
       }
       else if (altitude >= 0 && altitude < 200) {
        analogWrite(6,0);
        analogWrite(10,0);
        analogWrite(9,0);
        analogWrite(5,0);
       }
        break;
        
       case 1:
        Serial.println("Stop!!");
        analogWrite(6, 25);
        analogWrite(10, 25);
        analogWrite(9, 25);
        analogWrite(5, 25);
        }
    
    
    }
    else if(joystick[5] == 0) {
        analogWrite(6, 0);
        analogWrite(10, 0);
        analogWrite(9, 0);
        analogWrite(5, 0);
      }
}
}


  /*double speedA = throttle + BalY + BalX + BalZ;  
  double speedB = throttle - BalY + BalX - BalZ; 
  double speedC = throttle - BalY - BalX + BalZ;
  double speedD = throttle + BalY - BalX - BalZ;

  int iSpeedA = constrain((int)speedA, 0, 250);
  int iSpeedB = constrain((int)speedB, 0, 250);
  int iSpeedC = constrain((int)speedC, 0, 250);
  int iSpeedD = constrain((int)speedD, 0, 250);

  analogWrite(6, iSpeedA);
  analogWrite(10,iSpeedB);
  analogWrite(9, iSpeedC);
  analogWrite(5, iSpeedD);*/

 //  static int cnt_loop;
 //  cnt_loop ++;
 //  if(cnt_loop%100 !=0) return;

 //  Serial.print("GyY = "); Serial.print(GyY);
 //  Serial.print("GyYD = "); Serial.print(GyYD);
 //  Serial.print("GyYR = "); Serial.print(GyYR);
 //  Serial.print("dt = "); Serial.print(dt, 6);
 //  Serial.print("AngleY = "); Serial.print(AngleY);
 //  Serial.print(" | BalY = "); Serial.print(BalY);
 //  Serial.print("A = ");Serial.print(speedA);
 //  Serial.print(" | B = ");Serial.print(speedB);
 //  Serial.print(" | C = ");Serial.print(speedC);
 //  Serial.print(" | D = ");Serial.print(speedD);
 //  Serial.println();
//}
