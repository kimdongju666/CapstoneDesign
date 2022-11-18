#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9250.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

int Stay = 3;
int GoStop = 4;
const double PITCH = 0.00;
const double ROLL = 0.00;
const double YAW = 0.00;
int count = 0;

RF24 radio(7, 8); // SPI 버스에 nRF24L01 라디오를 설정하기 위해 CE, CSN를 선언.
const byte address[6] = "00001";

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 accelgyro;
I2Cdev   I2C_M;

void get_one_sample_date_mxyz();
void getAccel_Data(void);
void getGyro_Data(void);
void getCompass_Data(void);
void getCompassDate_calibrated ();

uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;

float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

#define sample_num_mdate  5000

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

float temperature;
float pressure;
float atm;
float altitude;

double joystick[7];

void setup()
{
    pinMode(Stay,INPUT_PULLUP);
    pinMode(GoStop, INPUT_PULLUP);
    radio.begin();
  radio.openWritingPipe(address); //이전에 설정한 5글자 문자열인 데이터를 보낼 수신의 주소를 설정
  radio.setPALevel(RF24_PA_MIN); 
  radio.stopListening();
    Wire.begin();
    Serial.begin(115200);                        // 통신속도 38400 bps
    Wire.setClock(400000);
    Wire.beginTransmission(0x68);
    Wire.write(0x6b);
    Wire.write(0x0);
    Wire.endTransmission(true);

    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
    delay(1000);
    Serial.println("     ");
    //  Mxyz_init_calibrated ();
    
}


void loop()
{
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
   
   Serial.print("AngleX(PITCH) = "); Serial.print(AngleX);
   Serial.print(", ");
   Serial.print("AngleY(ROLL) = "); Serial.print(AngleY);
   Serial.print(", ");
   Serial.print("AngleZ(YAW) = "); Serial.println(AngleZ);
   if(digitalRead(Stay) == HIGH) {
    AngleX = 0;
    AngleY = 0;
    AngleZ = 0;
   }

    getAccel_Data();
    getGyro_Data();
    getCompassDate_calibrated(); // compass data has been calibrated here
    getHeading();               //before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .
    getTiltHeading();

    
    /*Serial.println("Compass Value of X,Y,Z:");
    Serial.print(Mxyz[0]);
    Serial.print(",");
    Serial.print(Mxyz[1]);
    Serial.print(",");
    Serial.print("Compass(Z) = ");
    Serial.println(Mxyz[2]);
    Serial.println();*/

    /*if ((AngleX - PITCH <= 10.00 && AngleX - PITCH >= -10.00)
    && (AngleY - PITCH <= 10.00 && AngleY - PITCH >= -10.00)
    && (AngleZ - YAW <= 20.00 && AngleZ - YAW >= -20.00)) {
      Serial.println("PITCH : Stay");
      Serial.println("ROLL : Stay");
      Serial.println("YAW : Stay ");
    }
    else if (AngleX - PITCH >= -30.00 && AngleX - PITCH <= -10.00) {
      Serial.println("Go Straight - Speed 1");
    }
    else if (AngleX - PITCH >= -50.00 && AngleX - PITCH <= -30.00) {
      Serial.println("Go Straight - Speed 2");
    }
    else if (AngleX - PITCH >= -70.00 && AngleX - PITCH <= -50.00) {
      Serial.println("Go Straight - Speed 3");
    }
    else if (AngleX - PITCH <= 30.00 && AngleX - PITCH >= 10.00) {
      Serial.println("Go Back - Speed 1");
    }
    else if (AngleX - PITCH <= 50.00 && AngleX - PITCH >= 30.00) {
      Serial.println("Go Back - Speed 2");
    }
    else if (AngleX - PITCH <= 70.00 && AngleX - PITCH >= 50.00) {
      Serial.println("Go Back - Speed 3");
    } // 드론 앞 뒤 움직임 속도 제어

    else if (AngleY - ROLL >= -30.00 && AngleY - ROLL <= -10.00) {
      Serial.println("Go Left - Speed 1");
    }
    else if (AngleY - ROLL >= -50.00 && AngleY - ROLL <= -30.00) {
      Serial.println("Go Left - Speed 2");
    }
    else if (AngleY - ROLL >= -70.00 && AngleY - ROLL <= -50.00) {
      Serial.println("Go Left - Speed 3");
    }
    else if (AngleY - ROLL <= 30.00 && AngleY - ROLL >= 10.00) {
      Serial.println("Go Right - Speed 1");
    }
    else if (AngleY - ROLL <= 50.00 && AngleY - ROLL >= 30.00) {
      Serial.println("Go Right - Speed 2");
    }
    else if (AngleY - ROLL <= 70.00 && AngleY - ROLL >= 50.00) {
      Serial.println("Go Right - Speed 3");
    } // 드론 좌 우 움직임 속도 제어
    
    else if (AngleZ - YAW >= -30.00 && AngleZ - YAW <= -10.00) {
      Serial.println("Turn Right - Speed 1");
    }
    else if (AngleZ - YAW >= -50.00 && AngleZ - YAW <= -30.00) {
      Serial.println("Turn Right - Speed 2");
    }
    else if (AngleZ - YAW >= -70.00 && AngleZ - YAW <= -50.00) {
      Serial.println("Turn Right - Speed 3");
    }
    else if (AngleZ - YAW <= 30.00 && AngleZ - YAW >= 10.00) {
      Serial.println("Turn Left - Speed 1");
    }
    else if (AngleZ - YAW <= 50.00 && AngleZ - YAW >= 30.00) {
      Serial.println("Turn Left - Speed 2");
    }
    else if (AngleZ - YAW <= 70.00 && AngleZ - YAW >= 50.00) {
      Serial.println("Turn Left - Speed 3");
    } // 드론 좌 우 수평 회전 속도 제어 */



    if (digitalRead(GoStop) == LOW) {
          count++;
          delay(500);
        }   
    if(count % 2 != 0) {
        if(digitalRead(Stay) == LOW) {
          Serial.println("abc");
         joystick[0] = AngleX;
         joystick[1] = AngleY;
         joystick[2] = AngleZ;
         joystick[3] = Mxyz[2];
         joystick[4] = 0;
         joystick[5] = count%2;
         joystick[6] = analogRead(A0);
         radio.write(joystick,sizeof(joystick));
       }
        else if (digitalRead(Stay) == HIGH) {
          Serial.println("def");
          joystick[0] = AngleX;
          joystick[1] = AngleY;
          joystick[2] = AngleZ;
          joystick[3] = Mxyz[2];
          joystick[4] = 1;
          joystick[5] = count%2;
          joystick[6] = analogRead(A0);
         radio.write(joystick,sizeof(joystick));
        }
    }
    else if(digitalRead(GoStop) == HIGH) {
      Serial.println("h");
      joystick[5] = count%2;
      radio.write(joystick,sizeof(joystick));
    }
      
      
delay(100);
}

void getHeading(void)
{
    heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
    if (heading < 0) heading += 360;
}
void getTiltHeading(void)
{
    float pitch = asin(-Axyz[0]);
    float roll = asin(Axyz[1] / cos(pitch));

    float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
    float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
    float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
    tiltheading = 180 * atan2(yh, xh) / PI;
    if (yh < 0)    tiltheading += 360;
}
void Mxyz_init_calibrated ()
{

    Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
    Serial.print("  ");
    Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
    Serial.print("  ");
    Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
    while (!Serial.find("ready"));
    Serial.println("  ");
    Serial.println("ready");
    Serial.println("Sample starting......");
    Serial.println("waiting ......");

    get_calibration_Data ();

    Serial.println("     ");
    Serial.println("compass calibration parameter ");
    Serial.print(mx_centre);
    Serial.print("     ");
    Serial.print(my_centre);
    Serial.print("     ");
    Serial.println(mz_centre);
    Serial.println("    ");
}
void get_calibration_Data ()
{
    for (int i = 0; i < sample_num_mdate; i++)
    {
        get_one_sample_date_mxyz();

        if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
        if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
        if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];

        if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
        if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
        if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];
    }
    mx_max = mx_sample[1];
    my_max = my_sample[1];
    mz_max = mz_sample[1];

    mx_min = mx_sample[0];
    my_min = my_sample[0];
    mz_min = mz_sample[0];
    
    mx_centre = (mx_max + mx_min) / 2;
    my_centre = (my_max + my_min) / 2;
    mz_centre = (mz_max + mz_min) / 2;
}
void get_one_sample_date_mxyz()
{
    getCompass_Data();
    mx_sample[2] = Mxyz[0];
    my_sample[2] = Mxyz[1];
    mz_sample[2] = Mxyz[2];
}
void getAccel_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = (double) ax / 16384;
    Axyz[1] = (double) ay / 16384;
    Axyz[2] = (double) az / 16384;
}
void getGyro_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    Gxyz[0] = (double) gx * 250 / 32768;
    Gxyz[1] = (double) gy * 250 / 32768;
    Gxyz[2] = (double) gz * 250 / 32768;
}
void getCompass_Data(void)
{
    I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
    delay(10);
    I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
    my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
    mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;

    Mxyz[0] = (double) mx * 1200 / 4096;
    Mxyz[1] = (double) my * 1200 / 4096;
    Mxyz[2] = (double) mz * 1200 / 4096;
}
void getCompassDate_calibrated ()
{
    getCompass_Data();
    Mxyz[0] = Mxyz[0] - mx_centre;
    Mxyz[1] = Mxyz[1] - my_centre;
    Mxyz[2] = Mxyz[2] - mz_centre;
}
