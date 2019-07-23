// BLIMPDUINO proyect. JJROBOTS.
// SMARTPHONE CONTROLLED BLIMP
//
// This code is prepared for the Blimpduino electronics
// Author: JJROBOTS.COM & mRobotics.io
// Date: 09/2017
// Updated: 10/02/2018
// Version: 1.02
// License: GPL v2
// Compiled and tested with Arduino 1.8.5.
// Project URL:

#include <Wire.h>
#include <SparkFun_VL53L1X_Arduino_Library.h>
#include <vl53l1_register_map.h>

#define TELEMETRY "192.168.4.2" // Default telemetry server (first client) port 2223

// NORMAL MODE PARAMETERS (MAXIMUN SETTINGS)
#define MAX_THROTTLE 240
#define MAX_STEERING 240

// Servo definitions
#define SERVO_AUX_NEUTRO 1500  // Servo neutral position
#define SERVO_MIN_PULSEWIDTH 700
#define SERVO_MAX_PULSEWIDTH 2500

#define SERVO2_NEUTRO 1500
#define SERVO2_RANGE 1400

#define BATTERY_FACTOR 16

// Telemetry
#define TELEMETRY_BATTERY 1
#define TELEMETRY_ANGLE 1
//#define TELEMETRY_DEBUG 1  // Dont use TELEMETRY_ANGLE and TELEMETRY_DEBUG at the same time!

#define DEBUG 0   // 0 = No debug info (default) DEBUG 1 for console output

#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

#define MSGMAXLEN 20  // Max message length. Message from the APP (parameters)
#define NODATA -20000

#define RED_LED A1
#define GREEN_LED A2

/*HEre you must define the physical port your motor is connected*/
#define mR 1 //Motor Right connected to port m1 
#define mL 0 //Motor Left connected to port m0 
#define mV 2 //Motor Vertical connected to port m2

/*Here you can reverse your motors*/
#define m0_rev 1
#define m1_rev 0
#define m2_rev 1
#define m3_rev 0

VL53L1X distanceSensor;

String MAC;  // MAC address of Wifi module

uint8_t loop_counter;       // To generate a medium loop 40Hz
uint8_t slow_loop_counter;  // slow loop 2Hz
uint8_t sendBattery_counter; // To send battery status
int16_t BatteryValue;

long timer_value;
float debugVariable;
float dt;

long timer_termPrint = 0; //Timer used for the Terminal output data. by Jordi
long timer_newData = 0; //Timer used to know when new data arrived.

float target_angle;
int16_t throttle;
float steering;
float max_throttle = MAX_THROTTLE;
float max_steering = MAX_STEERING;
float control_output;

//uint8_t mode;  // mode = 0 Manual mode, mode = 1 Aided mode

int modeSelector;

float control = 0;
float error = 0;
float error_old = 0;
float error_h = 0;
float error_h_old = 0;
float control_h = 0;
float kp_h = 1.0;
float kd_h = 1.0;
float ki_h = 1.0;

//VL53L0X sensor;
int laser_height;
int height;
int height_old;
float height_dt;
uint8_t height_sensor = 0;
unsigned long timer_laser = 0; //Timer for the LIDAR/Laser/Altimeter
unsigned long timer_laser_old = 0; //Last timer interaction.
float target_height = 1200;
int laser_newDataReady = 0;


float MPU_yaw_angle;
long timer_mpu = 0;
long timer_mpu_old = 0;
int mpu_newDataReady = 0;
float MPU_dt = 0;


// Message variables
int16_t iCH1;
int16_t iCH2;
int16_t iCH3;
int16_t iCH4;
int16_t iCH5;
int16_t iCH6;
int16_t iCH7;
int16_t iCH8;
int newMessage = 0;
uint8_t MsgBuffer[MSGMAXLEN + 1];

// output motor variables
int mRight_Value = 0;
int mLeft_Value = 0;
int mVertical_Value = 0;

// Global variable used by roll/pitch sensing. T matrix is rotation matrix from UAV to IMU
int16_t T1_imu[3] = {0,1,0}; //Row 1 of T matrix
int16_t T2_imu[3] = {-1,0,0}; //Row 2 of T matrix
int16_t T3_imu[3] = {0,0,1}; //Row 3 of T matrix

//Gyro offsets for
int16_t gyro_offsets_x_y[2] = {0,0};



// INITIALIZATION
void setup()
{
  pinMode(13, OUTPUT); //Not connected in version 1.0.

  // LEDS
  pinMode(RED_LED, OUTPUT); // RED LED
  pinMode(GREEN_LED, OUTPUT); // GREEN LED

  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);

  delay(2000);
  SerialUSB.begin(115200); // Serial output to console
  Serial1.begin(115200);
  SerialUSB.println("Inicializando");

  // Initialize I2C bus (MPU6050 is connected via I2C)
  Wire.begin();

#if DEBUG > 0
  delay(9000);
#else
  delay(2000);
#endif

  SerialUSB.println("JJROBOTS & mRo");
  delay(200);
  SerialUSB.println("Don't move for 10 sec...");
  MPU6050_setup();  // setup MPU6050 IMU at 50Hz
  delay(500);
  MPU6050_calibrate_x_y(gyro_offsets_x_y);//Own calbration code.

  // With the new ESP8266 WIFI MODULE WE NEED TO MAKE AN INITIALIZATION PROCESS
  SerialUSB.println("WIFI init");
  Serial1.flush();
  Serial1.print("+++");  // To ensure we exit the transparent transmision mode
  delay(100);
  ESPsendCommand("AT", "OK", 1);
  ESPsendCommand("AT+RST", "OK", 2); // ESP Wifi module RESET
  ESPwait("ready", 6);
  ESPsendCommand("AT+GMR", "OK", 5);

#ifdef EXTERNAL_WIFI
  ESPsendCommand("AT+CWQAP", "OK", 3);
  ESPsendCommand("AT+CWMODE=1", "OK", 3);
  //String auxCommand = (String)"AT+CWJAP="+WIFI_SSID+","+WIFI_PASSWORD;
  char auxCommand[90] = "AT+CWJAP=\"";
  strcat(auxCommand, WIFI_SSID);
  strcat(auxCommand, "\",\"");
  strcat(auxCommand, WIFI_PASSWORD);
  strcat(auxCommand, "\"");
  ESPsendCommand(auxCommand, "OK", 14);
#ifdef WIFI_IP
  strcpy(auxCommand, "AT+CIPSTA=\"");
  strcat(auxCommand, WIFI_IP);
  strcat(auxCommand, "\"");
  ESPsendCommand(auxCommand, "OK", 4);
#endif
  ESPsendCommand("AT+CIPSTA?", "OK", 4);
#else  // Deafault : we generate a wifi network
  Serial1.println("AT+CIPSTAMAC?");
  ESPgetMac();
  SerialUSB.print("MAC:");
  SerialUSB.println(MAC);
  delay(200);
  //ESPsendCommand("AT+CWQAP", "OK", 3);
  //SerialUSB.println("Aqui ok");
  ESPsendCommand("AT+CWMODE=2", "OK", 3); // Soft AP mode
  //SerialUSB.println("Aqui tambien");
  // Generate Soft AP. SSID=JJROBOTS, PASS=87654321
  char *cmd =  "AT+CWSAP=\"JJROBOTS_XX\",\"87654321\",5,3";
  // Update XX characters with MAC address (last 2 characters)
  //cmd[19] = MAC[10];
  //cmd[20] = MAC[11];
  ESPsendCommand(cmd, "OK", 6);
#endif
  // Start UDP SERVER on port 2222, telemetry port 2223
  SerialUSB.println("Start UDP server");
  ESPsendCommand("AT+CIPMUX=0", "OK", 3);  // Single connection mode
  ESPsendCommand("AT+CIPMODE=1", "OK", 3); // Transparent mode
  char Telemetry[80];
  strcpy(Telemetry, "AT+CIPSTART=\"UDP\",\"");
  strcat(Telemetry, TELEMETRY);
  //strcat(Telemetry, "\",2222,2222,0");
  strcat(Telemetry, "\",2223,2222,0");
  ESPsendCommand(Telemetry, "OK", 3);

  // Calibrate gyros
  MPU6050_calibrate();

  ESPsendCommand("AT+CIPSEND", ">", 2); // Start transmission (transparent mode)

  if (distanceSensor.begin() == false)
    SerialUSB.println("Holly Fffff, LIDAR is offline!");
  else
    SerialUSB.println("LIDAR OK!");

  BatteryValue = analogRead(A0) / BATTERY_FACTOR;
#if TELEMETRY_BATTERY==1
  SerialUSB.print("BATT:");
  SerialUSB.println(BatteryValue);
#endif


  // Take first laser reading
  distanceSensor.startMeasurement(); //Jose Julio, agregale TIMEOUT para que no se cuelgue. Plis
  while (distanceSensor.newDataReady() == false)
    delay(5);
  laser_height = distanceSensor.getDistance();
  distanceSensor.startMeasurement();
  height = laser_height;
  laser_newDataReady = 1;
  timer_laser = millis();
  SerialUSB.print("Laser Alt:");
  SerialUSB.println(laser_height);

  SerialUSB.println("Blimpduino by JJROBOTS & mRo v0.10");
  SerialUSB.println("Start...");
  
  timer_laser_old = micros();
  timer_mpu_old = micros();

  target_angle = 0.0;
  target_height = 1000;

  error = 0;
  //digitalWrite(A2, LOW);
  //sensor.readRangeSingleMillimetersStart();

  initilize_motors();

  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
}

// MAIN LOOP
void loop()
{

  TTLcmd(); //Commands used for system test

  MsgRead();  // Read UDP messages

  Dameon_Loop(); //Background services...

  USB_Print_Loop(500); //Send telemetry every 500ms....

  /**************************************************/

  /*Mode selector is better known as System State Selector.
    Which is basically a series of "if" conditions main-
    ly controlled by you -via the App- and external inputs -if any-.
    This will output a positive value from 0 to x, this value
    controls "System Mode" section*/

  /*------->Mode Selector Begins<-------*/
  modeSelector = 0;


  if ((iCH5 & 1) == 1) {
    modeSelector = 1;
  }

  if ((iCH5 & 2) == 2){
    modeSelector+=2;
  }

  if ((modeSelector != 2) && (modeSelector != 3)) { //Resets the angle so it avoids overshooting and knows what angle to keep when yawStabilize is selected.
    target_angle = MPU_yaw_angle;
  }

  if ((modeSelector != 1) && (modeSelector != 3)) { //Selects the current altitude to hold when switching to Alt Hold. 
    target_height = height;
  }
  
   
  if (millis() - timer_newData > 2000) {   //<--- Move this to state 100 later.
    modeSelector = 100; //NO UDP DATA MODE
  }

  /*------->Mode Selector Ends<-------*/

  /*System mode is better know as the "system state" and is controlled by modeSelector (above). modeSelector is mainly controlled by you and external sources.*/
  
  /*------->System Mode begins<-------*/
  //static unsigned long timer_sysMode = 0;

  // if ((millis() - timer_sysMode) > 5) {
  //timer_sysMode = millis();
  switch (modeSelector) {

    case 0: //Mode 0: Full Manual control.
      manualControl();
      altitudeManual(4);

      m_set_direct(mR, mLeft_Value);
      m_set_direct(mV, mVertical_Value);
      m_set_direct(mL, mRight_Value);
      break;

    case 1: //Manual Control with Altitude hold
      manualControl();
      altitudeHold();

      m_set_direct(mR, mLeft_Value);
      m_set_direct(mV, mVertical_Value);
      m_set_direct(mL, mRight_Value);
      break;

    case 2: //Yaw stabiliztion (Inverse Kinematic)
      yawStabilized();
      altitudeManual(4);

      m_set(mR, mLeft_Value, 4); //DeadZone set to 4. 
      m_set(mV, mVertical_Value, 4);
      m_set(mL, mRight_Value, 4);
      break;

    case 3: //Yaw stabilization with Altitude hold (Inverse Kinematic)
      yawStabilized();
      altitudeHold();

      m_set(mR, mLeft_Value, 4); //DeadZone set to 4. 
      m_set(mV, mVertical_Value, 4);
      m_set(mL, mRight_Value, 4);
      break;

    case 100:{ //No UDP data received.
        m_stopAll();//<----Add
        float accPitchRoll[2];
        float gyroPitchRoll[2];

        MPU6050_Acc_Pitch_Roll_Angle(accPitchRoll);
        MPU6050_Gyro_Pitch_Roll_Rate(gyroPitchRoll);

    //  float pitchangl = MPU6050_pitchAngle();
    //  float rollangl = MPU6050_rollAngle();
     // printAccel();

     //float pRate_ = MPU6050_pitchRate();
     //float rRate_ = MPU6050_rollRate();
//SerialUSB.print("Pitch: ");SerialUSB.print(pitchangl);SerialUSB.print(", Roll: ");SerialUSB.print(rollangl);SerialUSB.print(" rad\n");
SerialUSB.print("Pitch: ");SerialUSB.print(accPitchRoll[0]);SerialUSB.print(", Roll: ");SerialUSB.print(accPitchRoll[1]);SerialUSB.print(" rad\n");
SerialUSB.print("Pitchrate: ");SerialUSB.print(gyroPitchRoll[0]);SerialUSB.print(", Rollrate: ");SerialUSB.print(gyroPitchRoll[1]);SerialUSB.print(" deg/s \n");
      delay(60);
      
 /*     
      // Start of custom code
      float messageRPI[3] = {1,2,(float)laser_height/1000};
      int stat = writeToRpi(messageRPI,3);
      switch(stat){
        case 0:{SerialUSB.println("Success wrote to pi!");break;}
        case 1:{SerialUSB.println("Data too long for tx buffer");break;}
        case 2:{SerialUSB.println("Recieved NACK on transmit of address");break;}
        case 3:{SerialUSB.println("Recieved NACK on transmit of data");break;}
        case 4:{SerialUSB.println("Some error in i2c transmit");break;}
        }

      delay(2000);



      //Clear wire.
      while(Wire.available()){
        Wire.read();
      }
      float recv_values[3];
      int recieved_floats = readFromRpi(recv_values);
      switch(recieved_floats){
          case -3:{SerialUSB.println("No ID byte.");break;}
          case -2:{SerialUSB.println("Complete message did not fit in rx buffer. discarding");break;}
          case 0:{ SerialUSB.println("No bytes recieved from rpi. moving on...");break;}        
          default:{SerialUSB.print("Recieved ");SerialUSB.print(recieved_floats);SerialUSB.print(" floats.\n");
              for(int i=0;i< recieved_floats;i++){
                  SerialUSB.print(recv_values[i]);SerialUSB.print(", ");
              }
              SerialUSB.print("\n");
          }  
      }
      delay(2000);
*/
      //End of custom code
      break;
    }

    case 101: //Reserved for low battery
      m_stopAll();//<----Add Low Battery Routine HERE.
      break;

    default: //Unknown State.
      Serial1.print("Bunny in the pot");
      break;

  } /*------->System Mode ends<-------*/
  // }

}

