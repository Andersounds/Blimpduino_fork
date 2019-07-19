// File for definition of additional processing functions
// -Fusing of pressure sensor + range sensor +  filtering for height estimation
// -Controller
/* Global MPU6050 variables are located in the accel_t_gyro_union-struct (defined on row 433 in MPU6050)
 *  It is instantiated on line 466 to the object accel_t_gyro. We acces the relevant values by:
        int16_t   accel_t_gyro.value.x_accel
        int16_t   accel_t_gyro.value.y_accel
        int16_t   accel_t_gyro.value.z_accel
        int16_t   accel_t_gyro.value.x_gyro
        int16_t   accel_t_gyro.value.y_gyro
        int16_t   accel_t_gyro.value.z_gyro
 * These functions are called in General, on lines 20-23
  The function MPU6050_getAngle implements a cmplementary filter. This will be used for roll and pitch as well
 */


/*
We must know the sampling time. It must be constant right?

*/
void printAccel(void){
        SerialUSB.print("X: ");SerialUSB.print(accel_t_gyro.value.x_accel);SerialUSB.print("Y: ");SerialUSB.print(accel_t_gyro.value.y_accel);
        SerialUSB.print("Z: ");SerialUSB.print(accel_t_gyro.value.z_accel);SerialUSB.print(" rad\n");
}

/* This function takes IMU acceleration values, transforms them from IMU to UAV frame according to the global T-matrix (T1_imu, T2_imu,T3_imu)
 * It then uses the acceleration values to calculate the pitch and roll angle of the UAV within the range +-pi/2. Angles may not superceed these limit
 *  NOTE must call  MPU6050_read_3axis(); outside to get as fresh as possible data
 */
int MPU6050_Acc_Pitch_Roll_Angle(float* pitchRoll)
{
  //Calculate pitch and roll according to
  // https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing

  // Calculate sensor values in UAV frame from IMU frame via T matrix (Defined as global)
  int16_t Gpx = (accel_t_gyro.value.x_accel*T1_imu[0] + accel_t_gyro.value.y_accel*T1_imu[1] + accel_t_gyro.value.z_accel*T1_imu[2]);
  int16_t Gpy = (accel_t_gyro.value.x_accel*T2_imu[0] + accel_t_gyro.value.y_accel*T2_imu[1] + accel_t_gyro.value.z_accel*T2_imu[2]);
  int16_t Gpz = (accel_t_gyro.value.x_accel*T3_imu[0] + accel_t_gyro.value.y_accel*T3_imu[1] + accel_t_gyro.value.z_accel*T3_imu[2]);
  
  float num_P = (float)-Gpx;
  float den_P = sqrt((float)(Gpy*Gpy + Gpz*Gpz));
  float num_R = (float)Gpy;
  float den_R = (float)Gpz;
  pitchRoll[0] = atan(num_P/den_P); //Pitch angle
  pitchRoll[1] = atan(num_R/den_R); //Roll angle
  
  return 1;
  //float accel_roll_angle = atan2f((float)accel_t_gyro.value.y_accel, (float)accel_t_gyro.value.z_accel)
  // LP of acc
  // Integrate + HP of gyro
  // Fusing?
}


//These do not consided uav to imu transofmration. just choose the axle which correspond to the physical direction
int MPU6050_Gyro_Pitch_Roll_Rate(float* pitchRollRate){
  static float gyroToRadsPSec = RAD2GRAD*500.0/32767.0;//500 is one sided range as set in initialization. 32767 is bit range.
  // Calculate sensor values in UAV frame from IMU frame via T matrix (Defined as global)
  int16_t Grx = ((accel_t_gyro.value.x_gyro+GYRO_OFFSETS_X_Y[0])*T1_imu[0] + (accel_t_gyro.value.y_gyro+GYRO_OFFSETS_X_Y[1])*T1_imu[1] + accel_t_gyro.value.z_gyro*T1_imu[2]);
  int16_t Gry = ((accel_t_gyro.value.x_gyro+GYRO_OFFSETS_X_Y[0])*T2_imu[0] + (accel_t_gyro.value.y_gyro+GYRO_OFFSETS_X_Y[1])*T2_imu[1] + accel_t_gyro.value.z_gyro*T2_imu[2]);
  //int16_t Grz = (accel_t_gyro.value.x_gyro*T3_imu[0] + accel_t_gyro.value.y_gyro*T3_imu[1] + accel_t_gyro.value.z_accel*T3_imu[2]);
  
  pitchRollRate[0] = ((float)Gry)*gyroToRadsPSec;//Pitch rate expressed in rads/sec. 
  pitchRollRate[1] = ((float)Grx)*gyroToRadsPSec;//Roll rate expressed in rads/sec

  return 1;
}



//This function is just taken from MPU6050 file. Adapt and make it calibrate in around x and y axis. save bias somewhere it can be used
// Calibrate function. Take 100 readings (over 2 seconds) to calculate the gyro offset value. IMU should be steady in this process...
void MPU6050_calibrate_x_y(int16_t* offsets)
{
  int i;
  long value_x = 0;
  long value_y = 0;
  float dev_x = 0;
  float dev_y = 0;
  int16_t values_x[100];
  int16_t values_y[100];
  bool gyro_cal_ok = false;

  delay(500);
  while (!gyro_cal_ok) {
    SerialUSB.println("Gyro calibration x-y... DONT MOVE!");
    // we take 100 measurements in 4 seconds
    for (i = 0; i < 100; i++)
    {
      MPU6050_read_3axis();
      values_x[i] = accel_t_gyro.value.x_gyro;  //x_gyro
      value_x += accel_t_gyro.value.x_gyro;   //x_gyro
      values_y[i] = accel_t_gyro.value.y_gyro;  //x_gyro
      value_y += accel_t_gyro.value.y_gyro;   //x_gyro
      delay(25);
    }
    // mean value
    value_x = value_x / 100;
    value_y = value_y / 100;
    // calculate the standard deviation
    dev_x = 0;
    dev_y = 0;
    for (i = 0; i < 100; i++){
      dev_x += (values_x[i] - value_x) * (values_x[i] - value_x);
      dev_y += (values_y[i] - value_y) * (values_y[i] - value_y);
    }
    dev_x = sqrt((1 / 100.0) * dev_x);
    dev_y = sqrt((1 / 100.0) * dev_y);
    
    SerialUSB.print("offset x: ");
    SerialUSB.print(value_x);
    SerialUSB.print("  stddev x: ");
    SerialUSB.println(dev_x);
    if (dev < 100.0)
      gyro_cal_ok = true;
    else
      SerialUSB.println("Repeat, DONT MOVE!");
  }
  offsets[0] = (int16_t)value_x;
  offsets[1] = (int16_t)value_y;
}


