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
}



int MPU6050_Gyro_Pitch_Roll_Rate(float* pitchRollRate){
  static float gyroToRadsPSec = RAD2GRAD*500.0/32767.0;//500 is one sided range as set in initialization. 32767 is bit range.
  // Calculate sensor values in UAV frame from IMU frame via T matrix (Defined as global)
  int16_t Grx = ((accel_t_gyro.value.x_gyro-x_gyro_offset)*T1_imu[0] + (accel_t_gyro.value.y_gyro-y_gyro_offset)*T1_imu[1] + (accel_t_gyro.value.z_gyro-z_gyro_offset)*T1_imu[2]);
  int16_t Gry = ((accel_t_gyro.value.x_gyro-x_gyro_offset)*T2_imu[0] + (accel_t_gyro.value.y_gyro-y_gyro_offset)*T2_imu[1] + (accel_t_gyro.value.z_gyro-z_gyro_offset)*T2_imu[2]);
  //int16_t Grz = (accel_t_gyro.value.x_gyro*T3_imu[0] + accel_t_gyro.value.y_gyro*T3_imu[1] + accel_t_gyro.value.z_accel*T3_imu[2]);
  
  pitchRollRate[0] = ((float)Gry)*gyroToRadsPSec;//Pitch rate expressed in rads/sec. 
  pitchRollRate[1] = ((float)Grx)*gyroToRadsPSec;//Roll rate expressed in rads/sec

  return 1;
}

float complFilterPitch(float dt, float accValue, float gyroValue){
    static float wc = 64.0;
    static float snorkAcc = 0.99;
    static float snork2 = 1-snorkAcc;
    static float acc_prev = 0;//accValue;
    static float acc_filt_prev =0;// accValue;
    static float gyro_prev = 0;//gyroValue;
    static float gyro_filt_prev =0;// gyroValue;

    float acc_filt = ( (accValue + acc_prev)*dt*wc + acc_filt_prev*(2-dt*wc) ) / (2+dt*wc);
    float gyro_filt = ( (gyroValue + gyro_prev)*dt + gyro_filt_prev*(2-dt*wc) ) / (2+dt*wc);

    acc_prev = accValue;
    acc_filt_prev = acc_filt;
    gyro_prev = gyroValue;
    gyro_filt_prev = gyro_filt;

    //return gyro_filt;//snorkAcc*acc_filt + snork2*gyro_filt;
    return snorkAcc*acc_filt + snork2*gyro_filt;
}

float complFilterRoll(float dt, float accValue, float gyroValue){
    static float wc = 64.0;
    static float snorkAcc = 0.99;
    static float snork2 = 1-snorkAcc;
    static float acc_prev =accValue;
    static float acc_filt_prev = accValue;
    static float gyro_prev = gyroValue;
    static float gyro_filt_prev = gyroValue;

    float acc_filt = ( (accValue + acc_prev)*dt*wc + acc_filt_prev*(2-dt*wc) ) / (2+dt*wc);
    float gyro_filt = ( (gyroValue + gyro_prev)*dt + gyro_filt_prev*(2-dt*wc) ) / (2+dt*wc);

    acc_prev = accValue;
    acc_filt_prev = acc_filt;
    gyro_prev = gyroValue;
    gyro_filt_prev = gyro_filt;

    //return acc_filt;//snorkAcc*acc_filt + snork2*gyro_filt;
    return snorkAcc*acc_filt + snork2*gyro_filt;
}

//Gyro calibration is done in the main calibration function. Added functionality for calculationf offsets for x and y direction as well
