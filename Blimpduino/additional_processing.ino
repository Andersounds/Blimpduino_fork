// File for definition of additional processing functions
// -Fusing of pressure sensor + range sensor +  filtering for height estimation
// -Controller
/*
TODO
  -Filter of height measurement

*/

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
  int16_t Gpx = (accel_t_gyro.value.x_accel*T1_imu_inv[0] + accel_t_gyro.value.y_accel*T1_imu_inv[1] + accel_t_gyro.value.z_accel*T1_imu_inv[2]);
  int16_t Gpy = (accel_t_gyro.value.x_accel*T2_imu_inv[0] + accel_t_gyro.value.y_accel*T2_imu_inv[1] + accel_t_gyro.value.z_accel*T2_imu_inv[2]);
  int16_t Gpz = (accel_t_gyro.value.x_accel*T3_imu_inv[0] + accel_t_gyro.value.y_accel*T3_imu_inv[1] + accel_t_gyro.value.z_accel*T3_imu_inv[2]);
    SerialUSB.print(Gpx);
    SerialUSB.print("\t");
    SerialUSB.print(Gpy);
    SerialUSB.print("\t");
    SerialUSB.println(Gpz);
}
void printGyro(void){

  int16_t Gpx = (accel_t_gyro.value.x_gyro*T1_imu_inv[0] + accel_t_gyro.value.y_gyro*T1_imu_inv[1] + accel_t_gyro.value.z_gyro*T1_imu_inv[2]);
  int16_t Gpy = (accel_t_gyro.value.x_gyro*T2_imu_inv[0] + accel_t_gyro.value.y_gyro*T2_imu_inv[1] + accel_t_gyro.value.z_gyro*T2_imu_inv[2]);
  int16_t Gpz = (accel_t_gyro.value.x_gyro*T3_imu_inv[0] + accel_t_gyro.value.y_gyro*T3_imu_inv[1] + accel_t_gyro.value.z_gyro*T3_imu_inv[2]);
    SerialUSB.print(Gpx);
    SerialUSB.print("\t");
    SerialUSB.print(Gpy);
    SerialUSB.print("\t");
    SerialUSB.println(Gpz);
}
void printUAVRollPitch(int refreshRate){
    static long timer = 0; //Setting timer
    if((millis() - timer) > refreshRate){
        SerialUSB.print(roll_rpi*RAD2GRAD);
        SerialUSB.print("\t");
        SerialUSB.println(pitch_rpi*RAD2GRAD);
        timer = millis();//Reset timer
    }
}
/* This function takes IMU acceleration values, transforms them from IMU to UAV frame according to the global T-matrix (T1_imu_inv, T2_imu_inv,T3_imu_inv)
 * It then uses the acceleration values to calculate the pitch and roll angle of the UAV within the range +-pi/2. Angles may not superceed these limit
 *  NOTE must call  MPU6050_read_3axis(); outside to get as fresh as possible data
 */
int MPU6050_Acc_Pitch_Roll_Angle(float* pitchRoll)
{
  //Calculate pitch and roll according to
  // https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing

  // Calculate sensor values in UAV frame from IMU frame via T matrix (Defined as global)
  int16_t Gpx = (accel_t_gyro.value.x_accel*T1_imu_inv[0] + accel_t_gyro.value.y_accel*T1_imu_inv[1] + accel_t_gyro.value.z_accel*T1_imu_inv[2]);
  int16_t Gpy = (accel_t_gyro.value.x_accel*T2_imu_inv[0] + accel_t_gyro.value.y_accel*T2_imu_inv[1] + accel_t_gyro.value.z_accel*T2_imu_inv[2]);
  int16_t Gpz = (accel_t_gyro.value.x_accel*T3_imu_inv[0] + accel_t_gyro.value.y_accel*T3_imu_inv[1] + accel_t_gyro.value.z_accel*T3_imu_inv[2]);
  
  float num_P = (float)-Gpx;
  float den_P = sqrt((float)(Gpy*Gpy + Gpz*Gpz));
  float num_R = (float)Gpy;
  float den_R = (float)Gpz;
  pitchRoll[0] = -atan(num_P/den_P); //Pitch angle /* ======= SIGN IS FLIPPED HERE DUE TO Z AXIS POINTING DOWN =======   */
  pitchRoll[1] = atan(num_R/den_R); //Roll angle
  //pitchRoll[0] = atan2(num_P,den_P); //Pitch angle
  //pitchRoll[1] = atan2(num_R,den_R); //Roll angle
  
  return 1;
}

// Takes the latest gyro measurements and translates them via T inv matrix to UAV frame
int MPU6050_Gyro_Pitch_Roll_Rate(float* pitchRollRate){
  //static float gyroToRadsPSec = RAD2GRAD*500.0/32767.0;//500 is one sided range as set in initialization. 32767 is bit range.
  static float gyroToRadsPSec = GRAD2RAD*500.0/32767.0;//500 is one sided range as set in initialization. 32767 is bit range.
  // Calculate sensor values in UAV frame from IMU frame via T matrix (Defined as global)
  int16_t Grx = ((accel_t_gyro.value.x_gyro-x_gyro_offset)*T1_imu_inv[0] + (accel_t_gyro.value.y_gyro-y_gyro_offset)*T1_imu_inv[1] + (accel_t_gyro.value.z_gyro-z_gyro_offset)*T1_imu_inv[2]);
  int16_t Gry = ((accel_t_gyro.value.x_gyro-x_gyro_offset)*T2_imu_inv[0] + (accel_t_gyro.value.y_gyro-y_gyro_offset)*T2_imu_inv[1] + (accel_t_gyro.value.z_gyro-z_gyro_offset)*T2_imu_inv[2]);
  //int16_t Grz = (accel_t_gyro.value.x_gyro*T3_imu_inv[0] + accel_t_gyro.value.y_gyro*T3_imu_inv[1] + accel_t_gyro.value.z_accel*T3_imu_inv[2]);
  
  pitchRollRate[0] = ((float)Gry)*gyroToRadsPSec;//Pitch rate expressed in rads/sec. 
  pitchRollRate[1] = ((float)Grx)*gyroToRadsPSec;//Roll rate expressed in rads/sec

  return 1;
}
//Just LP filter for gyro. To be used before integration
float LPfilterGyroRoll(float dt, float rate){
//Init cutoff frequency and some static variables
    static float wc = 30.0;                           //Cutoff frequency (May be changed) We are setting a pretty  high frequency here
    static float rate_prev = 0;
    static float rate_filt_prev = 0;
    //Apply LP filter
    float rate_filt = ( (rate + rate_prev)*dt*wc + rate_filt_prev*(2-dt*wc) ) / (2+dt*wc);
    //Shift values
    rate_prev = rate;
    rate_filt_prev = rate_filt;
    return rate_filt;
}
//Another instance of function above, for pitch angle
float LPfilterGyroPitch(float dt, float rate){
//Init cutoff frequency and some static variables
    static float wc = 30.0;                           //Cutoff frequency (May be changed)
    static float rate_prev = 0;
    static float rate_filt_prev = 0;
    //Apply LP filter
    float rate_filt = ( (rate + rate_prev)*dt*wc + rate_filt_prev*(2-dt*wc) ) / (2+dt*wc);
    //Shift values
    rate_prev = rate;
    rate_filt_prev = rate_filt;
    return rate_filt;
}


float complFilterPitch(float dt, float acc_angle, float gyro_rate_raw){
    //Init cutoff frequency and some static variables
    static float wc = 10.0;                           //Cutoff frequency (May be changed)
    static float acc_angle_prev = 0;                  // Angle from accelerometer (previous);
    static float acc_angle_filt_prev = 0;             // Filtered angle from accelerometer (previous);
    static float gyro_rate_prev = 0;                  // Angle RATE from gyro (previous)
    static float gyro_angle_prev = 0;                 // Angle from gyro (integrated) (previous)
    static float gyro_angle_filt_prev = 0;            // Filtered angle from gyro (previous)  
    // LP filt of accelerometer angle (1st order LP discretized using tustin)
    float acc_angle_filt = ( (acc_angle + acc_angle_prev)*dt*wc + acc_angle_filt_prev*(2-dt*wc) ) / (2+dt*wc);
    //Lowpass of raw gyro rate value
    float gyro_rate = LPfilterGyroPitch(dt,gyro_rate_raw);
    //Integrate gyro rate to angle using tustin estimation
    float gyro_angle = gyro_angle_prev + (gyro_rate+gyro_rate_prev)*dt/2;
    // HP filt of integrated gyro angle (1st order HP dicretized using tustin)
    float gyro_angle_filt = ( (gyro_angle - gyro_angle_prev)*2 + gyro_angle_filt_prev*(2 - dt*wc) )/(2 + dt*wc);
    // Time shift static variables
    acc_angle_prev = acc_angle;
    acc_angle_filt_prev = acc_angle_filt;
    gyro_rate_prev = gyro_rate;
    gyro_angle_prev = gyro_angle;
    gyro_angle_filt_prev = gyro_angle_filt;
    // Combine filtered estimations
    return (gyro_angle_filt + acc_angle_filt);
}

float complFilterRoll(float dt, float acc_angle, float gyro_rate_raw){
    //Init cutoff frequency and some static variables
    static float wc = 10.0;
    static float acc_angle_prev = 0;                  // Angle from accelerometer (previous);
    static float acc_angle_filt_prev = 0;             // Filtered angle from accelerometer (previous);
    static float gyro_rate_prev = 0;                  // Angle RATE from gyro (previous)
    static float gyro_angle_prev = 0;                 // Angle from gyro (integrated) (previous)
    static float gyro_angle_filt_prev = 0;            // Filtered angle from gyro (previous)  
    // LP filt of accelerometer angle (1st order LP discretized using tustin)
    float acc_angle_filt = ( (acc_angle + acc_angle_prev)*dt*wc + acc_angle_filt_prev*(2-dt*wc) ) / (2+dt*wc);
    //Lowpass of raw gyro rate value
    float gyro_rate = LPfilterGyroRoll(dt,gyro_rate_raw);
    //Integrate gyro rate to angle using tustin estimation
    float gyro_angle = gyro_angle_prev + (gyro_rate+gyro_rate_prev)*dt/2;
    // HP filt of integrated gyro angle (1st order HP dicretized using tustin)
    float gyro_angle_filt = ( (gyro_angle - gyro_angle_prev)*2 + gyro_angle_filt_prev*(2 - dt*wc) )/(2 + dt*wc);
    // Time shift static variables
    acc_angle_prev = acc_angle;
    acc_angle_filt_prev = acc_angle_filt;
    gyro_rate_prev = gyro_rate;
    gyro_angle_prev = gyro_angle;
    gyro_angle_filt_prev = gyro_angle_filt;
    // Combine filtered estimations
    return (gyro_angle_filt + acc_angle_filt);
}
//Filter of height sensor value. 
// 1. Use roll and pitch to get vertical distance to horizontal plane (To prevent vertical oscillations due to tilting oscillation)
// 2. Differentiate to get vertical velocity
// 3. Low pass filter to remove any spikes due to for exapmple flying over a table as well as noise
// 4. Integrate to get delta-height
// 5. Add to previous height
// Previous height is either the previous estimation or a new value from azipe.
// The filtering is only done on delta-height, so should not be affected by changing the offset.
float delta_height(float dt, float dist,float roll, float pitch){
    static float wc = 10;//Cutoff frequency of LP filter
    static float height_prev = 0;
    static float height_prim_prev = 0;
    static float height_prim_filt_prev = 0; //Should this have another start value? maybe start out as height_prim_filt?
    float height = dist*cos(roll)*cos(pitch); //Calculate vertical height
    float height_prim = (height-height_prev)/dt;    //Differentiate to get vertical velocity
    if(abs(height_prim) > 1000){
      height_prim = height_prim_prev;
    }
    // LP filt of vertical velocity (1st order LP discretized using tustin)
    float height_prim_filt = ( (height_prim + height_prim_prev)*dt*wc + height_prim_filt_prev*(2-dt*wc) ) / (2+dt*wc);
    // Integrate using tustin estimation
    float delta_height_filt = (height_prim_filt + height_prim_filt_prev)*dt/2;
    //Shift values
    height_prev = height;
    height_prim_prev = height_prim;
    height_prim_filt_prev = height_prim_filt;
    //return delta_height_filt;
    return height_prim_filt;
   

 }

//Differentiate twice
//Constrain vertical acceleration
//integrate
//LP
//Integrate
float delta_height_2(float dt, float dist,float roll, float pitch){
    static float wc1 = 10;//Cutoff frequency of LP filter
    static float wc2 = 50;//Cutoff frequency of LP filter
    static float height_prev = 0;
    static float height_filt_prev = 0;
    static float height_filt_prim_prev = 0;
    static float height_filt_prim_filt_prev = 0;
    float height = dist*cos(roll)*cos(pitch); //Calculate vertical height
    //Filter height data
    float height_filt = ( (height + height_prev)*dt*wc1 + height_filt_prev*(2-dt*wc1) ) / (2+dt*wc1);
    //Differentiate filtered height
    float height_filt_prim = (height_filt-height_filt_prev)/dt;
    //Filter velocity data
    float height_filt_prim_filt = ( (height_filt_prim + height_filt_prim_prev)*dt*wc2 + height_filt_prim_filt_prev*(2-dt*wc2) ) / (2+dt*wc2);
    // limit second derivative
    float primprim = (abs(height_filt_prim_filt - height_filt_prim_filt_prev)/dt);
    if( primprim> 2000 ){
        height_filt_prim_filt = 0;//height_filt_prim_filt_prev*0.1;
    }
    float delta_height_filt = (height_filt_prim_filt + height_filt_prim_filt_prev)*dt/2;

    height_prev = height;
    height_filt_prev = height_filt;
    height_filt_prim_prev = height_filt_prim;
    height_filt_prim_filt_prev = height_filt_prim_filt;

    return delta_height_filt;
   // return primprim;

}
//Gyro calibration is done in the main calibration function. Added functionality for calculationf offsets for x and y direction as well
