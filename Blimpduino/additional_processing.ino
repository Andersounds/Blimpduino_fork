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

 float MPU6050_pitchAngle(void)
{
  //Calculate pitch and roll according to
  // https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
  // Note that no UAV to IMU mat is defined separately, i just use other axles. ie uav(y) = -imu(x), uav(x)=imu(y), uav(z)=imu(z)

  float Gpx = (float)accel_t_gyro.value.y_accel;
  float Gpy = (float)-accel_t_gyro.value.x_accel;
  float Gpz = (float)accel_t_gyro.value.z_accel;
  float num_ = -Gpx;
  float den_ = sqrt(Gpy*Gpy + Gpz*Gpz);
  float accel_pitch_angle = atan(num_/den_);
  
  return accel_pitch_angle;
  //float accel_roll_angle = atan2f((float)accel_t_gyro.value.y_accel, (float)accel_t_gyro.value.z_accel)
  // LP of acc
  // Integrate + HP of gyro
  // Fusing?
}

float MPU6050_rollAngle(void)
{
  //float accel_roll_angle = atan2f((float)-accel_t_gyro.value.x_accel, (float)accel_t_gyro.value.z_accel);
   float accel_roll_angle = atan((float)-accel_t_gyro.value.x_accel/(float)accel_t_gyro.value.z_accel);
  return accel_roll_angle;
  //z_gyro_value = (accel_t_gyro.value.z_gyro - z_gyro_offset) / 65.5;  // Accel scale at 500deg/seg  => 65.5 LSB/deg/s
  //yawAngle = yawAngle + z_gyro_value * dt;
  //return yawAngle;
}

//These do not consided uav to imu transofmration. just choose the axle which correspond to the physical direction
float MPU6050_pitchRate(void){
  return (float)-accel_t_gyro.value.x_gyro*500.0/32767.0;
}

float MPU6050_rollRate(void){
  return (float)accel_t_gyro.value.y_gyro*500.0/32767.0;
}



//This function is just taken from MPU6050 file. Adapt and make it calibrate in around x and y axis. save bias somewhere it can be used
// Calibrate function. Take 100 readings (over 2 seconds) to calculate the gyro offset value. IMU should be steady in this process...
void MPU6050_calibrate_y_x()
{
  int i;
  long value = 0;
  float dev;
  int16_t values[100];
  bool gyro_cal_ok = false;

  delay(500);
  while (!gyro_cal_ok) {
    SerialUSB.println("Gyro calibration... DONT MOVE!");
    // we take 100 measurements in 4 seconds
    for (i = 0; i < 100; i++)
    {
      MPU6050_read_3axis();
      values[i] = accel_t_gyro.value.z_gyro;  //x_gyro
      value += accel_t_gyro.value.z_gyro;   //x_gyro
      delay(25);
    }
    // mean value
    value = value / 100;
    // calculate the standard deviation
    dev = 0;
    for (i = 0; i < 100; i++)
      dev += (values[i] - value) * (values[i] - value);
    dev = sqrt((1 / 100.0) * dev);
    SerialUSB.print("offset: ");
    SerialUSB.print(value);
    SerialUSB.print("  stddev: ");
    SerialUSB.println(dev);
    if (dev < 100.0)
      gyro_cal_ok = true;
    else
      SerialUSB.println("Repeat, DONT MOVE!");
  }
  //x_gyro_offset = value;
  z_gyro_offset = value;

  yawAngle = 0;
  // Take the first reading of angle from accels
  //angle = atan2f((float)accel_t_gyro.value.y_accel, (float)accel_t_gyro.value.z_accel) * RAD2GRAD;
}


