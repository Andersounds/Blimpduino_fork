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
 * These functions are called in General, on lines 20-25
  The function MPU6050_getAngle implements a cmplementary filter. This will be used for roll and pitch as well
 */

 float MPU6050_pitchAngle(float dt)
{
  // LP of acc
  // Integrate + HP of gyro
  // Fusing?
}

float MPU6050_rollAngle(float dt)
{
  z_gyro_value = (accel_t_gyro.value.z_gyro - z_gyro_offset) / 65.5;  // Accel scale at 500deg/seg  => 65.5 LSB/deg/s
  yawAngle = yawAngle + z_gyro_value * dt;
  return yawAngle;
}



