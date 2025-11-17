#ifndef ASSEMBLY_H
#define ASSEMBLY_H

#ifdef __ASSEMBLER__


#else 

  #include <stdint.h>
  #ifdef __cplusplus
  extern "C" {
  #endif

  //Assembly fucntions

  void motor_init(void);
  void motor_drive(uint8_t left, uint8_t right);
  void motor_off(void);
  void led_init(void);
  void led_on(void);
  void led_off(void); 
  void ultrasonic_init(void);

  //C fucntions

  //gyroscope
  void mpu6050_init(void);
  void mpu6050_calibrate_gyro_z(uint16_t samples);
  float mpu6050_read_gyro_z_dps(void);

  float mpu6050_read_gyro_x_dps(void);
  void mpu6050_calibrate_gyro_x(uint16_t samples);

  #ifdef __cplusplus
  }
  #endif

#endif
#endif



