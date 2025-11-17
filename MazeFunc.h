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

  void I2C_init(void);
  void I2C_start(void);
  void I2C_stop(void);
  void I2C_write(uint8_t data);
  uint8_t I2C_read_ack(void);
  uint8_t I2C_read_nack(void);

  void mpu6050_init(void);
  void mpu6050_calibrate_gyro_z(uint16_t samples);
  int16_t mpu6050_read_gyro_z_raw(void);
  float mpu6050_read_gyro_z_dps(void);

  #ifdef __cplusplus
  }
  #endif

#endif
#endif



