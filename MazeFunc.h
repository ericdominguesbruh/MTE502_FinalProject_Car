#ifdef __ASSEMBLER__


#else 
  extern "C" void motor_init(void);
  extern "C" void motor_drive(uint8_t left, uint8_t right);
  extern "C" void motor_off(void);
  extern "C" void led_init(void);
  extern "C" void led_on(void);
  extern "C" void led_off(void); 
  extern "C" void ultrasonic_init(void);
#endif


