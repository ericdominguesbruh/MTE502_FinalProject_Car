//V3: adds gyroscope for turns
#include "MazeFunc.h"
#include <avr/io.h>

//gloabl variables

//for line following sensors (phototransistors)
uint16_t R; //A0
uint16_t M; //A1
uint16_t L; //A2
uint8_t Apins[] = {0, 1, 2};
uint16_t *trackers[] = {&R, &M, &L};
int Black[3]; //holds bool values for if its black or not (R -> L)
int black_colour = 500; //threshhold for black line

int LineFlag = false; //flag for when line following is done

int done = false; //is robot out of maze?

//global fucntions

void tracker_init(){
  //ref to 5v
  ADMUX = (1 << REFS0);

  ADCSRA = (1 << ADEN)         // enable ADC
  | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);   // prescaler 128
}

void motor_pwm_init(void) {
  // Set D5 (PD5) and D6 (PD6) as outputs for PWM
  DDRD |= (1 << PD5) | (1 << PD6);

  // Fast PWM, non-inverting on OC0A (D6) and OC0B (D5)
  // WGM01=1, WGM00=1 → Fast PWM, TOP = 0xFF
  // COM0A1=1 → PWM on OC0A (D6)
  // COM0B1=1 → PWM on OC0B (D5)
  TCCR0A = (1 << WGM01) | (1 << WGM00)
          | (1 << COM0A1) | (1 << COM0B1);

  // Prescaler /64 → ~1kHz PWM (fine for motors)
  TCCR0B = (1 << CS01) | (1 << CS00);

  // Start stopped
  OCR0A = 0;   // motor B speed
  OCR0B = 0;   // motor A speed
}

void motor_drive2(uint8_t left, uint8_t right, uint8_t speed){
  if (left == 1) {
    PORTB |=  (1 << PB0);  // AIN1 = 1
    if (right == 1) PORTD |=  (1 << PD7);  // BIN1 = 1
    else PORTD &=  ~(1 << PD7); 
  }
  else PORTB &= ~(1 << PB0);

  OCR0B = speed;   // A 
  OCR0A = speed;   // B
}

void motor_off2() {
  motor_drive2(1,1,0);
}

void servo_init(){
  DDRB |= (1<<PB2);
  TCCR1A = (1 << COM1B1);                  // non-inverting mode on OC1B
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);                    // prescaler = 8
  TCCR1A |= (1 << WGM11);
  ICR1 = 39999;   // 20 ms frame @ 16MHz/8
}

void servo_move(char theta){
  int diff = -30;

  switch(theta){
    case 'S':
      OCR1B = ((1500*2)+diff*5);
      break;
    case 'L':
      OCR1B = ((2500*2)+diff*4);
      break;
    case 'R':
      OCR1B = ((500*2)-20);
      break;
  }
}

void read_tracker(){
  for (int i = 0; i <3; i++){
      ADMUX = (ADMUX & 0xF0) | Apins[i];     // ADC
    ADCSRA |= (1 << ADSC);          // start
    while (ADCSRA & (1 << ADSC));   // wait
    *trackers[i] = ADC;    
    if (*trackers[i] > black_colour /*&& *trackers[i] < 950*/) { //upper and lower values of detecting black line
      Black[i] = true; 
    }
    else Black[i] = false;
  }
}

void LineFollowNew(){
  read_tracker();
  uint8_t speed = 70;
  if(Black[0] == 0 && Black[1] == 1 && Black[2] == 0) {
    motor_drive2(1,1,speed);  //line in middle, go straight
  }
  else if (Black[0] == 1) { //small left
    motor_drive2(1,0,speed);
  }
  else if (Black[2] == 1) { //small right
    motor_drive2(0,1,speed);
  }
  if (Black[0] == 1 && Black[1] == 1 && Black[2] == 1) {
    motor_off2(); 
    LineFlag = true;
  }
}

void trig_pulse() {
    PORTB &= ~(1 << PB5);      // ensure low
    _delay_us(2);

    PORTB |= (1 << PB5);       // HIGH
    _delay_us(10);             // 10 µs pulse

    PORTB &= ~(1 << PB5);      // LOW
}

int ultrasonic_read(){
  trig_pulse();
  uint16_t width = 0;
  // Wait for echo HIGH (start of pulse)
  while (!(PINB & (1 << PB4)));  

  // Count while echo is HIGH
  while (PINB & (1 << PB4)) {
      _delay_us(1);
      width++;
  }
  return width/58; //distance in cm
}

void read_wall(){   //if wall stop, if no wall go forward
  int wall_dis = 10;

  int d = ultrasonic_read();
  if (d <= wall_dis) {
    motor_off2(); 
    _delay_ms(500);
    maze();
    //if (d > 100) done = true;
  }
}

void drive_up_a_bit(){    //fucntion to drive a lil foreward bc it stops kinda far from wall so left & right scan read better
  motor_drive2(1,1,80);
  _delay_ms(250);
  motor_off2();
  _delay_ms(100); //see if works without
}

void maze(){
  int dis = 20;
  int delay = 400;

  servo_move('L');    //look left and right
  _delay_ms(delay);
  int left = ultrasonic_read();
  _delay_ms(delay);
  servo_move('R');
  _delay_ms(delay);
  int right = ultrasonic_read();
  _delay_ms(delay);

  servo_move('S');
  //_delay_ms(delay);

  if (right > dis) {
    drive_up_a_bit(); 
    turn90('R');
    }
  else if (left > dis) {
    drive_up_a_bit(); 
    turn90('L');
  }
  else turn180();
}

void turn90(char dir){  //fucntion to turn 90 degrees left or right
  int speed = 70;
  float target_angle = 70;  //ik it supposed to turn 90 but Im tweaking cuz of errors
  float angle = 0;
  int dt_ms = 5; //ms
  float dt = 0.005;

  switch(dir){
    case 'R':
      motor_drive2(1,0,speed);
      while (fabs(angle) < target_angle){
        float wz = mpu6050_read_gyro_z_dps(); //deg /s 
        angle += wz * (dt);
        _delay_ms(dt_ms);
      }
      motor_off2();

      break;

    case 'L':
      motor_drive2(0,1,speed);
      while (fabs(angle) < target_angle){
        float wz = mpu6050_read_gyro_z_dps(); //deg /s 
        angle += wz * (dt);
        _delay_ms(dt_ms);
      }
      motor_off2();

      break;
  }
  motor_drive2(1,1,100);
}

void turn180(){
  int speed = 70;
  float target_angle = 158;  //ik it supposed to turn 180 but Im tweaking cuz of errors
  float angle = 0;
  int dt_ms = 5; //ms
  float dt = 0.005;

  motor_drive2(1,0,speed);
  while (fabs(angle) < target_angle){
        float wz = mpu6050_read_gyro_z_dps(); //deg /s 
        angle += wz * (dt);
        _delay_ms(dt_ms);
      }
  motor_off2();
  _delay_ms(100);
  motor_drive2(1,1,100);
}


int main(){
  motor_init();
  motor_pwm_init();
  tracker_init();
  led_init();
  servo_init();
  ultrasonic_init();
                                              Serial.begin(9600);          //delete all serial stuffs**************************************8

  while (LineFlag == false){
    LineFollowNew();
  }

  led_on();
  servo_move('S');
  _delay_ms(1000);

  mpu6050_init();     //initialize and calibrate gyro
  mpu6050_calibrate_gyro_z(200);

  _delay_ms(2000);  //stop fir 3 seconds
  motor_drive2(1,1,100);

  while (LineFlag == true && done == false){   //navigate once in the maze
    read_wall();
    _delay_ms(5); //doesnt work without

    //testing turns
    /*motor_off2();
    turn90('L');
    motor_off2();
    _delay_ms(2000);
    turn90('R');
    motor_off2();
    _delay_ms(2000);
    turn180();
    motor_off2();
    _delay_ms(2000);*/
  }

    motor_off2();
    led_off();
    turn180();
    motor_off2();

  while (done == true){
    motor_drive2(1,0, 120);
    led_on();
    _delay_ms(200);
    motor_off2();
    motor_drive2(0,1,120);
    led_off();
    _delay_ms(200);
    motor_off2();
  }

  return 0;
}
//add celebrate mode at end