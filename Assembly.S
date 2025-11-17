;defining stuff
.equ DDRB, 0x04
.equ DDRD, 0x0A
.equ PORTB, 0x05
.equ PORTD, 0x0B
.equ DDRC, 0x07
.equ PORTC, 0x08

;                               ***motors****
.global motor_init
motor_init:
sbi DDRB, 0 ; AIN1
sbi DDRD, 7 ; BIN1
sbi DDRD, 5 ; PWMA
sbi DDRD, 6 ; PWMB
sbi DDRD, 3 ; STBY
sbi PORTD, 3  ;set STBY pin high
ret

.global motor_drive
motor_drive:
sbi PORTD, 5 ;set PWMA high (max speed cuz bruh)
sbi PORTD, 6 ;set PWMB high

;for AIN(B0) and BIN(D7), 1 = forward, 0 = backward
;goes to r24(A), r22(B)

tst r24
breq left_back

sbi PORTB, 0  ;make left go forward
rjmp right

left_back: 
cbi PORTB, 0  ;make left go backward

right:

tst r22
breq right_back

sbi PORTD, 7  ;make right go forward
ret

right_back:
cbi PORTD, 7  ;make right go back
ret


.global motor_off
motor_off: 
;set all to 0
cbi PORTD, 5 
cbi PORTD, 6
ret


;                                         **LED**
.global led_init
led_init:
sbi DDRB, 3
cbi PORTB, 3
ret

.global led_on
led_on:
sbi PORTB, 3
ret

.global led_off
led_off:
cbi PORTB, 3
ret

;                                           ***Ultrasonic sensor**
.global ultrasonic_init
ultrasonic_init:
sbi DDRB, 5 ;trig pin in
cbi DDRB, 4 ;echo out
ret