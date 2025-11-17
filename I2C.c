#include <avr/io.h>
#include <util/delay.h>
#include <math.h>

//                                                ***I2C communication***

void I2C_init(void) {
    // 100kHz SCL at 16MHz: TWBR = 72, prescaler = 1
    TWSR = 0x00;          // prescaler = 1
    TWBR = 72;            // ~100kHz
    TWCR = (1 << TWEN);   // enable TWI
}

void I2C_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void I2C_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

void I2C_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

uint8_t I2C_read_ack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t I2C_read_nack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}


//                                                    ***reading gyro***

#define MPU_ADDR       0x68
#define MPU_PWR_MGMT_1 0x6B
#define MPU_GYRO_CONFIG 0x1B
#define MPU_SMPLRT_DIV 0x19
#define MPU_CONFIG      0x1A
#define MPU_GYRO_ZOUT_H 0x47

// global bias (deg/s)
static float gyro_z_bias = 0.0f;

void mpu6050_write_reg(uint8_t reg, uint8_t val) {
    I2C_start();
    I2C_write((MPU_ADDR << 1) | 0); // write
    I2C_write(reg);
    I2C_write(val);
    I2C_stop();
}

uint8_t mpu6050_read_reg(uint8_t reg) {
    uint8_t val;
    I2C_start();
    I2C_write((MPU_ADDR << 1) | 0); // write
    I2C_write(reg);
    I2C_start();
    I2C_write((MPU_ADDR << 1) | 1); // read
    val = I2C_read_nack();
    I2C_stop();
    return val;
}

void mpu6050_init(void) {
    I2C_init();

    // Wake up device (clear sleep bit)
    mpu6050_write_reg(MPU_PWR_MGMT_1, 0x00);   // use internal 8MHz, no sleep

    // Gyro config: ±250 deg/s (highest resolution)
    // FS_SEL = 0 -> 131 LSB / deg/s
    mpu6050_write_reg(MPU_GYRO_CONFIG, 0x00);

    // Optional: low-pass filter and sample rate
    mpu6050_write_reg(MPU_CONFIG, 0x03);       // DLPF ~44Hz
    mpu6050_write_reg(MPU_SMPLRT_DIV, 0x04);   // sample rate ~200Hz (if desired)

    gyro_z_bias = 0.0f;
}

int16_t mpu6050_read_gyro_z_raw(void) {
    // read two bytes: ZOUT_H, ZOUT_L
    int16_t z;
    I2C_start();
    I2C_write((MPU_ADDR << 1) | 0);       // write
    I2C_write(MPU_GYRO_ZOUT_H);
    I2C_start();
    I2C_write((MPU_ADDR << 1) | 1);       // read
    uint8_t hi = I2C_read_ack();
    uint8_t lo = I2C_read_nack();
    I2C_stop();

    z = (int16_t)((hi << 8) | lo);
    return z;
}

// convert raw to deg/s, subtract bias
float mpu6050_read_gyro_z_dps(void) {
    int16_t raw = mpu6050_read_gyro_z_raw();
    // sensitivity = 131 LSB / (deg/s) for ±250 dps
    float dps = (float)raw / 131.0f;
    return dps - gyro_z_bias;
}

// call at startup, keep robot still
void mpu6050_calibrate_gyro_z(uint16_t samples) {
    long sum = 0;
    for (uint16_t i = 0; i < samples; i++) {
        sum += mpu6050_read_gyro_z_raw();
        _delay_ms(5);
    }
    float avg = (float)sum / (float)samples;
    gyro_z_bias = avg / 131.0f;  // store in deg/s units
}


//                                        ***accelerorometer***
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_ACCEL_ZOUT_H 0x3F

int16_t mpu6050_read_accel_x_raw(void) {
    int16_t x;
    I2C_start();
    I2C_write((MPU_ADDR << 1) | 0);      // write
    I2C_write(MPU_ACCEL_XOUT_H);
    I2C_start();
    I2C_write((MPU_ADDR << 1) | 1);      // read
    uint8_t hi = I2C_read_ack();
    uint8_t lo = I2C_read_nack();
    I2C_stop();
    x = (int16_t)((hi << 8) | lo);
    return x;
}

int16_t mpu6050_read_accel_z_raw(void) {
    int16_t z;
    I2C_start();
    I2C_write((MPU_ADDR << 1) | 0);
    I2C_write(MPU_ACCEL_ZOUT_H);
    I2C_start();
    I2C_write((MPU_ADDR << 1) | 1);
    uint8_t hi = I2C_read_ack();
    uint8_t lo = I2C_read_nack();
    I2C_stop();
    z = (int16_t)((hi << 8) | lo);
    return z;
}

float mpu6050_read_pitch_deg(void) {
    int16_t ax_raw = mpu6050_read_accel_x_raw();
    int16_t az_raw = mpu6050_read_accel_z_raw();

    // Assuming ~16384 LSB/g for ±2g; but we only care about the ratio
    float ax = ax_raw / 16384.0f;
    float az = az_raw / 16384.0f;

    float pitch = atan2f(-ax, az) * 180.0f / M_PI;   // sign might need flipping
    return pitch;
}