#include "mbed.h"
#include "arm_math.h"
#include <cstdio>  // 添加这个头文件以使用 `fopen` 等函数

#define SAMPLE_RATE 50
#define SAMPLE_DURATION 3
#define SAMPLE_COUNT (SAMPLE_RATE * SAMPLE_DURATION)
#define FFT_SIZE 256
#define MIN_AMPLITUDE 0.3f

#define LSM6DSL_ADDR (0x6A << 1)
#define OUTX_L_XL 0x28
#define CTRL1_XL 0x10
#define WHO_AM_I 0x0F

float vector_buffer[FFT_SIZE] = {0};
float fft_out[FFT_SIZE];
float magnitude[FFT_SIZE / 2];
volatile bool button_pressed_feedback = false;

UnbufferedSerial pc(USBTX, USBRX, 115200);  // TX, RX
I2C i2c(PB_11, PB_10);

InterruptIn button(BUTTON1);
volatile bool isRunning = false;
void button_isr() {
    isRunning = !isRunning;
    button_pressed_feedback = true;
}

DigitalOut ledTremor (LED1);
DigitalOut ledDyskinesia (LED2); 

uint8_t read_reg(uint8_t reg) {
    char data = reg;
    i2c.write(LSM6DSL_ADDR, &data, 1, true);
    i2c.read(LSM6DSL_ADDR, &data, 1);
    return data;
}

void write_reg(uint8_t reg, uint8_t value) {
    char data[2] = { (char)reg, (char)value };
    i2c.write(LSM6DSL_ADDR, data, 2);
}

void init_lsm6dsl() {
    // ODR_XL = 52 Hz, FS_XL = ±2g, BW = 100 Hz
    write_reg(CTRL1_XL, 0b01000000);
}

void read_accel(float &ax, float &ay, float &az) {
    char reg = OUTX_L_XL;
    char raw[6];
    i2c.write(LSM6DSL_ADDR, &reg, 1, true);
    i2c.read(LSM6DSL_ADDR, raw, 6);

    int16_t x = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t y = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t z = (int16_t)((raw[5] << 8) | raw[4]);

    float sensitivity = 0.061f;
    ax = x * sensitivity * 0.00981f;
    ay = y * sensitivity * 0.00981f;
    az = z * sensitivity * 0.00981f;

    // printf("Raw X: %d, Y: %d, Z: %d\r\n", x, y, z);
}

int main() {
    init_lsm6dsl();
    pc.set_blocking(false);
    i2c.frequency(400000);
    button.fall(&button_isr);

    arm_rfft_fast_instance_f32 fft_instance;
    arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);

    while (!isRunning) {
        ThisThread::sleep_for(100ms);
    }

    while (true) {

        if (button_pressed_feedback) {
            button_pressed_feedback = false;
            ledTremor = 1;
            ledDyskinesia = 1;
            isRunning == false? ThisThread::sleep_for(2000ms) : ThisThread::sleep_for(1000ms); //2s for stop, 1s for start
            ledTremor = 0;
            ledDyskinesia = 0;
        }

        if (!isRunning) {
            while (!isRunning) {
                ThisThread::sleep_for(100ms);
            }
        }

        ledTremor = 0;
        ledDyskinesia = 0;

        for (int i = 0; i < SAMPLE_COUNT; i++) {
            float32_t ax, ay, az;
            read_accel(ax, ay, az);
            float32_t acc_magnitude = sqrt(ax*ax + ay*ay + az*az);
            vector_buffer[i] = acc_magnitude;
            ThisThread::sleep_for(20ms);
        }
        
        float sum = 0.0f;
        for (int i = 0; i < SAMPLE_COUNT; i++) sum += vector_buffer[i];
        float mean = sum / SAMPLE_COUNT;
        for (int i = 0; i < SAMPLE_COUNT; i++) vector_buffer[i] -= mean;

        for (int i = 0; i < SAMPLE_COUNT; i++) {
            float w = 0.54f - 0.46f * cosf(2 * PI * i / (SAMPLE_COUNT - 1));
            vector_buffer[i] *= w;
        }

        for (int i = SAMPLE_COUNT; i < FFT_SIZE; i++) vector_buffer[i] = 0.0f;

        arm_rfft_fast_f32(&fft_instance, vector_buffer, fft_out, 0);
        arm_cmplx_mag_f32(fft_out, magnitude, FFT_SIZE / 2);
        
        uint32_t max_index;
        float max_value;
        float32_t freq_res = (float32_t)SAMPLE_RATE / FFT_SIZE;

        // filter out noise
        for (int i = 0; i < FFT_SIZE / 2; i++) {
            if (magnitude[i] < MIN_AMPLITUDE) {
                magnitude[i] = 0.0f;
            }
        }

        arm_max_f32(magnitude, FFT_SIZE / 2, &max_value, &max_index);

        float dominant_freq = max_index * freq_res;
        printf("Max magnitude: %.1f at bin %lu (%.2f Hz)\r\n", 
            max_value, max_index, dominant_freq);

        if (dominant_freq >= 3.0f && dominant_freq <= 5.0f){
            printf("Likely Tremor\r\n");
            for (int k = 0; k < 10; k++) {
                ledTremor = !ledTremor;
                thread_sleep_for(100);
            }
        }
        else if (dominant_freq > 5.0f && dominant_freq <= 7.0f){
            printf("Likely Dyskinesia\r\n");
            for (int k = 0; k < 20; k++) {
                ledDyskinesia = !ledDyskinesia;
                thread_sleep_for(100);
            }
        }
        else{
            printf("No significant activity\r\n");
            ledTremor = 0;
            ledDyskinesia = 0;
        }
    }
}