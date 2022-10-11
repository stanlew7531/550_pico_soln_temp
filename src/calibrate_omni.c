/*******************************************************************************
* calibrate_motors.c
*
*
*******************************************************************************/
#include <pico/stdlib.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>
#include <rc/motor/motor.h>
#include <rc/mpu/mpu.h>
#include <rc/fram/fram.h>
#include <rc/defs/mbot_omni_defs.h>
#include <comms/common.h>
#include <comms/protocol.h>
#include <comms/listener.h>
#include <comms/topic_data.h>
#include <comms/messages_mb.h>

#include <math.h>
#include <inttypes.h>

#define LED_PIN 25
#define DT 0.1f


#define MOTOR_STEPS 128

int linear_regression(int data[4][MOTOR_STEPS], float* coeffs)
{
    double sum_x[3] = {0.0f, 0.0f, 0.0f};
    double sum_x_squared[3] = {0.0f, 0.0f, 0.0f};
    double sum_x_y[3] = {0.0f, 0.0f, 0.0f};
    double sum_y[3] = {0.0f, 0.0f, 0.0f};
    for (int j = 0; j < MOTOR_STEPS; j++)
    {
        for (int i = 0; i < 3; i++)
        {
            sum_x[i] += data[i + 1][j];
            sum_x_squared[i] += pow(data[i + 1][j],2);
            sum_y[i] += data[0][j];
            sum_x_y[i] += data[0][j] * data[i + 1][j];
        }
    }
    for (int i = 0; i < 3; i++)
    {
        coeffs[2*i] = (MOTOR_STEPS * sum_x_y[i] - sum_x[i] * sum_y[i]) / (MOTOR_STEPS * sum_x_squared[i] - pow(sum_x[i],2));
        coeffs[2*i + 1] = (sum_x_squared[i] * sum_y[i] - sum_x[i] * sum_x_y[i]) / (MOTOR_STEPS * sum_x_squared[i] - pow(sum_x[i],2));
    }
    return 1;
}


int main() {
    sleep_ms(2000);

    bi_decl(bi_program_description("The binary for motor calibration on MBot Pico Board."));
    bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));
    set_sys_clock_khz(250000, true);

    stdio_init_all();
    sleep_ms(1500); // quick sleep so we can catch the bootup process in terminal
    printf("\nMBot Booting Up!\n");

    printf("initing motors...\n");
    rc_motor_init();
    printf("initing encoders...\n");
    rc_encoder_init();

    // Pins
    // for the i2c to the IMU
    const uint sda_pin = 4;
    const uint scl_pin = 5;

    // Ports
    i2c_inst_t *i2c = i2c0;
    //Initialize I2C port at 400 kHz
    i2c_init(i2c, 400 * 1000);
    // Initialize I2C pins
    printf("setting i2c functions...\n");
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    printf("setting i2c pullups...\n");
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(sda_pin, scl_pin, GPIO_FUNC_I2C));

    printf("setting heartbeat LED GPIOs...\n");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    int step = 32768 / (MOTOR_STEPS / 4);
    const float delta2angvel =  (2 * M_PI / (GEAR_RATIO * ENCODER_RES)) / DT; // 4.027682915E-2

    int enc_delta_a = 0;
    int enc_delta_b = 0;
    int enc_delta_c = 0;

    int data[4][MOTOR_STEPS];
    int index = 0;
    
    printf("starting calibration...");
    for (int duty = 0; index < MOTOR_STEPS; index += 1)
    {
        rc_motor_set(0, duty); // sets all motors
        sleep_ms(DT * 1000);
        enc_delta_a = rc_encoder_read_delta(A_MOTOR_CHANNEL);
        enc_delta_b = rc_encoder_read_delta(B_MOTOR_CHANNEL);
        enc_delta_c = rc_encoder_read_delta(C_MOTOR_CHANNEL);

        data[0][index] = duty;
        data[1][index] = (float)enc_delta_a * delta2angvel;
        data[2][index] = (float)enc_delta_b * delta2angvel;
        data[3][index] = (float)enc_delta_c * delta2angvel;

        switch (index)
        {
            case MOTOR_STEPS / 4:
                step *= -1;
                break;
            case 3 * (MOTOR_STEPS / 4):
                step *= -1;
                break;
            default:
                break;
        }
        duty += step;
    }
    rc_motor_set(0, 0); // sets all motors
    printf("done collecting data\n");
    sleep_ms(1000);

    printf("creating coeffs...\n");
    float* C = (float*)malloc(6 * sizeof(float));

    printf("doing linear regression...\n");
    linear_regression(data, C);

    printf("got coefficients:\n");
    for (int i = 0; i < 3; i++) printf("\t%f, %f\n", C[2*i], C[2*i+1]);

    sleep_ms(500);

    // store to FRAM here
    uint8_t B[WHEEL_CALIBRATION_LEN];
    memcpy(B, C, WHEEL_CALIBRATION_LEN);
    printf("writing to fram...\r\n");

    if (!mb_write_fram(i2c, WHEEL_CALIBRATION_ADDR, WHEEL_CALIBRATION_LEN, &B[0])) 
    {
        printf("writing to fram success.\n");
    }
    else 
    {
        printf("writing to fram failed.\n");
    }

    rc_motor_cleanup();
    rc_encoder_cleanup();
    printf("done!\r\n");
    return 0;
}
