/**
 * This file is the main executable for the MBot firmware.
 */

#include <pico/stdlib.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>
#include <rc/motor/motor.h>
#include <rc/mpu/mpu.h>
#include <rc/defs/common_defs.h>
#include <comms/common.h>
#include <comms/protocol.h>
#include <comms/listener.h>
#include <comms/topic_data.h>
#include <comms/messages_mb.h>

#include <math.h>
#include <inttypes.h>

// taken from my old botlab code
#define SLOPE_L                 -1.01
#define SLOPE_R                 1.01
#define INTERCEPT_L             -0.0
#define INTERCEPT_R             -0.0

// data to hold current mpu state
static mb_mpu_data_t mpu_data;

uint64_t timestamp_offset = 0;
uint64_t current_pico_time = 0;

//data to hold the IMU results
mbot_imu_t current_imu = {0};
// data to hold the received timestamp
timestamp_t received_time = {0};
// current odometry state
odometry_t current_odom = {0};
// current encoder states
mbot_encoder_t current_encoders = {0};
float _meters_per_tick = ((2.0 * PI * WHEEL_RADIUS) / (GEAR_RATIO * ENCODER_RES));
// current body frame command
mbot_motor_command_t current_cmd = {0};

void timestamp_cb(timestamp_t* received_timestamp)
{
    // if we havent set the offset yet
    if(timestamp_offset == 0)
    {
        uint64_t cur_pico_time = to_us_since_boot(get_absolute_time());
        timestamp_offset = received_timestamp->utime - cur_pico_time;
    }
}

void reset_encoders_cb(mbot_encoder_t* received_encoder_vals)
{
    rc_encoder_write(LEFT_MOTOR_CHANNEL, received_encoder_vals->leftticks);
    rc_encoder_write(RIGHT_MOTOR_CHANNEL, received_encoder_vals->rightticks);
}


void reset_odometry_cb(odometry_t* received_odom)
{
    current_odom.utime = received_odom->utime;
    current_odom.x = received_odom->x;
    current_odom.y = received_odom->y;
    current_odom.theta = received_odom->theta;
}

void register_topics()
{
    // timesync topic
    comms_register_topic(MBOT_TIMESYNC, sizeof(timestamp_t), (Deserialize)&timestamp_t_deserialize, (Serialize)&timestamp_t_serialize, (MsgCb)&timestamp_cb);
    // odometry topic
    comms_register_topic(ODOMETRY, sizeof(odometry_t), (Deserialize)&odometry_t_deserialize, (Serialize)&odometry_t_serialize, NULL);
    // reset odometry topic
    comms_register_topic(RESET_ODOMETRY, sizeof(odometry_t), (Deserialize)&odometry_t_deserialize, (Serialize)&odometry_t_serialize, (MsgCb)&reset_odometry_cb);
    // IMU topic
    comms_register_topic(MBOT_IMU, sizeof(mbot_imu_t), (Deserialize)&mbot_imu_t_deserialize, (Serialize)&mbot_imu_t_serialize, NULL);
    // encoders topic
    comms_register_topic(MBOT_ENCODERS, sizeof(mbot_encoder_t), (Deserialize)&mbot_encoder_t_deserialize, (Serialize)&mbot_encoder_t_serialize, NULL);
    // reset encoders topic
    comms_register_topic(RESET_ENCODERS, sizeof(mbot_encoder_t), (Deserialize)&mbot_encoder_t_deserialize, (Serialize)&mbot_encoder_t_serialize, (MsgCb)&reset_encoders_cb);
    // motor commands topic
    comms_register_topic(MBOT_MOTOR_COMMAND, sizeof(mbot_motor_command_t), (Deserialize)&mbot_motor_command_t_deserialize, (Serialize)&mbot_motor_command_t_serialize, NULL);
}

bool timer_cb(repeating_timer_t* rt)
{    
    // only run if we've received a timesync message...
    if(comms_get_topic_data(MBOT_TIMESYNC, &received_time))
    {
        uint64_t cur_pico_time = to_us_since_boot(get_absolute_time()) + timestamp_offset;
        uint64_t latency_time = cur_pico_time - current_pico_time;
        current_pico_time = cur_pico_time;
        printf("start cycle...\r\n");
        // first, get the IMU data and send across the wire
        current_imu.utime = cur_pico_time;//received_time.utime;
        current_imu.accel[0] = mpu_data.accel[0];
        current_imu.accel[1] = mpu_data.accel[1];
        current_imu.accel[2] = mpu_data.accel[2];
        current_imu.gyro[0] = mpu_data.gyro[0];
        current_imu.gyro[1] = mpu_data.gyro[1];
        current_imu.gyro[2] = mpu_data.gyro[2];
        current_imu.tb[0] = mpu_data.dmp_TaitBryan[0];
        current_imu.tb[1] = mpu_data.dmp_TaitBryan[1];
        current_imu.tb[2] = mpu_data.dmp_TaitBryan[2];
        current_imu.temperature = mpu_data.temp;

        // read the encoders
        int enc_cnt_l = rc_encoder_read_count(LEFT_MOTOR_CHANNEL);
        int enc_delta_l = rc_encoder_read_delta(LEFT_MOTOR_CHANNEL);
        int enc_cnt_r = rc_encoder_read_count(RIGHT_MOTOR_CHANNEL);
        int enc_delta_r = rc_encoder_read_delta(RIGHT_MOTOR_CHANNEL);
        current_encoders.utime = cur_pico_time;//received_time.utime;
        current_encoders.right_delta = enc_delta_r;
        current_encoders.rightticks = enc_cnt_r;
        current_encoders.left_delta = enc_delta_l;
        current_encoders.leftticks = enc_cnt_l;

        // compute new odometry
        float dx = _meters_per_tick*(current_encoders.left_delta + current_encoders.right_delta) / 2;
        float dtheta_odom = _meters_per_tick*(current_encoders.right_delta - current_encoders.left_delta) / WHEEL_BASE;
        current_odom.utime = cur_pico_time;//received_time.utime;
        current_odom.x += dx * cos(current_odom.theta + dtheta_odom / 2.0f);
        current_odom.y += dx * sin(current_odom.theta + dtheta_odom / 2.0f);
        current_odom.theta += dtheta_odom;
        // constrain +/- 180
        while(current_odom.theta > PI)
        {
            current_odom.theta -= 2 * PI;
        }
        while(current_odom.theta < -PI)
        {
            current_odom.theta += 2 * PI;
        }
        // if(current_odom.theta > PI || current_odom.theta < -PI)
        // {
        //     current_odom.theta = current_odom.theta - floor(current_odom.theta / (2 * PI)) * (2 * PI);
        // }

        // get the current motor command state (if we have one)
        if(comms_get_topic_data(MBOT_MOTOR_COMMAND, &current_cmd))
        {
            // compute the desired speed of each wheel
            float l_duty = SLOPE_L * (current_cmd.trans_v - WHEEL_BASE * current_cmd.angular_v / 2) + INTERCEPT_L;
            float r_duty = SLOPE_R * (current_cmd.trans_v + WHEEL_BASE * current_cmd.angular_v / 2) + INTERCEPT_R;
            int16_t l_cmd = (int)(l_duty * 0.95 * pow(2, 15));
            int16_t r_cmd = (int)(r_duty * 0.95 * pow(2, 15));
            // issue the commands to the motors
            rc_motor_set(LEFT_MOTOR_CHANNEL, l_cmd);
            rc_motor_set(RIGHT_MOTOR_CHANNEL, r_cmd);


            printf("got motor commands trans = %f, ang = %f \r\n l = %f r = %f, setting rc motors l = %d, r = %d\r\n",
                current_cmd.trans_v, current_cmd.angular_v,
                l_duty, r_duty,
                l_cmd,  r_cmd);
        }

        // write the encoders to serial
        comms_write_topic(MBOT_ENCODERS, &current_encoders);
        // send odom on wire
        comms_write_topic(ODOMETRY, &current_odom);
        // write the IMU to serial
        comms_write_topic(MBOT_IMU, &current_imu);
        uint64_t fn_run_len = to_us_since_boot(get_absolute_time()) + timestamp_offset - cur_pico_time;
        printf("end cycle, runtime = %" PRIu64 ", latency = %" PRIu64 "\r\n", fn_run_len, latency_time);
    }

    return true;
}

int main() {
    bi_decl(bi_program_description("The main binary for the MBot Pico Board."));
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

    printf("initializing DMP...\n");
    mb_mpu_config_t mpu_config = mb_mpu_default_config();
    mpu_config.i2c_bus = i2c;
    mpu_config.dmp_fetch_accel_gyro=1;
    mpu_config.enable_magnetometer = 1;
    mpu_config.read_mag_after_callback = 0;
    mpu_config.orient = ORIENTATION_Z_UP;
    mpu_config.dmp_sample_rate = 200;
    //mb_mpu_reset_accel_cal(mpu_config.i2c_bus);
    mb_mpu_calibrate_gyro_routine(mpu_config);
    //sleep_ms(2000);
    //mb_mpu_calibrate_accel_routine(mpu_config);
    sleep_ms(500);
    mb_mpu_initialize_dmp(&mpu_data, mpu_config);
    gpio_set_irq_enabled_with_callback(MB_MPU_INTERRUPT_GPIO, GPIO_IRQ_EDGE_FALL, true, &mb_dmp_callback);
    printf("MPU Initialized!\n");


    //create topics and register the serialize/deserialize functions
    printf("init comms...\r\n");
    comms_init_protocol();
    comms_init_topic_data();
    register_topics();

    // launch the other core's comm loop
    printf("starting comms on core 1...\r\n");
    multicore_launch_core1(comms_listener_loop);

    // wait for other core to get rolling
    sleep_ms(500);

    int running = 1;

    // run the main loop as a timed interrupt
    printf("starting the timed interrupt...\r\n"); 
    repeating_timer_t loop_timer;
    add_repeating_timer_ms(MAIN_LOOP_PERIOD * 1000, timer_cb, NULL, &loop_timer); // 1000x to convert to ms

    printf("Done Booting Up!\n\n");   

    while (running)
    {
        //spin
        int asdf = 1;
    }
}