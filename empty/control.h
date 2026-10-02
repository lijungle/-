#ifndef CONTROL_H
#define CONTROL_H
#include <stdint.h>

typedef struct { float kp, ki, kd, integral, previous, limit; } PID;
enum { ROUTE_IDLE, ROUTE_STRAIGHT, ROUTE_SQUARE_FORWARD, ROUTE_BRAKE, ROUTE_TURN_OUT, ROUTE_CHECK,
       ROUTE_SHIFT_OUT, ROUTE_TURN_FORWARD, ROUTE_PASS, ROUTE_TURN_IN,
       ROUTE_SHIFT_IN, ROUTE_TURN_HOME };
enum { STOP_NONE, STOP_DONE, STOP_KEY, STOP_IMU, STOP_SENSOR, STOP_BLOCKED,
       STOP_ENDPOINT, STOP_STALL, STOP_TURN_TIMEOUT, STOP_PATH };
enum { SETTING_NONE, SETTING_DISTANCE, SETTING_SPEED, SETTING_COUNTS };
enum { MODE_STRAIGHT, MODE_SQUARE };
enum { SONAR_PENDING, SONAR_VALID, SONAR_NO_ECHO, SONAR_FAULT };

typedef struct {
    uint8_t ir_left, ir_right, sonar_result;
    uint16_t distance_mm;
    uint32_t measured_ms;
    uint8_t ir_left_level, ir_right_level, echo_level, echo_stage;
    uint16_t echo_us, sonar_triggers;                       /* 检测页显示输入电平、捕获阶段、脉宽与触发次数 */
} ObstacleState;
typedef struct {
    PID left, right;
    float target, heading_target, heading_integral;
    float start_heading, previous_heading, x, y, bypass_x, detour_start_x;
    float motion_distance, square_distance;
    int32_t previous_left, previous_right;
    uint32_t phase_ms, aligned_ms, motion_ms;
    uint8_t running, phase, next_phase, aligned, setting, distance_m, stop_reason, tried_other;
    uint8_t mode, square_edge;
    uint16_t counts_per_meter;
    uint8_t sensor_test;                                   /* 检测页只允许在停车时进入 */
    int8_t detour_side;                                    /* 左绕为正，右绕为负 */
    uint8_t keys_last, key2_long, key3_long;
    uint32_t key2_ms;
} CarControl;

void pid_reset(PID *p);
float pid_step(PID *p, float target, float measured, float dt);
float wrap_angle(float x);
float fuzzy_heading(CarControl *c, float heading, float gyro, float dt, float *correction);
void control_init(CarControl *c);
void control_start(CarControl *c, float heading, int32_t left_total, int32_t right_total, uint32_t now_ms);
void control_stop(CarControl *c);
int control_keys(CarControl *c, int key1, int key2, int key3, int key4, uint32_t now_ms);
void control_step(CarControl *c, float left_count, float right_count, int32_t left_total, int32_t right_total,
                  float heading, float gyro, float dt, uint32_t now_ms, const ObstacleState *obstacle,
                  float *left_pwm, float *right_pwm);

#endif
