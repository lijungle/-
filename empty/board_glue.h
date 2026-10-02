#ifndef BOARD_GLUE_H
#define BOARD_GLUE_H
#include <stdint.h>
#include "control.h"
#include "jy61p.h"
void board_init(void);
void board_poll(void);
void oled_init(void);
void oled_poll(void);
uint32_t board_millis(void);
int key1_pressed(void), key2_pressed(void), key3_pressed(void);
void motor_set(float left, float right);
void oled_status(const CarControl *car, const ObstacleState *obstacle, float yaw);
const ObstacleState *obstacle_state(void);
int encoder_left_delta(void), encoder_right_delta(void);
void encoder_counts(int32_t *left, int32_t *right);
void encoder_reset_counts(void);
float imu_yaw(void), imu_gyro_z(void);
JY61P *imu_state(void);
#endif
