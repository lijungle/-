#include <stdint.h>
#include "control.h"
#include "jy61p.h"
#include "board_glue.h"

/******************************************************************
 * 函 数 名 称：main
 * 函 数 说 明：处理参数、GPS、ESP32 通信和十毫秒控制
 * 函 数 形 参：无
 * 函 数 返 回：永不返回
 ******************************************************************/
int main(void)
{
    CarControl car;
    control_init(&car);                                     /* 初始化左右轮 PID 参数 */
    board_init();                                           /* 初始化 GPIO、UART 和定时器 */
    uint32_t last_ms = board_millis();                       /* 从外设初始化完成后开始计时 */
    for (;;) {
        board_poll();                                       /* 非阻塞刷新和惯导快照更新 */
        uint32_t now = board_millis();
        int key1 = key1_pressed();
        int key2 = key2_pressed();
        int key3 = key3_pressed();
        int key4 = key4_pressed();
        uint32_t elapsed = now - last_ms;                   /* 无符号减法允许毫秒计数自然折返 */
        if (elapsed < 10) continue;                         /* 只在新的控制节拍执行一次 */
        last_ms = now;
        const ObstacleState *o = obstacle_state();
        int imu_fault = jy61p_timeout(imu_state(), now);
        int start = control_keys(&car, key1, key2, key3, key4, now);
        if (imu_fault && car.running) { control_stop(&car); car.stop_reason = STOP_IMU; }
        ESPCommand command;
        if (esp_command(&command)) {
            start = 0;                                    /* 同周期串口停止不能被启动键覆盖 */
            if (command.cmd == ESP_STOP || (!key3 && !imu_fault))
                control_remote(&car, &command, now);       /* 实体停止键和惯导故障优先于远程起动 */
        }
        int32_t left_total, right_total;
        if (start && car.phase != ROUTE_REMOTE) {
            if (imu_fault) car.stop_reason = STOP_IMU;
            else {
                encoder_reset_counts();                    /* 新任务重新建立位置零点，速度历史继续维护 */
                encoder_counts(&left_total, &right_total);
                control_start(&car, imu_yaw(), left_total, right_total, now);
            }
        }
        encoder_counts(&left_total, &right_total);
        float left_pwm = 0.0f;
        float right_pwm = 0.0f;
        float left_speed = (float)encoder_left_delta() * 10.0f / elapsed;
        float right_speed = (float)encoder_right_delta() * 10.0f / elapsed;
        control_step(&car, left_speed, right_speed,
                     left_total, right_total, imu_yaw(), imu_gyro_z(), elapsed / 1000.0f,
                     now, o, &left_pwm, &right_pwm);        /* 反馈折算到每十毫秒计数，位置使用累计值 */
        motor_set(left_pwm, right_pwm);                     /* 停止、设置、故障周期均为零输出刹车 */
        esp_report(imu_yaw(), (left_speed + right_speed) * 50.0f / car.counts_per_meter); /* 平均轮速换算为米每秒，后退为负 */
        oled_status(&car, o, imu_yaw());
    }
}
