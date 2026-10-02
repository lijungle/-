/* 主机集成自检：运行实际主循环、控制和惯导超时判断，不连接硬件。 */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include "../board_glue.h"

static jmp_buf finish;
static uint32_t test_ms;
static unsigned scenario, was_running;
static JY61P sample;
static ObstacleState obstacle;
static CarControl last_car;
static float last_left, last_right;

/* 每次轮询推进十毫秒，用有限模拟时间结束实际无限主循环。 */
#define board_init() ((void)0)
#define board_millis() test_ms
#define board_poll() do { \
    test_ms += 10; if (test_ms > 2500) longjmp(finish, 1); \
    sample.valid = scenario != 0; \
    if (scenario != 3 || test_ms <= 30) sample.last_ms = test_ms; \
    if (scenario != 4 || test_ms <= 30) obstacle.measured_ms = test_ms; \
} while (0)
#define key1_pressed() (scenario < 6 ? test_ms == 20 : scenario == 6 ? test_ms == 1080 : test_ms == 30)
#define key2_pressed() (scenario == 6 && test_ms >= 20 && test_ms <= 1020)
#define key3_pressed() ((scenario == 5 && test_ms == 30) || \
    (scenario == 6 && (test_ms == 1050 || (test_ms >= 1100 && test_ms <= 2100))))
#define key4_pressed() (scenario == 7 && test_ms >= 20 && test_ms <= 100)
#define imu_state() (&sample)
#define imu_yaw() 0.0f
#define imu_gyro_z() 0.0f
#define obstacle_state() (&obstacle)
#define encoder_reset_counts() ((void)0)
#define encoder_counts(left, right) (*(left) = 0, *(right) = 0)
#define encoder_left_delta() 0
#define encoder_right_delta() 0
#define motor_set(left, right) (last_left = (left), last_right = (right))
#define oled_status(car, o, yaw) (last_car = *(car), was_running += (car)->running)
#define main firmware_main
#include "../main.c"
#undef main

/******************************************************************
 * 函 数 名 称：main
 * 函 数 说 明：验证启动门槛、惯导与测距超时、停止键和设置页刹车
 * 函 数 形 参：无
 * 函 数 返 回：零表示全部场景通过
 ******************************************************************/
int main(void)
{
    for (scenario = 0; scenario < 8; ++scenario) {
        test_ms = was_running = 0;
        sample = (JY61P){0}; last_car = (CarControl){0};
        obstacle = (ObstacleState){.sonar_result = SONAR_NO_ECHO};
        if (scenario == 1) obstacle.sonar_result = SONAR_PENDING;
        if (scenario == 2) obstacle.sonar_result = SONAR_FAULT;
        if (!setjmp(finish)) firmware_main();
        assert(!last_car.running && last_left == 0 && last_right == 0);
        if (scenario == 0 || scenario == 3) assert(last_car.stop_reason == STOP_IMU);
        if (scenario == 1 || scenario == 2 || scenario == 4) assert(last_car.stop_reason == STOP_SENSOR);
        if (scenario == 5) assert(last_car.stop_reason == STOP_KEY);
        if (scenario >= 3 && scenario <= 5) assert(was_running > 0);
        else assert(was_running == 0);                     /* 启动无效或设置期间电机始终刹车 */
        if (scenario == 6) assert(!last_car.setting && last_car.target == 16 && last_car.distance_m == 1);
        if (scenario == 7) assert(last_car.sensor_test && !last_car.setting); /* 长按 KEY4 不反复切页，检测期间 KEY1 不启动 */
    }
    puts("main integration self-check: PASS");
    return 0;
}
