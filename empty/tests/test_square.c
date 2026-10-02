/* 主机路线自检：运行真实控制算法，不接真实电机。 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "../control.h"

static CarControl car;
static ObstacleState obstacle;
static uint32_t now;
static int32_t left, right;
static float yaw, lp, rp;

/******************************************************************
 * 函 数 名 称：step
 * 函 数 说 明：以指定时间间隔运行一次控制，传感器结果保持新鲜
 * 函 数 形 参：ms - 时间间隔；gyro - 当前角速度
 * 函 数 返 回：无
 ******************************************************************/
static void step(uint32_t ms, float gyro) {
    now += ms; obstacle.measured_ms = now;
    control_step(&car, 0, 0, left, right, yaw, gyro, ms / 1000.0f, now, &obstacle, &lp, &rp);
}

/******************************************************************
 * 函 数 名 称：start
 * 函 数 说 明：从任意计数和跨边界航向建立新任务
 * 函 数 形 参：meters - 目标米数
 * 函 数 返 回：无
 ******************************************************************/
static void start(unsigned meters) {
    control_init(&car); car.distance_m = (uint8_t)meters;
    left = 123; right = -45; now = 0; yaw = 170;
    obstacle = (ObstacleState){.sonar_result = SONAR_NO_ECHO};
    control_start(&car, yaw, left, right, now);
}

/******************************************************************
 * 函 数 名 称：turn
 * 函 数 说 明：模拟转到阶段目标并保持稳定
 * 函 数 形 参：无
 * 函 数 返 回：无
 ******************************************************************/
static void turn(void) {
    step(10, 0);
    yaw = car.heading_target;
    left += 500; right -= 500;                             /* 原地转向不产生车体中心位移 */
    step(10, 8); assert(!car.aligned && lp == 0 && rp == 0);
    step(10, 0); assert(car.aligned);
    step(199, 0); assert(car.aligned);
    step(1, 0);
}

/******************************************************************
 * 函 数 名 称：move
 * 函 数 说 明：在当前航向模拟一次指定米数的直线位移
 * 函 数 形 参：meters - 位移米数
 * 函 数 返 回：无
 ******************************************************************/
static void move(float meters) {
    int count = (int)roundf(meters * 3140);
    left += count; right += count;
    step(10, 0);
}

/******************************************************************
 * 函 数 名 称：press
 * 函 数 说 明：模拟一次完整短按并松开
 * 函 数 形 参：key - 一至三号按键
 * 函 数 返 回：无
 ******************************************************************/
static void press(int key) {
    assert(!control_keys(&car, key == 1, key == 2, key == 3, 0, ++now));
    assert(!control_keys(&car, 0, 0, 0, 0, ++now));
}

/******************************************************************
 * 函 数 名 称：main
 * 函 数 说 明：验证左右绕行、终点、保护、里程折返与按键设置
 * 函 数 形 参：无
 * 函 数 返 回：零表示全部断言通过
 ******************************************************************/
int main(void) {
    start(1);
    assert(car.target == 15 && car.left.kd == 0 && car.right.kd == 0);
    move(0.94f); assert(car.running);
    move(0.06f); assert(!car.running && car.stop_reason == STOP_DONE);
    assert(fabsf(car.x - 1) < 0.001f && fabsf(car.y) < 0.001f); /* 一米对应三千一百四十计数 */

    for (int side = -1; side <= 1; side += 2) {
        start(3);
        obstacle.ir_left = side < 0; obstacle.ir_right = side > 0;
        obstacle.sonar_result = SONAR_VALID; obstacle.distance_mm = 500;
        step(10, 0);
        assert(car.phase == ROUTE_BRAKE && car.detour_side == side && lp == 0 && rp == 0);
        float expected_x = car.bypass_x;
        assert(fabsf(expected_x - 1.16f) < 0.001f);
        step(299, 0); assert(car.phase == ROUTE_BRAKE);
        step(1, 0); assert(car.phase == ROUTE_TURN_OUT);
        step(10, 0); assert(side < 0 ? (lp > 0 && rp < 0) : (lp < 0 && rp > 0));
        obstacle.ir_left = obstacle.ir_right = 0;
        obstacle.sonar_result = SONAR_NO_ECHO;
        turn(); assert(car.phase == ROUTE_CHECK);
        step(60, 0); assert(car.phase == ROUTE_SHIFT_OUT);
        move(0.5f); assert(car.phase == ROUTE_BRAKE);
        assert(fabsf(car.x) < 0.001f && fabsf(car.y - side * 0.5f) < 0.001f);
        step(300, 0); turn(); assert(car.phase == ROUTE_PASS);
        move(expected_x); assert(car.phase == ROUTE_BRAKE);
        step(300, 0); turn(); assert(car.phase == ROUTE_SHIFT_IN);
        move(0.5f); assert(car.phase == ROUTE_BRAKE);
        step(300, 0); turn(); assert(car.phase == ROUTE_STRAIGHT);
        assert(fabsf(car.y) < 0.001f && fabsf(car.x - expected_x) < 0.001f);
        move(3 - car.x); assert(!car.running && car.stop_reason == STOP_DONE);
        assert(fabsf(car.x - 3) < 0.002f);                   /* 横移与转向路程没有提前结束三米任务 */
    }
    start(1); obstacle.ir_left = 1; step(10, 0);
    assert(car.stop_reason == STOP_ENDPOINT && !car.running);
    start(3); obstacle.sonar_result = SONAR_VALID; obstacle.distance_mm = 150; step(10, 0);
    assert(car.stop_reason == STOP_BLOCKED);
    start(3); obstacle.sonar_result = SONAR_FAULT; step(10, 0);
    assert(car.stop_reason == STOP_SENSOR);
    start(3); obstacle.sonar_result = SONAR_PENDING; step(10, 0);
    assert(car.stop_reason == STOP_SENSOR);
    start(3); obstacle.sonar_result = SONAR_VALID; obstacle.distance_mm = 900;
    step(10, 0); assert(lp > 0 && lp < 8.1f && rp > 0 && rp < 8.1f); /* 一米内限速十 */
    start(3); step(10, 0); assert(lp > 12 && rp > 12);      /* 无回波按未发现近障碍处理 */
    start(3); car.x = 3.06f; step(10, 0); assert(car.stop_reason == STOP_PATH);
    start(3); car.phase = ROUTE_TURN_HOME; car.y = 0.06f;
    turn(); assert(car.stop_reason == STOP_PATH);          /* 航向对齐但横向误差过大不能宣称回线完成 */
    start(3); yaw = NAN; step(10, 0); assert(car.stop_reason == STOP_IMU);
    start(3); step(2000, 0); assert(car.stop_reason == STOP_STALL);
    start(3); car.phase = ROUTE_TURN_OUT; car.detour_side = -1; step(10000, 0);
    assert(car.stop_reason == STOP_TURN_TIMEOUT);
    start(3); obstacle.ir_left = 1; step(10, 0); step(300, 0); turn();
    step(60, 0); assert(car.tried_other && car.detour_side == 1);
    step(300, 0); turn(); step(60, 0);
    assert(!car.running && car.stop_reason == STOP_BLOCKED); /* 两侧被挡只尝试一次另一侧 */
    start(3); car.phase = ROUTE_PASS; obstacle.ir_right = 1; step(10, 0);
    assert(car.stop_reason == STOP_BLOCKED);
    start(3); obstacle.measured_ms = 0;
    control_step(&car, 0, 0, left, right, yaw, 0, .01f, 201, &obstacle, &lp, &rp);
    assert(car.stop_reason == STOP_SENSOR && lp == 0 && rp == 0);
    start(3); left = right = INT32_MAX - 10;
    control_start(&car, 170, left, right, UINT32_MAX - 100);
    now = UINT32_MAX - 100; yaw = -170;
    left = right = (int32_t)((uint32_t)left + 20U);
    step(110, 0); assert(car.running && isfinite(car.x));   /* 时间、累计计数和航向均允许折返 */

    control_init(&car); now = 0;
    control_keys(&car, 0, 1, 0, 0, now);
    control_keys(&car, 0, 1, 0, 0, now = 1000);
    assert(car.setting == SETTING_DISTANCE && car.distance_m == 1);
    control_keys(&car, 0, 0, 0, 0, ++now);
    for (int i = 0; i < 15; ++i) press(1);
    assert(car.distance_m == 10 && !car.running);
    for (int i = 0; i < 15; ++i) press(2);
    assert(car.distance_m == 1);
    press(3); assert(car.setting == SETTING_SPEED);
    for (int i = 0; i < 40; ++i) press(1);
    assert(car.target == 30);
    for (int i = 0; i < 40; ++i) press(2);
    assert(car.target == 5);
    control_keys(&car, 0, 0, 1, 0, ++now);
    control_keys(&car, 0, 0, 1, 0, now += 1000);
    assert(!car.setting && !car.running);
    control_keys(&car, 0, 0, 0, 0, ++now);
    assert(control_keys(&car, 1, 0, 0, 0, ++now));
    assert(!control_keys(&car, 1, 0, 0, 0, ++now));             /* 长按不重复启动 */
    control_start(&car, 0, 0, 0, now);
    control_keys(&car, 0, 0, 1, 0, ++now);
    assert(!car.running && car.stop_reason == STOP_KEY);
    control_init(&car); car.sensor_test = 1;
    assert(!control_keys(&car, 1, 0, 0, 0, ++now));
    control_keys(&car, 0, 1, 0, 0, ++now);
    control_keys(&car, 0, 1, 0, 0, now += 1000);
    assert(car.sensor_test && !car.setting && !car.running);
    control_keys(&car, 0, 0, 1, 0, ++now);
    assert(!car.sensor_test && !car.running);               /* 检测页只能退出，不启动或进入设置 */
    puts("obstacle route self-check: PASS");
    return 0;
}
