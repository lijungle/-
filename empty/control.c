#include "control.h"
#include <math.h>

#define ROUTE_DEFAULT_COUNTS_PER_METER 3140U              /* 上电默认每米编码器计数 */
#define ROUTE_BRAKE_MS 300U                               /* 直行结束后先刹车，再原地转向 */
#define ROUTE_TURN_TIMEOUT_MS 10000U
#define TURN_TOLERANCE_DEG 2.0f
#define TURN_STABLE_MS 200U
#define TURN_MIN_SPEED 2.0f                               /* 单位为每 10 ms 编码器计数，需实车调整 */
#define TURN_MAX_SPEED 8.0f
#define SONAR_AXLE_OFFSET_M 0.14f                          /* 待实测：超声波探头到驱动轮轴中点的前向距离 */
#define OBSTACLE_LENGTH_M 0.28f
#define REAR_CLEARANCE_M 0.14f                             /* 车尾到轮轴中点距离，待核对 */
#define BYPASS_MARGIN_M 0.10f
#define DETOUR_OFFSET_M 0.50f
#define RAD_PER_DEG 0.01745329252f

/******************************************************************
 * 函 数 名 称：clampf
 * 函 数 说 明：将浮点数限制在指定上下限内
 * 函 数 形 参：x - 输入值；lo - 下限；hi - 上限
 * 函 数 返 回：限幅后的数值
 ******************************************************************/
static float clampf(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }

/******************************************************************
 * 函 数 名 称：pid_reset
 * 函 数 说 明：清除 PID 积分和上一次误差
 * 函 数 形 参：p - 待复位的 PID 状态
 * 函 数 返 回：无
 * 备       注：保留增益及限幅参数，启停时调用
 ******************************************************************/
void pid_reset(PID *p) { p->integral = 0.0f; p->previous = 0.0f; }

/******************************************************************
 * 函 数 名 称：pid_step
 * 函 数 说 明：执行一次位置式 PID 计算并限制输出范围
 * 函 数 形 参：p - PID 状态；target - 目标值；measured - 测量值
 *             dt - 计算周期，单位为秒，必须大于零
 * 函 数 返 回：限幅后的控制输出
 * 备       注：积分状态和输出共用 limit 的数值限制
 ******************************************************************/
float pid_step(PID *p, float target, float measured, float dt) {
    float e = target - measured;
    p->integral = clampf(p->integral + e * dt, -p->limit, p->limit); /* 积分限幅，避免持续累积 */
    float out = p->kp * e + p->ki * p->integral + p->kd * (e - p->previous) / dt; /* 合成比例、积分和微分项 */
    p->previous = e;                                        /* 保存误差，供下周期微分使用 */
    return clampf(out, -p->limit, p->limit);                /* 将电机输出限制到允许范围 */
}

/******************************************************************
 * 函 数 名 称：wrap_angle
 * 函 数 说 明：将角度折返到 -180~180 度，得到最短转向误差
 * 函 数 形 参：x - 有限角度值，单位为度
 * 函 数 返 回：折返后的角度
 ******************************************************************/
float wrap_angle(float x) {
    while (x > 180.0f) x -= 360.0f;                         /* 越过正边界时减去一整圈 */
    while (x < -180.0f) x += 360.0f;                        /* 越过负边界时加上一整圈 */
    return x;
}

/******************************************************************
 * 函 数 名 称：fuzzy_index
 * 函 数 说 明：把归一化输入量化为七级模糊规则表索引
 * 函 数 形 参：x - 归一化误差或误差变化量
 * 函 数 返 回：0~6 的规则表索引
 * 备       注：使用最近等级的离散量化，不进行隶属度插值
 ******************************************************************/
static int fuzzy_index(float x) {
    x = clampf(x, -3.0f, 3.0f);                             /* 限制输入，防止规则表越界 */
    return (int)floorf(x + 3.5f);                           /* 将 -3~3 七个等级映射为 0~6 */
}

/******************************************************************
 * 函 数 名 称：fuzzy_heading
 * 函 数 说 明：按航向误差和角速度调整增益，计算左右轮差速修正
 * 函 数 形 参：c - 小车控制状态；heading - 当前航向，单位为度
 *             gyro - Z 轴角速度，单位为度每秒；dt - 周期，单位为秒
 *             correction - 输出差速修正量的地址
 * 函 数 返 回：折返后的航向误差，单位为度
 * 备       注：修正量单位与轮速目标相同，为每 10 ms 编码器计数
 ******************************************************************/
float fuzzy_heading(CarControl *c, float heading, float gyro, float dt, float *correction) {
    float e = wrap_angle(c->heading_target - heading);       /* 使用跨越正负 180 度后的最短误差 */
    float de = -gyro;                                       /* 目标航向固定，误差变化率取角速度的负值 */
    static const int8_t rule[7][7] = {
        {-3,-3,-2,-2,-1,-1,0}, {-3,-2,-2,-1,-1,0,1}, {-2,-2,-1,-1,0,1,1},
        {-2,-1,-1,0,1,1,2}, {-1,-1,0,1,1,2,2}, {-1,0,1,1,2,2,3}, {0,1,1,2,2,3,3}
    };
    int r = rule[fuzzy_index(e * 0.08f)][fuzzy_index(de * 0.04f)]; /* 缩放误差和变化率后查询七级规则表 */
    float kp = 1.2f + 0.18f * r;                             /* 按规则结果调整比例增益 */
    float kd = 0.04f + 0.012f * (float)(r < 0 ? -r : r);     /* 按规则结果绝对值调整阻尼增益 */
    c->heading_integral = clampf(c->heading_integral + e * dt, -30.0f, 30.0f); /* 航向积分单独限幅 */
    *correction = clampf(kp * e + kd * de + 0.02f * c->heading_integral, -6.0f, 6.0f); /* 限制左右轮目标的差速修正 */
    return e;
}

/******************************************************************
 * 函 数 名 称：control_init
 * 函 数 说 明：初始化左右轮 PID、速度目标和航向控制状态
 * 函 数 形 参：c - 小车控制状态
 * 函 数 返 回：无
 * 备       注：速度目标为每 10 ms 计数，输出为百分比；参数需实车标定
 ******************************************************************/
void control_init(CarControl *c) {
    *c = (CarControl){0};                                  /* 包括路线阶段及计数基准，上电不自动启动 */
    c->left = (PID){0.8f, 0.15f, 0, 0, 0, 100};             /* 关闭速度微分项，减少编码器跳变引起的抖动 */
    c->right = (PID){0.8f, 0.15f, 0, 0, 0, 100};
    c->target = 15.0f;
    c->distance_m = 1;                                     /* 默认直行终点在起点前方一米 */
    c->counts_per_meter = ROUTE_DEFAULT_COUNTS_PER_METER;  /* 计数设置断电恢复默认值 */
}

/******************************************************************
 * 函 数 名 称：control_start
 * 函 数 说 明：记录当前位置和航向，启动当前选择的路线控制
 * 函 数 形 参：c - 小车控制状态；heading - 启动时航向，单位为度
 *             left_total/right_total - 当前累计计数；now_ms - 当前毫秒数
 * 函 数 返 回：无
 ******************************************************************/
void control_start(CarControl *c, float heading, int32_t left_total, int32_t right_total, uint32_t now_ms) {
    c->heading_target = wrap_angle(heading);
    c->start_heading = c->previous_heading = c->heading_target;
    c->heading_integral = 0;
    c->previous_left = left_total; c->previous_right = right_total;
    c->x = c->y = c->motion_distance = c->square_distance = 0; /* 建立路线起点坐标和边计数 */
    c->phase_ms = c->motion_ms = now_ms;
    c->aligned = c->tried_other = c->stop_reason = 0;
    c->square_edge = 0;
    c->phase = c->mode == MODE_SQUARE ? ROUTE_SQUARE_FORWARD : ROUTE_STRAIGHT;
    c->running = 1;
    pid_reset(&c->left); pid_reset(&c->right);
}

/******************************************************************
 * 函 数 名 称：control_stop
 * 函 数 说 明：取消运行标志并清除航向积分和左右轮 PID 历史状态
 * 函 数 形 参：c - 小车控制状态
 * 函 数 返 回：无
 * 备       注：只更新控制状态；实际刹车由主循环调用 motor_set 完成
 ******************************************************************/
void control_stop(CarControl *c) { c->running = 0; c->phase = ROUTE_IDLE; c->aligned = 0; c->heading_integral = 0; pid_reset(&c->left); pid_reset(&c->right); }

/******************************************************************
 * 函 数 名 称：control_keys
 * 函 数 说 明：处理启停、模式切换、距离速度和计数设置
 * 函 数 形 参：c - 控制状态；key1~key4 - 消抖后电平；now_ms - 毫秒数
 * 函 数 返 回：1 请求启动，0 不启动；有效性检查由主循环完成
 ******************************************************************/
int control_keys(CarControl *c, int key1, int key2, int key3, int key4, uint32_t now_ms) {
    uint8_t keys = (key1 ? 1U : 0U) | (key2 ? 2U : 0U) | (key3 ? 4U : 0U) | (key4 ? 8U : 0U);
    uint8_t pressed = keys & (uint8_t)~c->keys_last;
    uint8_t released = c->keys_last & (uint8_t)~keys;
    int start = 0;
    if (pressed & 2U) { c->key2_ms = now_ms; c->key2_long = c->running; }
    if (key3 && c->running) {
        control_stop(c); c->stop_reason = STOP_KEY;         /* 行驶时停止键优先，不能进入设置 */
    }
    if (c->sensor_test) {
        if (key3) c->sensor_test = 0;                      /* 检测页禁止启动和修改设置，停止键可以退出 */
        c->keys_last = keys;
        return 0;
    }
    if (c->setting) {
        if (key2 && now_ms - c->key2_ms >= 1000) {
            c->setting = SETTING_NONE;                     /* 设置页长按 KEY2 退出 */
            c->key2_long = 1;
        } else if ((released & 2U) && !c->key2_long) {
            c->setting = c->setting == SETTING_DISTANCE ? SETTING_SPEED :
                         c->setting == SETTING_SPEED ? SETTING_COUNTS : SETTING_DISTANCE;
        } else {
            if (c->setting == SETTING_DISTANCE) {
                if ((pressed & 1U) && c->distance_m < 10) ++c->distance_m;
                if ((pressed & 4U) && c->distance_m > 1) --c->distance_m;
            } else if (c->setting == SETTING_SPEED) {
                if ((pressed & 1U) && c->target < 30) c->target += 1;
                if ((pressed & 4U) && c->target > 5) c->target -= 1;
            } else {
                if ((pressed & 1U) && c->counts_per_meter < 10000) c->counts_per_meter += 10;
                if ((pressed & 4U) && c->counts_per_meter > 1000) c->counts_per_meter -= 10;
            }
        }
    } else if (!c->running && !key3) {
        if (key2 && !c->key2_long && now_ms - c->key2_ms >= 1000) {
            c->setting = SETTING_DISTANCE; c->key2_long = 1;
        } else if (pressed & 8U) {
            c->mode = c->mode == MODE_SQUARE ? MODE_STRAIGHT : MODE_SQUARE;
        } else if ((pressed & 1U) && !key2 && !key4) start = 1;
    }
    c->keys_last = keys;
    return start;
}

/******************************************************************
 * 函 数 名 称：route_enter
 * 函 数 说 明：切换路线阶段并清除控制历史与计时
 * 函 数 形 参：c - 状态；phase - 新阶段；now_ms - 毫秒数
 * 函 数 返 回：无
 ******************************************************************/
static void route_enter(CarControl *c, uint8_t phase, uint32_t now_ms) {
    c->phase = phase;
    c->phase_ms = c->motion_ms = now_ms;
    if (phase == ROUTE_SQUARE_FORWARD) c->square_distance = 0; /* 每条方形边重新计量 */
    c->aligned = 0; c->heading_integral = c->motion_distance = 0;
    pid_reset(&c->left); pid_reset(&c->right);               /* 阶段切换不继承上段速度积分 */
}

/******************************************************************
 * 函 数 名 称：route_brake
 * 函 数 说 明：先刹车三百毫秒再切换阶段
 * 函 数 形 参：c - 状态；next - 后续阶段；now_ms - 毫秒数
 * 函 数 返 回：无
 ******************************************************************/
static void route_brake(CarControl *c, uint8_t next, uint32_t now_ms) {
    c->next_phase = next;
    route_enter(c, ROUTE_BRAKE, now_ms);
}

/******************************************************************
 * 函 数 名 称：route_stop
 * 函 数 说 明：停车并记录可显示的原因
 * 函 数 形 参：c - 状态；reason - 停车原因
 * 函 数 返 回：无
 ******************************************************************/
static void route_stop(CarControl *c, uint8_t reason) {
    control_stop(c); c->stop_reason = reason;
}

/******************************************************************
 * 函 数 名 称：obstacle_blocked
 * 函 数 说 明：合并红外近障与超声波五十厘米阈值
 * 函 数 形 参：o - 同周期障碍快照
 * 函 数 返 回：1 被挡，0 未发现近障碍
 ******************************************************************/
static int obstacle_blocked(const ObstacleState *o) {
    return o->ir_left || o->ir_right || (o->sonar_result == SONAR_VALID && o->distance_mm <= 500);
}

/******************************************************************
 * 函 数 名 称：control_step
 * 函 数 说 明：更新局部位置并执行直行控制
 * 函 数 形 参：c - 状态；left_count/right_count - 每十毫秒轮速反馈
 *             left_total/right_total - 带方向累计计数；heading/gyro - 航向与角速度
 *             dt - 周期秒数；now_ms - 毫秒数；o - 障碍快照
 *             left_pwm/right_pwm - 百分比输出地址
 * 函 数 返 回：无
 * 备       注：正航向为左转；位置单位为米；所有切换周期输出均为零
 ******************************************************************/
void control_step(CarControl *c, float left_count, float right_count, int32_t left_total, int32_t right_total,
                  float heading, float gyro, float dt, uint32_t now_ms, const ObstacleState *o,
                  float *left_pwm, float *right_pwm) {
    *left_pwm = *right_pwm = 0;
    if (!c->running) return;
    if (!isfinite(heading) || !isfinite(gyro) || !isfinite(dt) || dt <= 0) {
        route_stop(c, STOP_IMU); return;
    }
    int32_t dl = (int32_t)((uint32_t)left_total - (uint32_t)c->previous_left);
    int32_t dr = (int32_t)((uint32_t)right_total - (uint32_t)c->previous_right);
    float ds = ((float)dl + (float)dr) * 0.5f / (float)c->counts_per_meter; /* 使用当前标定值换算中心位移 */
    float angle = wrap_angle(c->previous_heading + wrap_angle(heading - c->previous_heading) * 0.5f - c->start_heading); /* 用跨界中间航向投影 */
    c->x += ds * cosf(angle * RAD_PER_DEG);
    c->y += ds * sinf(angle * RAD_PER_DEG);                  /* 对称原地转向两轮增量相抵，不虚增位移 */
    c->previous_left = left_total; c->previous_right = right_total; c->previous_heading = heading;
    if (c->mode == MODE_STRAIGHT && c->x > c->distance_m + 0.05f) { route_stop(c, STOP_PATH); return; }
    if (c->phase == ROUTE_BRAKE) {
        if (now_ms - c->phase_ms >= ROUTE_BRAKE_MS) route_enter(c, c->next_phase, now_ms);
        return;
    }
    if (c->phase == ROUTE_CHECK) {
        if (now_ms - c->phase_ms < 60 || (int32_t)(o->measured_ms - c->phase_ms) < 0) return;
        if (obstacle_blocked(o)) {
            if (c->tried_other) { route_stop(c, STOP_BLOCKED); return; }
            c->detour_side = -c->detour_side; c->tried_other = 1;
            route_brake(c, ROUTE_TURN_OUT, now_ms);          /* 只尝试另一侧一次，不递归寻找出口 */
        } else {
            c->detour_start_x = c->x;
            route_enter(c, ROUTE_SHIFT_OUT, now_ms);
        }
        return;
    }
    int turning = c->phase == ROUTE_TURN_OUT || c->phase == ROUTE_TURN_FORWARD ||
                  c->phase == ROUTE_TURN_IN || c->phase == ROUTE_TURN_HOME;
    float left_target, right_target;
    if (turning) {
        if (now_ms - c->phase_ms >= ROUTE_TURN_TIMEOUT_MS) { route_stop(c, STOP_TURN_TIMEOUT); return; }
        float offset = c->mode == MODE_SQUARE ? -(float)(c->square_edge + 0U) * 90.0f :
                       c->phase == ROUTE_TURN_OUT ? c->detour_side * 90.0f :
                       c->phase == ROUTE_TURN_IN ? -c->detour_side * 90.0f : 0;
        c->heading_target = wrap_angle(c->start_heading + offset);
        float error = wrap_angle(c->heading_target - heading);
        if (fabsf(error) <= TURN_TOLERANCE_DEG) {
            pid_reset(&c->left); pid_reset(&c->right);
            if (fabsf(gyro) > 5.0f) { c->aligned = 0; return; }
            if (!c->aligned) { c->aligned = 1; c->aligned_ms = now_ms; }
            if (now_ms - c->aligned_ms >= TURN_STABLE_MS) {
                if (c->mode == MODE_SQUARE && c->phase == ROUTE_TURN_OUT) {
                    if (c->square_edge >= 4) { route_stop(c, STOP_DONE); return; }
                    route_enter(c, ROUTE_SQUARE_FORWARD, now_ms);
                    return;
                }
                uint8_t next = c->phase == ROUTE_TURN_OUT ? ROUTE_CHECK :
                               c->phase == ROUTE_TURN_FORWARD ? ROUTE_PASS :
                               c->phase == ROUTE_TURN_IN ? ROUTE_SHIFT_IN : ROUTE_STRAIGHT;
                if (c->phase == ROUTE_TURN_HOME && fabsf(c->y) > 0.05f) { route_stop(c, STOP_PATH); return; }
                route_enter(c, next, now_ms);
            }
            return;
        }
        c->aligned = 0;
        float speed = clampf(fabsf(error) * 0.08f, TURN_MIN_SPEED, TURN_MAX_SPEED);
        left_target = error < 0 ? speed : -speed;
        right_target = -left_target;                       /* 顺时针负角度：左轮前进、右轮后退 */
    } else {
        if (c->mode == MODE_SQUARE) {
            float remaining = (float)c->distance_m - c->square_distance;
            c->heading_target = wrap_angle(c->start_heading - (float)c->square_edge * 90.0f);
            if (remaining <= 0.05f) {
                if (c->square_edge >= 4) { route_stop(c, STOP_DONE); return; }
                ++c->square_edge; c->detour_side = -1;
                route_brake(c, ROUTE_TURN_OUT, now_ms);
                return;
            }
            c->square_distance += ds;
        } else if (obstacle_blocked(o)) {
            if (c->phase != ROUTE_STRAIGHT) { route_stop(c, STOP_BLOCKED); return; }
            float range = o->sonar_result == SONAR_VALID ? o->distance_mm / 1000.0f : 0.50f;
            c->bypass_x = c->x + range + SONAR_AXLE_OFFSET_M + OBSTACLE_LENGTH_M + REAR_CLEARANCE_M + BYPASS_MARGIN_M; /* 车尾越过障碍并留间隙后才返回 */
            if (c->bypass_x > c->distance_m) { route_stop(c, STOP_ENDPOINT); return; }
            c->detour_side = o->ir_right && !o->ir_left ? 1 : -1; /* 仅右红外触发时左绕，其余优先右绕 */
            c->tried_other = 0;
            route_brake(c, ROUTE_TURN_OUT, now_ms);
            return;
        }
        float remaining, cross, base;
        if (c->phase == ROUTE_SHIFT_OUT || c->phase == ROUTE_SHIFT_IN) {
            int direction = c->phase == ROUTE_SHIFT_OUT ? c->detour_side : -c->detour_side;
            float target_y = c->phase == ROUTE_SHIFT_OUT ? c->detour_side * DETOUR_OFFSET_M : 0;
            remaining = direction * (target_y - c->y);
            cross = -direction * (c->x - c->detour_start_x);
            base = direction * 90.0f;
        } else {
            remaining = (c->phase == ROUTE_PASS ? c->bypass_x : (float)c->distance_m) - c->x;
            cross = c->y - (c->phase == ROUTE_PASS ? c->detour_side * DETOUR_OFFSET_M : 0);
            base = 0;
        }
        float tolerance = c->phase == ROUTE_STRAIGHT ? 0.05f : 0.02f;
        if (c->mode == MODE_SQUARE) {
            remaining = (float)c->distance_m - c->square_distance;
            cross = 0;
            base = -(float)c->square_edge * 90.0f;
            tolerance = 0.05f;
        }
        if (c->mode != MODE_SQUARE && remaining <= tolerance) {
            if (fabsf(cross) > 0.05f) { route_stop(c, STOP_PATH); return; }
            if (c->phase == ROUTE_STRAIGHT) route_stop(c, STOP_DONE);
            else {
                uint8_t next = c->phase == ROUTE_SHIFT_OUT ? ROUTE_TURN_FORWARD :
                               c->phase == ROUTE_PASS ? ROUTE_TURN_IN : ROUTE_TURN_HOME;
                if (c->phase == ROUTE_PASS) c->detour_start_x = c->x;
                route_brake(c, next, now_ms);
            }
            return;
        }
        c->motion_distance += fabsf(ds);                  /* 独立累计本阶段位移，用于堵转保护 */
        if (c->motion_distance >= 0.005f) { c->motion_ms = now_ms; c->motion_distance = 0; }
        if (now_ms - c->motion_ms >= 2000) { route_stop(c, STOP_STALL); return; }
        float limit = c->phase == ROUTE_STRAIGHT ? c->target : fminf(c->target, 10.0f);
        if (o->sonar_result == SONAR_VALID && o->distance_mm < 1000) limit = fminf(limit, 10.0f);
        float speed = clampf(remaining * (float)c->counts_per_meter * 0.02f, 2.0f, limit);
        float correction;
        float trim = clampf(-atan2f(cross, 0.40f) / RAD_PER_DEG, -15.0f, 15.0f);
        c->heading_target = wrap_angle(c->start_heading + base + trim); /* 同时纠正横向偏移与航向 */
        fuzzy_heading(c, heading, gyro, dt, &correction);
        correction = clampf(correction, -speed * 0.5f, speed * 0.5f);
        left_target = speed - correction; right_target = speed + correction;
    }
    *left_pwm = pid_step(&c->left, left_target, left_count, dt);
    *right_pwm = pid_step(&c->right, right_target, right_count, dt);
}
