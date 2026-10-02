/* 主机自检：用寄存器替身运行实际 board_glue.c，禁止连接真实电机。 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "../board_glue.h"
#define ti_msp_dl_config_h

typedef struct { uint32_t levels, pending; } GPIO_Regs;
static GPIO_Regs gpio_a, gpio_b;
static uint32_t compare_a[2], compare_g[2];
static uint32_t action_a[2], action_g[2];
typedef int GPTIMER_Regs;
typedef int DL_TIMER_CC_INDEX;
static GPTIMER_Regs timer_a, timer_g;
static uint16_t sonar_count;
static unsigned triggers;
static uint8_t rx_data[11], rx_pos;
#define GPIOA (&gpio_a)
#define GPIOB (&gpio_b)
#define IR_LEFT_PORT GPIOB
#define IR_RIGHT_PORT GPIOB
#define IR_LEFT_IR_LEFT_PIN_PIN (1U << 10)
#define IR_RIGHT_IR_RIGHT_PIN_PIN (1U << 11)
#define SONAR_TRIG_SONAR_TRIG_PIN_PIN (1U << 7)
#define SONAR_ECHO_SONAR_ECHO_PIN_PIN (1U << 27)
#define SONAR_ECHO_PORT GPIOA
#define SONAR_TRIG_PORT GPIOA
#define SONAR_TIMER_INST 8
#define CPUCLK_FREQ 32000000U
#define DL_GPIO_PIN_12 (1U << 12)
#define DL_GPIO_PIN_15 (1U << 15)
#define DL_GPIO_PIN_16 (1U << 16)
#define DL_GPIO_PIN_17 (1U << 17)
#define DL_GPIO_PIN_24 (1U << 24)
#define DL_GPIO_PIN_25 (1U << 25)
#define DL_GPIO_PIN_26 (1U << 26)
#define KEY1_PORT GPIOA
#define KEY1_KEY1_PIN_PIN DL_GPIO_PIN_26
#define KEY2_PORT GPIOA
#define KEY2_KEY2_PIN_PIN DL_GPIO_PIN_25
#define KEY3_PORT GPIOA
#define KEY3_KEY3_PIN_PIN DL_GPIO_PIN_12
#define KEY4_PORT GPIOB
#define KEY4_KEY4_PIN_PIN (1U << 21)
#define ENC_LEFT_A_ENC_LEFT_A_PIN_PIN DL_GPIO_PIN_17
#define ENC_LEFT_B_ENC_LEFT_B_PIN_PIN DL_GPIO_PIN_24
#define ENC_RIGHT_A_ENC_RIGHT_A_PIN_PIN DL_GPIO_PIN_15
#define ENC_RIGHT_B_ENC_RIGHT_B_PIN_PIN DL_GPIO_PIN_16
#define PWM_0_INST (&timer_a)
#define PWM_1_INST (&timer_g)
#define GPIO_PWM_0_C0_IDX 0
#define GPIO_PWM_0_C1_IDX 1
#define GPIO_PWM_1_C0_IDX 0
#define GPIO_PWM_1_C1_IDX 1
#define GPIOA_INT_IRQn 0
#define CONTROL_TIMER_INST_INT_IRQN 1
#define JY61P_UART_INST_INT_IRQN 2
#define CONTROL_TIMER_INST 0
#define JY61P_UART_INST 0
#define DL_TIMERG_IIDX_ZERO 1
#define DL_UART_MAIN_IIDX_RX 1
#define CONTROL_TIMER_INST_IRQHandler Test_Timer_IRQHandler
#define JY61P_UART_INST_IRQHandler Test_UART_IRQHandler

/* 以下宏模拟 DriverLib 和 Cortex-M 的寄存器访问，算法仍执行工程原函数。 */
#define SYSCFG_DL_init() ((void)0)
#define NVIC_ClearPendingIRQ(irq) ((void)(irq))
#define NVIC_EnableIRQ(irq) ((void)(irq))
#define DL_TimerA_startCounter(inst) ((void)(inst))
#define DL_TimerG_startCounter(inst) ((void)(inst))
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(mask) ((void)(mask))
#define DL_GPIO_readPins(port,pin) ((port)->levels & (pin))
#define DL_GPIO_setPins(port,pin) ((port)->levels |= (pin), ++triggers)
#define DL_GPIO_clearPins(port,pin) ((port)->levels &= ~(pin))
#define delay_cycles(cycles) ((void)(cycles))
#define DL_TimerG_getTimerCount(inst) ((void)(inst),sonar_count)
#define DL_GPIO_getEnabledInterruptStatus(port,mask) ((port)->pending & (mask))
#define DL_GPIO_clearInterruptStatus(port,mask) ((port)->pending &= ~(mask))
#define DL_TimerA_getLoadValue(inst) ((void)(inst),399U)
#define DL_TimerA_setCaptureCompareValue(inst,value,idx) ((void)(inst),compare_a[idx]=(value))
#define DL_TimerG_setCaptureCompareValue(inst,value,idx) ((void)(inst),compare_g[idx]=(value))
#define DL_TIMER_CC_LACT_CCP_HIGH 1U
#define DL_TIMER_CC_LACT_CCP_LOW 2U
#define DL_TIMER_CC_CDACT_CCP_HIGH 4U
#define DL_TIMER_CC_CDACT_CCP_LOW 8U
#define DL_Timer_setCaptureCompareAction(inst,value,idx) ((inst)==PWM_0_INST ? (action_a[idx]=(value)) : (action_g[idx]=(value)))
#define DL_Timer_setCaptureCompareValue(inst,value,idx) ((inst)==PWM_0_INST ? (compare_a[idx]=(value)) : (compare_g[idx]=(value)))
#define DL_TimerG_getPendingInterrupt(inst) ((void)(inst),DL_TIMERG_IIDX_ZERO)
#define DL_UART_Main_getPendingInterrupt(inst) ((void)(inst),DL_UART_MAIN_IIDX_RX)
#define DL_UART_Main_isRXFIFOEmpty(inst) ((void)(inst),rx_pos == sizeof(rx_data))
#define DL_UART_Main_receiveData(inst) ((void)(inst),rx_data[rx_pos++])
#define oled_init() ((void)0)
#define oled_poll() ((void)0)
#include "../board_glue.c"

/******************************************************************
 * 函 数 名 称：main
 * 函 数 说 明：验证实际板级函数的输出映射、反馈计数和惯导解析
 * 函 数 形 参：无
 * 函 数 返 回：0 表示通过；断言失败则终止
 ******************************************************************/
int main(void)
{
    assert(motor_compensate(0, 20) == 0);
    assert(fabsf(motor_compensate(1, 20) - 20.8f) < 0.001f);
    assert(motor_compensate(50, 20) == 60 && motor_compensate(-50, 20) == -60);
    assert(motor_compensate(200, 20) == 100 && motor_compensate(-200, 20) == -100);
    assert(motor_compensate(25, 0) == 25);                  /* 未标定时保持原占空比，零指令始终刹车 */
    assert(motor_compensate(0, LEFT_MOTOR_DEADZONE_PERCENT) == 0);
    assert(fabsf(motor_compensate(10, LEFT_MOTOR_DEADZONE_PERCENT) - 17.2f) < 0.001f);
    assert(fabsf(motor_compensate(-10, RIGHT_MOTOR_DEADZONE_PERCENT) + 17.2f) < 0.001f); /* 8% 补偿保留正反方向 */
    motor_set(0, 0);
    assert(action_a[0] == 10 && action_a[1] == 10);
    assert(action_g[0] == 5 && action_g[1] == 5);            /* 反相一路功能恒低，实际输出恒高 */
    assert(compare_g[0] == 0 && compare_g[1] == 0);           /* 刹车：四路同高 */
    motor_set(25, 50);
    assert(action_a[0] == 10 && compare_g[0] == 124);
    assert(compare_a[1] >= 184 && compare_a[1] <= 185 && compare_g[1] == 0); /* 补偿后右轮 54%，反相通道写入周期减有效计数 */
    motor_set(-25, -50);
    assert(compare_a[0] == 276 && compare_g[0] == 0);
    assert(action_a[1] == 10 && compare_g[1] >= 215 && compare_g[1] <= 216);
    motor_set(200, -200);
    assert(action_a[0] == 10 && action_g[0] == 10);
    assert(action_a[1] == 10 && action_g[1] == 10);           /* 超范围限幅，恒低端点不写超范围比较值 */
    motor_set(NAN, 50);
    assert(action_a[0] == 10 && action_a[1] == 10);
    assert(action_g[0] == 5 && action_g[1] == 5);
    assert(compare_g[0] == 0 && compare_g[1] == 0);

    gpio_a.levels = DL_GPIO_PIN_17;
    gpio_a.pending = DL_GPIO_PIN_17 | DL_GPIO_PIN_15;
    GROUP1_IRQHandler();
    assert(encoder_left_delta() == 1 && encoder_right_delta() == 1);
    assert(encoder_left_delta() == 0 && encoder_right_delta() == 0);
    gpio_a.levels = DL_GPIO_PIN_15;
    gpio_a.pending = DL_GPIO_PIN_17 | DL_GPIO_PIN_15;
    GROUP1_IRQHandler();
    assert(encoder_left_delta() == -1 && encoder_right_delta() == -1);
    assert(gpio_a.pending == 0);                            /* 不丢方向，读取增量后下一周期为零 */

    int32_t left_total, right_total;
    encoder_reset_counts();
    left_count -= 1234; right_count += 1250;
    encoder_counts(&left_total, &right_total);
    assert(left_total == 1234 && right_total == 1250);
    assert(encoder_left_delta() == 1234 && encoder_right_delta() == 1250);
    left_count -= 5; right_count += 6;
    encoder_reset_counts();
    encoder_counts(&left_total, &right_total);
    assert(left_total == 0 && right_total == 0);
    assert(encoder_left_delta() == 5 && encoder_right_delta() == 6); /* 显示清零不吞掉速度反馈 */
    left_zero = 2; left_count = UINT32_MAX - 2;
    right_zero = UINT32_MAX - 2; right_count = 2;
    encoder_counts(&left_total, &right_total);
    assert(left_total == 5 && right_total == 5);           /* 累计原始计数折返时仍保留正确位移 */

    gpio_a.levels = DL_GPIO_PIN_26 | DL_GPIO_PIN_25 | DL_GPIO_PIN_12;
    ticks = 0;
    assert(!key1_pressed() && !key2_pressed() && !key3_pressed());
    gpio_a.levels = 0;
    ticks = 10; assert(!key1_pressed() && !key2_pressed() && !key3_pressed());
    ticks = 20; assert(!key1_pressed() && !key2_pressed() && !key3_pressed());
    ticks = 30; assert(key1_pressed() && key2_pressed() && key3_pressed()); /* 三个按键均稳定 20 ms 才确认 */
    gpio_b.levels = KEY4_KEY4_PIN_PIN;
    assert(!key4_pressed());
    gpio_b.levels = 0;
    ticks = 40; assert(!key4_pressed());
    ticks = 50; assert(!key4_pressed());
    ticks = 60; assert(key4_pressed());                     /* PB21 按键独立消抖，不能复用 KEY3 状态 */

    rx_data[0] = 0x55; rx_data[1] = 0x53;
    rx_data[6] = 0; rx_data[7] = 0x40;                     /* 协议第 7、8 字节为航向低、高位，数组下标为 6、7 */
    rx_data[8] = 0x12; rx_data[9] = 0x34;                 /* 尾部字段不能混入航向值 */
    for (unsigned i = 0; i < 10; ++i) rx_data[10] += rx_data[i];
    ticks = 100;
    Test_UART_IRQHandler();
    board_poll();
    assert(imu_yaw() == 90 && !jy61p_timeout(imu_state(), 400));
    assert(jy61p_timeout(imu_state(), 401));
    rx_data[10] ^= 1; rx_pos = 0;
    ticks = 500; Test_UART_IRQHandler(); board_poll();
    assert(imu_state()->last_ms == 100);                    /* 错误校验帧不能延长有效数据时间 */
    rx_data[1] = 0x52; rx_data[7] = 0x10;
    rx_data[10] = 0; rx_pos = 0;
    for (unsigned i = 0; i < 10; ++i) rx_data[10] += rx_data[i];
    Test_UART_IRQHandler(); board_poll();
    assert(imu_gyro_z() == 250 && imu_yaw() == 90);
    assert(imu_state()->last_ms == 100);                    /* 角速度帧尾部温度不能混入 Z 轴，也不延长角度有效时间 */
    rx_data[1] = 0x53; rx_data[7] = 0xC0;
    rx_data[10] = 0; rx_pos = 0;
    for (unsigned i = 0; i < 10; ++i) rx_data[10] += rx_data[i];
    Test_UART_IRQHandler(); board_poll();
    assert(imu_yaw() == -90);                              /* 有符号小端数值必须保留负航向 */
    Test_Timer_IRQHandler();
    assert(board_millis() == 510);

    /* 清空传感器替身，按真实下降计数和双沿中断推进测量。 */
    echo_phase = sonar_triggered = 0;
    obstacle = (ObstacleState){0};
    ir_raw_last[0] = ir_raw_last[1] = ir_stable[0] = ir_stable[1] = 0;
    ticks = 1000; sonar_count = 65535;
    gpio_a.levels = 0;
    gpio_b.levels = IR_LEFT_IR_LEFT_PIN_PIN | IR_RIGHT_IR_RIGHT_PIN_PIN; /* A 端口反向电平不能影响 B 端口红外 */
    obstacle_poll();
    unsigned first_trigger = triggers;
    assert(echo_phase == 1 && obstacle.sonar_result == SONAR_PENDING);
    gpio_a.levels |= SONAR_ECHO_SONAR_ECHO_PIN_PIN;
    gpio_a.pending = SONAR_ECHO_SONAR_ECHO_PIN_PIN;
    sonar_count = 65000; GROUP1_IRQHandler();
    gpio_a.levels &= ~SONAR_ECHO_SONAR_ECHO_PIN_PIN;
    gpio_a.pending = SONAR_ECHO_SONAR_ECHO_PIN_PIN;
    sonar_count -= 5800; GROUP1_IRQHandler();
    ticks = 1010; obstacle_poll();
    assert(obstacle.sonar_result == SONAR_VALID && obstacle.distance_mm == 1000);
    assert(obstacle.measured_ms == 1010 && echo_phase == 0);
    assert(obstacle.echo_stage == 2 && obstacle.echo_us == 5800 && obstacle.sonar_triggers == 1);
    assert(obstacle.ir_left_level == 1 && obstacle.ir_right_level == 1);
    ticks = 1050; obstacle_poll(); assert(triggers == first_trigger);
    ticks = 1060; sonar_count = 2000; obstacle_poll();
    assert(triggers == first_trigger + 1);
    sonar_count = 1000; gpio_a.levels |= SONAR_ECHO_SONAR_ECHO_PIN_PIN;
    gpio_a.pending = SONAR_ECHO_SONAR_ECHO_PIN_PIN; GROUP1_IRQHandler();
    sonar_count = (uint16_t)(1000 - 2900);
    gpio_a.levels &= ~SONAR_ECHO_SONAR_ECHO_PIN_PIN;
    gpio_a.pending = SONAR_ECHO_SONAR_ECHO_PIN_PIN; GROUP1_IRQHandler();
    ticks = 1070; obstacle_poll();
    assert(obstacle.sonar_result == SONAR_VALID && obstacle.distance_mm == 500); /* 定时器折返仍可测宽 */
    ticks = 1120; sonar_count = 30000; obstacle_poll();
    ticks = 1140; sonar_count = 5001; obstacle_poll(); assert(echo_phase == 1);
    ticks = 1150; sonar_count = 5000; obstacle_poll();
    assert(obstacle.sonar_result == SONAR_NO_ECHO && obstacle.measured_ms == 1150);
    assert(obstacle.echo_stage == 0 && obstacle.echo_us == 0);
    ticks = 1180; sonar_count = 30000; obstacle_poll();
    gpio_a.levels |= SONAR_ECHO_SONAR_ECHO_PIN_PIN;
    gpio_a.pending = SONAR_ECHO_SONAR_ECHO_PIN_PIN; GROUP1_IRQHandler();
    ticks = 1210; sonar_count = 5000; obstacle_poll();
    assert(obstacle.sonar_result == SONAR_FAULT);           /* 上升后持续高电平不能当作无障碍 */
    ticks = 1240; obstacle_poll(); assert(echo_phase == 0 && obstacle.sonar_result == SONAR_FAULT);
    gpio_a.levels &= ~SONAR_ECHO_SONAR_ECHO_PIN_PIN;
    gpio_b.levels &= ~IR_LEFT_IR_LEFT_PIN_PIN;
    ticks = 1250; obstacle_poll(); assert(!obstacle.ir_left);
    ticks = 1260; obstacle_poll(); assert(!obstacle.ir_left);
    ticks = 1270; obstacle_poll(); assert(obstacle.ir_left && !obstacle.ir_right);
    gpio_b.levels |= IR_LEFT_IR_LEFT_PIN_PIN;
    gpio_b.levels &= ~IR_RIGHT_IR_RIGHT_PIN_PIN;
    ticks = 1280; obstacle_poll();
    ticks = 1290; obstacle_poll(); assert(obstacle.ir_left && !obstacle.ir_right);
    ticks = 1300; obstacle_poll(); assert(!obstacle.ir_left && obstacle.ir_right);
    assert(obstacle.ir_left_level == 1 && obstacle.ir_right_level == 0);

    /* 不合法完整脉冲和遗漏下降沿都必须报故障，不能按没有障碍放行。 */
    echo_phase = 0; sonar_triggered = 0;
    ticks = 1400; sonar_count = 30000; obstacle_poll();
    gpio_a.levels |= SONAR_ECHO_SONAR_ECHO_PIN_PIN;
    gpio_a.pending = SONAR_ECHO_SONAR_ECHO_PIN_PIN; GROUP1_IRQHandler();
    sonar_count -= 100; gpio_a.levels &= ~SONAR_ECHO_SONAR_ECHO_PIN_PIN;
    gpio_a.pending = SONAR_ECHO_SONAR_ECHO_PIN_PIN; GROUP1_IRQHandler();
    ticks = 1410; obstacle_poll();
    assert(obstacle.sonar_result == SONAR_FAULT && obstacle.echo_stage == 2 && obstacle.echo_us == 100);
    ticks = 1460; sonar_count = 30000; obstacle_poll();
    gpio_a.levels |= SONAR_ECHO_SONAR_ECHO_PIN_PIN;
    gpio_a.pending = SONAR_ECHO_SONAR_ECHO_PIN_PIN; GROUP1_IRQHandler();
    gpio_a.levels &= ~SONAR_ECHO_SONAR_ECHO_PIN_PIN;
    ticks = 1490; sonar_count = 5000; obstacle_poll();
    assert(obstacle.sonar_result == SONAR_FAULT && obstacle.echo_stage == 1);
    puts("board self-check: PASS");
    return 0;
}
