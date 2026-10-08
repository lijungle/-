#include "board_glue.h"
#include "ti_msp_dl_config.h"
#include <math.h>

/* 按当前电机与编码器安装方向，将前进反馈统一换算为正。 */
#define LEFT_ENCODER_SIGN  (-1)
#define RIGHT_ENCODER_SIGN 1
/* 按用户要求暂设左右死区为 8%，尚未实测；后续可分别调整。 */
#define LEFT_MOTOR_DEADZONE_PERCENT  8.0f
#define RIGHT_MOTOR_DEADZONE_PERCENT 8.0f
static JY61P imu_rx;
static JY61P imu;
static GPSState gps;
static ESPLink esp;
static volatile uint32_t ticks;
static volatile uint32_t left_count, right_count;
static uint32_t left_previous, right_previous;
static uint32_t left_zero, right_zero;
static ObstacleState obstacle = { .sonar_result = SONAR_NO_ECHO };

/******************************************************************
 * 函 数 名 称：board_init
 * 函 数 说 明：初始化外设，先刹车再启动 PWM 和编码器中断
 * 函 数 形 参：无
 * 函 数 返 回：无
 ******************************************************************/
void board_init(void)
{
    jy61p_init(&imu_rx);
    jy61p_init(&imu);
    gps_init(&gps);
    esp_init(&esp);
    SYSCFG_DL_init();                                  /* 外设配置全部由 SysConfig 生成 */
    motor_set(0, 0);                                   /* 启动计数器前设置两输入同高刹车 */
    DL_TimerA_startCounter(PWM_0_INST);
    DL_TimerG_startCounter(PWM_1_INST);                 /* 参考工程的第二组 PWM 默认未启动 */
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);                    /* 仅启用编码器 GPIOA 中断 */
    NVIC_ClearPendingIRQ(CONTROL_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(CONTROL_TIMER_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(JY61P_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(JY61P_UART_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(GPS_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(GPS_UART_INST_INT_IRQN);              /* GPS 接收只启用 UART1 RX 中断 */
    NVIC_ClearPendingIRQ(ESP_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(ESP_UART_INST_INT_IRQN);
    oled_init();                                      /* 初始化时电机保持刹车 */
}

/******************************************************************
 * 函 数 名 称：board_poll
 * 函 数 说 明：取得惯导快照并推进 OLED 刷新
 * 函 数 形 参：无
 * 函 数 返 回：无
 ******************************************************************/
void board_poll(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    imu = imu_rx;                                     /* 防止主循环读到半更新的惯导状态 */
    __set_PRIMASK(mask);
    gps_poll(&gps, ticks);                             /* 限量解析，不能阻塞十毫秒控制节拍 */
    oled_poll();
}

/******************************************************************
 * 函 数 名 称：board_millis
 * 函 数 说 明：读取定时器累计毫秒数
 * 函 数 形 参：无
 * 函 数 返 回：毫秒数，分辨率为 10 ms
 ******************************************************************/
uint32_t board_millis(void) { return ticks; }

/******************************************************************
 * 函 数 名 称：key_debounce
 * 函 数 说 明：读取低有效按键，连续稳定 20 ms 后更新状态
 * 函 数 形 参：port/pin - 按键引脚；idx - 状态数组索引
 * 函 数 返 回：1 按下，0 松开
 ******************************************************************/
static int key_debounce(GPIO_Regs *port, uint32_t pin, unsigned idx)
{
    static uint8_t raw_last[4], stable[4];
    static uint32_t changed_ms[4];
    uint8_t raw = DL_GPIO_readPins(port, pin) == 0;
    uint32_t now = ticks;
    if (raw != raw_last[idx]) {
        raw_last[idx] = raw;
        changed_ms[idx] = now;                         /* 电平变化后重新计算消抖时间 */
    }
    if (now - changed_ms[idx] >= 20) stable[idx] = raw;
    return stable[idx];
}

/******************************************************************
 * 函 数 名 称：key1_pressed
 * 函 数 说 明：读取 PA26 启动键
 * 函 数 形 参：无
 * 函 数 返 回：消抖后的按下状态
 ******************************************************************/
int key1_pressed(void) { return key_debounce(KEY1_PORT, KEY1_KEY1_PIN_PIN, 0); }

/******************************************************************
 * 函 数 名 称：key2_pressed
 * 函 数 说 明：读取 PA25 参数设置键
 * 函 数 形 参：无
 * 函 数 返 回：消抖后的按下状态
 ******************************************************************/
int key2_pressed(void) { return key_debounce(KEY2_PORT, KEY2_KEY2_PIN_PIN, 2); }

/******************************************************************
 * 函 数 名 称：key3_pressed
 * 函 数 说 明：读取用户确认的 PA12 停止键
 * 函 数 形 参：无
 * 函 数 返 回：消抖后的按下状态
 ******************************************************************/
int key3_pressed(void) { return key_debounce(KEY3_PORT, KEY3_KEY3_PIN_PIN, 1); }

/******************************************************************
 * 函 数 名 称：key4_pressed
 * 函 数 说 明：读取 PB21 模式切换键
 * 函 数 形 参：无
 * 函 数 返 回：消抖后的按下状态
 ******************************************************************/
int key4_pressed(void) { return key_debounce(KEY4_PORT, KEY4_KEY4_PIN_PIN, 3); }

/******************************************************************
 * 函 数 名 称：motor_channel
 * 函 数 说 明：设置一路高电平占空比，并单独处理恒高和恒低端点
 * 函 数 形 参：timer/index - 定时器通道；high/period - 高电平计数/周期；invert - 是否反相
 * 函 数 返 回：无
 * 备       注：比较值大于 LOAD 不会触发比较事件，不能用它模拟反相端点
 ******************************************************************/
static void motor_channel(GPTIMER_Regs *timer, DL_TIMER_CC_INDEX index,
    uint32_t high, uint32_t period, int invert)
{
    uint32_t action = DL_TIMER_CC_LACT_CCP_HIGH | DL_TIMER_CC_CDACT_CCP_LOW;
    uint32_t compare = invert ? high : period - high;
    if (high == 0 || high == period) {
        int function_high = (high == period) != invert;
        action = function_high ?
            DL_TIMER_CC_LACT_CCP_HIGH | DL_TIMER_CC_CDACT_CCP_HIGH :
            DL_TIMER_CC_LACT_CCP_LOW | DL_TIMER_CC_CDACT_CCP_LOW;
        compare = 0;                                       /* 固定动作确保刹车端点没有 PWM 窄脉冲 */
    }
    DL_Timer_setCaptureCompareAction(timer, action, index);
    DL_Timer_setCaptureCompareValue(timer, compare, index);
}

/******************************************************************
 * 函 数 名 称：motor_compensate
 * 函 数 说 明：把非零控制指令映射到电机有效占空比区间
 * 函 数 形 参：output - 带方向的百分比指令；deadzone - 起转补偿占空比
 * 函 数 返 回：补偿后的百分比，占空比幅值不超过 100
 * 备       注：零指令仍为刹车；死区为零时不改变指令
 ******************************************************************/
static float motor_compensate(float output, float deadzone)
{
    if (output == 0) return 0;                             /* 停止时不能因补偿产生起转输出 */
    float magnitude = fabsf(output);
    if (magnitude > 100) magnitude = 100;
    magnitude = deadzone + (100 - deadzone) * magnitude / 100; /* 将幅值映射到设定死区至满占空比 */
    return output < 0 ? -magnitude : magnitude;
}

/******************************************************************
 * 函 数 名 称：motor_set
 * 函 数 说 明：按参考工程的 AT8236 慢衰减方式设置四路 PWM
 * 函 数 形 参：left/right - 左右轮百分比，范围 -100~100
 * 函 数 返 回：无
 * 备       注：左轮前进对应 AO_Control(0)，右轮前进对应 BO_Control(1)
 ******************************************************************/
void motor_set(float left, float right)
{
    if (!isfinite(left) || !isfinite(right)) left = right = 0; /* 无效输出统一刹车 */
    left = motor_compensate(left, LEFT_MOTOR_DEADZONE_PERCENT);
    right = motor_compensate(right, RIGHT_MOTOR_DEADZONE_PERCENT);
    uint32_t period = DL_TimerA_getLoadValue(PWM_0_INST) + 1;
    uint32_t l = (uint32_t)(fabsf(left) * (float)period / 100);
    uint32_t r = (uint32_t)(fabsf(right) * (float)period / 100);
    motor_channel(PWM_0_INST, GPIO_PWM_0_C0_IDX,
        left >= 0 ? period : period - l, period, 1);        /* PB4 反相 PWM */
    motor_channel(PWM_1_INST, GPIO_PWM_1_C0_IDX,
        left >= 0 ? period - l : period, period, 0);        /* PA23 非反相 PWM */
    motor_channel(PWM_0_INST, GPIO_PWM_0_C1_IDX,
        right >= 0 ? period - r : period, period, 1);       /* PB1，右轮机械安装方向与左轮相反 */
    motor_channel(PWM_1_INST, GPIO_PWM_1_C1_IDX,
        right >= 0 ? period : period - r, period, 0);       /* PA18；零输出时四路均为高电平刹车 */
}

/******************************************************************
 * 函 数 名 称：encoder_left_delta
 * 函 数 说 明：读取并计算左轮累计计数的周期增量
 * 函 数 形 参：无
 * 函 数 返 回：带方向的编码器增量
 ******************************************************************/
int encoder_left_delta(void)
{
    uint32_t now = left_count;
    int32_t delta = (int32_t)(now - left_previous);
    left_previous = now;
    return LEFT_ENCODER_SIGN * delta;                       /* 无符号累计计数允许自然折返 */
}

/******************************************************************
 * 函 数 名 称：encoder_right_delta
 * 函 数 说 明：读取并计算右轮累计计数的周期增量
 * 函 数 形 参：无
 * 函 数 返 回：带方向的编码器增量
 ******************************************************************/
int encoder_right_delta(void)
{
    uint32_t now = right_count;
    int32_t delta = (int32_t)(now - right_previous);
    right_previous = now;
    return RIGHT_ENCODER_SIGN * delta;
}

/******************************************************************
 * 函 数 名 称：encoder_counts
 * 函 数 说 明：读取从任务零点开始的左右轮带方向累计计数
 * 函 数 形 参：left/right - 累计计数输出地址
 * 函 数 返 回：无
 * 备       注：前进为正；不改变速度闭环使用的上次读数
 ******************************************************************/
void encoder_counts(int32_t *left, int32_t *right)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *left = (int32_t)((left_count - left_zero) * (uint32_t)LEFT_ENCODER_SIGN);
    *right = (int32_t)((right_count - right_zero) * (uint32_t)RIGHT_ENCODER_SIGN);
    __set_PRIMASK(mask);                                   /* 同一快照中的左右计数使用各自前进方向 */
}

/******************************************************************
 * 函 数 名 称：encoder_reset_counts
 * 函 数 说 明：记录任务累计计数零点，用于建立路线起点
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：只更改累计计数零点，不清除中断累计值和速度增量历史
 ******************************************************************/
void encoder_reset_counts(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    left_zero = left_count;
    right_zero = right_count;
    __set_PRIMASK(mask);                                   /* 防止读取两侧零点期间被编码器中断打断 */
}

/******************************************************************
 * 函 数 名 称：imu_yaw
 * 函 数 说 明：读取主循环惯导快照中的航向
 * 函 数 形 参：无
 * 函 数 返 回：航向角，单位为度
 * 备       注：快照由 board_poll 更新；有效性由 jy61p_timeout 判断
 ******************************************************************/
float imu_yaw(void) { return imu.yaw; }

/******************************************************************
 * 函 数 名 称：imu_gyro_z
 * 函 数 说 明：读取主循环惯导快照中的 Z 轴角速度
 * 函 数 形 参：无
 * 函 数 返 回：Z 轴角速度，单位为度每秒
 ******************************************************************/
float imu_gyro_z(void) { return imu.gyro_z; }

/******************************************************************
 * 函 数 名 称：imu_state
 * 函 数 说 明：取得主循环惯导快照地址，供有效性和超时检查使用
 * 函 数 形 参：无
 * 函 数 返 回：惯导快照地址
 * 备       注：串口中断更新独立接收状态，board_poll 负责复制快照
 ******************************************************************/
JY61P *imu_state(void) { return &imu; }
const GPSState *gps_state(void) { return &gps; }

/******************************************************************
 * 函 数 名 称：GROUP1_IRQHandler
 * 函 数 说 明：处理编码器 A 相双沿计数
 * 函 数 形 参：无
 * 函 数 返 回：无
 ******************************************************************/
void GROUP1_IRQHandler(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(GPIOA,
        ENC_LEFT_A_ENC_LEFT_A_PIN_PIN | ENC_RIGHT_A_ENC_RIGHT_A_PIN_PIN);
    if (pending & ENC_LEFT_A_ENC_LEFT_A_PIN_PIN) {
        int a = DL_GPIO_readPins(GPIOA, ENC_LEFT_A_ENC_LEFT_A_PIN_PIN) != 0;
        int b = DL_GPIO_readPins(GPIOA, ENC_LEFT_B_ENC_LEFT_B_PIN_PIN) != 0;
        left_count += a == b ? 1U : (uint32_t)-1;            /* A==B 加计数，A!=B 减计数 */
    }
    if (pending & ENC_RIGHT_A_ENC_RIGHT_A_PIN_PIN) {
        int a = DL_GPIO_readPins(GPIOA, ENC_RIGHT_A_ENC_RIGHT_A_PIN_PIN) != 0;
        int b = DL_GPIO_readPins(GPIOA, ENC_RIGHT_B_ENC_RIGHT_B_PIN_PIN) != 0;
        right_count += a == b ? 1U : (uint32_t)-1;
    }
    DL_GPIO_clearInterruptStatus(GPIOA, pending);            /* 清除已处理的编码器中断 */
}

const ObstacleState *obstacle_state(void) { return &obstacle; }

/******************************************************************
 * 函 数 名 称：CONTROL_TIMER_INST_IRQHandler
 * 函 数 说 明：累加 10 ms 控制节拍
 * 函 数 形 参：无
 * 函 数 返 回：无
 ******************************************************************/
void CONTROL_TIMER_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(CONTROL_TIMER_INST) == DL_TIMERG_IIDX_ZERO)
        ticks += 10;
}

/******************************************************************
 * 函 数 名 称：JY61P_UART_INST_IRQHandler
 * 函 数 说 明：读取 UART3 接收 FIFO 并解析惯导帧
 * 函 数 形 参：无
 * 函 数 返 回：无
 ******************************************************************/
void JY61P_UART_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(JY61P_UART_INST) == DL_UART_MAIN_IIDX_RX) {
        while (!DL_UART_Main_isRXFIFOEmpty(JY61P_UART_INST))
            jy61p_feed(&imu_rx, DL_UART_Main_receiveData(JY61P_UART_INST), ticks);
    }
}

/******************************************************************
 * 函 数 名 称：GPS_UART_INST_IRQHandler
 * 函 数 说 明：读取 UART1 FIFO 并写入 GPS 环形缓冲区
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：中断不做 NMEA 解析，避免阻塞惯导和电机控制
 ******************************************************************/
void GPS_UART_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(GPS_UART_INST) == DL_UART_MAIN_IIDX_RX) {
        while (!DL_UART_Main_isRXFIFOEmpty(GPS_UART_INST))
            gps_isr_byte(&gps, DL_UART_Main_receiveData(GPS_UART_INST));
    }
}

/******************************************************************
 * 函 数 名 称：esp_fill_tx
 * 函 数 说 明：把发送环形缓冲区数据送入 FIFO，队列空时关闭 TX 中断
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：只在 UART 中断或短暂关中断期间调用，无阻塞等待
 ******************************************************************/
static void esp_fill_tx(void)
{
    uint8_t byte;
    while (!DL_UART_Main_isTXFIFOFull(ESP_UART_INST) && esp_tx_byte(&esp, &byte))
        DL_UART_Main_transmitData(ESP_UART_INST, byte);    /* FIFO 有空位才写入 */
    if (esp.tx_tail == esp.tx_head)
        DL_UART_Main_disableInterrupt(ESP_UART_INST, DL_UART_MAIN_INTERRUPT_TX);
    else DL_UART_Main_enableInterrupt(ESP_UART_INST, DL_UART_MAIN_INTERRUPT_TX);
}

/******************************************************************
 * 函 数 名 称：esp_command
 * 函 数 说 明：主循环非阻塞读取新的远程命令
 * 函 数 形 参：command - 输出命令
 * 函 数 返 回：1 有命令，0 无命令
 ******************************************************************/
int esp_command(ESPCommand *command) { return esp_poll(&esp, command); }

/******************************************************************
 * 函 数 名 称：esp_report
 * 函 数 说 明：每二百毫秒发送一次位置，发送环形缓冲区满时跳过
 * 函 数 形 参：heading - 惯导航向；speed - 编码器测得的米每秒
 * 函 数 返 回：无
 ******************************************************************/
void esp_report(float heading, float speed)
{
    static uint32_t previous;
    uint32_t now = ticks, mask;
    if (now - previous < 200U) return;
    previous = now;
    if (!esp_position(&esp, gps.latitude, gps.longitude, heading, speed)) return;
    mask = __get_PRIMASK(); __disable_irq();
    esp_fill_tx();                                       /* 首次填 FIFO 并启动后续 TX 中断 */
    __set_PRIMASK(mask);
}

/******************************************************************
 * 函 数 名 称：ESP_UART_INST_IRQHandler
 * 函 数 说 明：UART2 中断搬运收发字节，不解析 JSON
 * 函 数 形 参：无
 * 函 数 返 回：无
 ******************************************************************/
void ESP_UART_INST_IRQHandler(void)
{
    uint32_t pending;
    while ((pending = DL_UART_Main_getPendingInterrupt(ESP_UART_INST)) != DL_UART_MAIN_IIDX_NO_INTERRUPT) {
        if (pending == DL_UART_MAIN_IIDX_RX) {
            while (!DL_UART_Main_isRXFIFOEmpty(ESP_UART_INST))
                esp_rx_byte(&esp, DL_UART_Main_receiveData(ESP_UART_INST));
        } else if (pending == DL_UART_MAIN_IIDX_TX) esp_fill_tx();
    }
}
