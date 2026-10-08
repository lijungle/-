#include "board_glue.h"
#include "ti_msp_dl_config.h"
#include "oled_ascii.h"
#include "oled_font_cn.h"
#include <stdio.h>
#include <string.h>

static uint8_t gram[8][128];
static uint8_t available, pending, refreshing, page, column, address_pending;
static uint32_t sent_ms, frame_ms;

/******************************************************************
 * 函 数 名 称：oled_send
 * 函 数 说 明：沿用参考工程的 I2C 控制字节和 0x3C 地址发起短传输
 * 函 数 形 参：mode - 命令或数据控制字节；data - 待发送字节
 * 函 数 返 回：无
 ******************************************************************/
static void oled_send(uint8_t mode, uint8_t data)
{
    uint8_t packet[2] = {mode, data};
    if (DL_I2C_fillControllerTXFIFO(OLED_I2C_INST, packet, 2) != 2) {
        available = 0;                                    /* FIFO 不足时禁止发送残缺命令或数据 */
        return;
    }
    DL_I2C_startControllerTransfer(OLED_I2C_INST, 0x3C,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);                /* 与已验证的参考工程保持相同的两字节传输 */
    for (unsigned retry = 0; retry < 100; ++retry) {
        if (DL_I2C_getControllerStatus(OLED_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
            sent_ms = board_millis();
            pending = 1;                                  /* 先观察到 BUSY，再允许空闲状态表示完成 */
            return;
        }
        delay_cycles(CPUCLK_FREQ / 100000);                /* 启动检查最多等待约 1 ms，避免显示故障卡住控制 */
    }
    available = 0;
}

/******************************************************************
 * 函 数 名 称：oled_idle
 * 函 数 说 明：检查传输是否完成，故障或超时后停止刷新
 * 函 数 形 参：无
 * 函 数 返 回：1 空闲，0 忙或不可用
 ******************************************************************/
static int oled_idle(void)
{
    uint32_t status = DL_I2C_getControllerStatus(OLED_I2C_INST);
    if ((pending && (status & DL_I2C_CONTROLLER_STATUS_ERROR)) ||
        (pending && board_millis() - sent_ms >= 20)) {
        available = 0;                                    /* 显示器断线不会阻塞电机控制 */
        return 0;
    }
    if (!(status & DL_I2C_CONTROLLER_STATUS_IDLE) ||
        (status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)) return 0;
    pending = 0;
    return available;
}

/******************************************************************
 * 函 数 名 称：oled_init
 * 函 数 说 明：按参考工程的 SSD1306 命令序列初始化显示器
 * 函 数 形 参：无
 * 函 数 返 回：无
 ******************************************************************/
void oled_init(void)
{
    static const uint8_t commands[] = {
        0xAE,0x00,0x10,0x40,0x81,0xCF,0xA1,0xC8,0xA6,0xA8,0x3F,
        0xD3,0x00,0xD5,0x80,0xD9,0xF1,0xDA,0x12,0xDB,0x40,
        0x20,0x02,0x8D,0x14,0xA4,0xA6
    };
    delay_cycles(CPUCLK_FREQ / 10);                        /* 上电复位等待 100 ms，电机尚未运行 */
    available = 1;
    for (unsigned i = 0; i < sizeof(commands); ++i) {
        uint32_t start_ms = board_millis();
        while (!oled_idle() && available) {
            if (board_millis() - start_ms >= 20) available = 0;
        }
        if (!available) return;
        oled_send(0x00, commands[i]);
    }
    memset(gram, 0, sizeof(gram));
    page = column = 0;
    address_pending = 3;
    refreshing = 1;
    while (available && (refreshing || pending)) oled_poll(); /* 电机未启动，先写完八页空白显存 */
    if (!available) return;
    oled_send(0x00, 0xAF);                                 /* 清屏完成后才开启显示，消除上电随机点阵 */
    while (available && !oled_idle()) {}
}

/******************************************************************
 * 函 数 名 称：oled_pixel
 * 函 数 说 明：在 128x64 显存中设置一个像素
 * 函 数 形 参：x/y - 像素坐标
 * 函 数 返 回：无
 ******************************************************************/
static void oled_pixel(unsigned x, unsigned y)
{
    if (x < 128 && y < 64) gram[y / 8][x] |= (uint8_t)(1U << (y % 8));
}

/******************************************************************
 * 函 数 名 称：oled_chinese
 * 函 数 说 明：绘制 car_smart 的固定 16x16 中文字形
 * 函 数 形 参：x/y - 像素坐标；idx - 字库编号
 * 函 数 返 回：无
 ******************************************************************/
static void oled_chinese(unsigned x, unsigned y, unsigned idx)
{
    for (unsigned row = 0; row < 16; ++row)
        for (unsigned col = 0; col < 16; ++col)
            if (oled_cn_16x16[idx][row * 2 + col / 8] & (0x80U >> (col % 8)))
                oled_pixel(x + col, y + row);              /* 中文字库为逐行、高位在左的点阵 */
}

/******************************************************************
 * 函 数 名 称：oled_text
 * 函 数 说 明：使用参考工程的 ASCII 字库绘制 8x16 字符
 * 函 数 形 参：x/y - 像素坐标；text - ASCII 字符串
 * 函 数 返 回：无
 ******************************************************************/
static void oled_text(unsigned x, unsigned y, const char *text)
{
    while (*text >= ' ' && *text <= '~' && x <= 120) {
        const unsigned char *font = asc2_1608[(unsigned char)*text++ - ' '];
        for (unsigned col = 0; col < 8; ++col)
            for (unsigned row = 0; row < 16; ++row)
                if (font[col * 2 + row / 8] & (0x80U >> (row % 8)))
                    oled_pixel(x + col, y + row);          /* 与参考 OLED_ShowChar 的纵向取模一致 */
        x += 8;
    }
}

/******************************************************************
 * 函 数 名 称：oled_words
 * 函 数 说 明：绘制四字以内的状态标签
 * 函 数 形 参：y - 像素行；a/b/c/d - 字形索引，负数不绘制
 * 函 数 返 回：无
 ******************************************************************/
static void oled_words(unsigned y, int a, int b, int c, int d)
{
    const int chars[4] = {a, b, c, d};
    for (unsigned i = 0; i < 4; ++i)
        if (chars[i] >= 0) oled_chinese(i * 16, y, (unsigned)chars[i]);
}

/******************************************************************
 * 函 数 名 称：oled_status
 * 函 数 说 明：显示设置页、停车原因、前进距离与障碍快照
 * 函 数 形 参：car - 控制状态；o - 障碍状态；yaw - 航向
 * 函 数 返 回：无
 ******************************************************************/
void oled_status(const CarControl *car, const ObstacleState *o, float yaw)
{
    char text[32];                                       /* 容纳格式化函数对完整整数范围的输出要求 */
    const GPSState *gps = gps_state();
    if (!available || refreshing || board_millis() - frame_ms < 100) return;
    frame_ms = board_millis();
    memset(gram, 0, sizeof(gram));
    if (0) {
        oled_words(0, CN_CHUAN, CN_GAN, CN_JIAN, CN_CE);
        snprintf(text, sizeof(text), "T%03u", o->sonar_triggers % 1000);
        oled_text(88, 0, text);
        snprintf(text, sizeof(text), "PB10:%u PB11:%u", o->ir_left_level, o->ir_right_level);
        oled_text(0, 16, text);                             /* 原始电平不经过有效极性转换或二十毫秒消抖 */
        if (o->sonar_result == SONAR_VALID) {
            snprintf(text, sizeof(text), "%ucm", o->distance_mm / 10);
            oled_text(0, 32, text);
        } else if (o->sonar_result == SONAR_NO_ECHO) oled_words(32, CN_WU, CN_HUI, CN_BO, -1);
        else if (o->sonar_result == SONAR_FAULT) oled_words(32, CN_HUI, CN_BO, CN_GU, CN_ZHANG);
        else oled_text(0, 32, "--");
        snprintf(text, sizeof(text), "L%u R%u", o->ir_left, o->ir_right);
        oled_text(72, 32, text);
        snprintf(text, sizeof(text), "E%u P%u %05uus", o->echo_stage, o->echo_level, o->echo_us);
        oled_text(0, 48, text);                             /* 零未收到上升沿，一只有上升沿，二完整脉冲 */
    } else if (car->setting == SETTING_GPS) {
        oled_text(0, 0, "GPS");
        if (gps->overflow) oled_text(64, 0, "OVF");
        if (gps->last_sentence_ms == 0 || board_millis() - gps->last_sentence_ms > 3000U) {
            oled_text(0, 16, "NO DATA");
        } else {
            snprintf(text, sizeof(text), "%c%09.5f", gps->latitude < 0 ? 'S' : 'N',
                     gps->latitude < 0 ? -gps->latitude : gps->latitude);
            oled_text(0, 16, text);                       /* 纬度显示十进制度和南北半球 */
            snprintf(text, sizeof(text), "%c%010.5f", gps->longitude < 0 ? 'W' : 'E',
                     gps->longitude < 0 ? -gps->longitude : gps->longitude);
            oled_text(0, 32, text);                       /* 经度显示十进制度和东西半球 */
            snprintf(text, sizeof(text), "Q:%u HDOP:%4.2f", (unsigned)gps->quality, gps->hdop);
            oled_text(0, 48, text);                       /* 定位质量 0 表示当前无有效定位 */
        }
    } else if (car->setting) {
        oled_words(0, car->setting == SETTING_DISTANCE ? CN_JU :
                   car->setting == SETTING_SPEED ? CN_SU : CN_JU,
                   car->setting == SETTING_DISTANCE ? CN_LI :
                   car->setting == SETTING_SPEED ? CN_DU : CN_LI, CN_SHE, CN_ZHI_SET);
        snprintf(text, sizeof(text), car->setting == SETTING_DISTANCE ? "%um" :
                 car->setting == SETTING_SPEED ? "%u/10ms" : "%u",
                 car->setting == SETTING_DISTANCE ? (unsigned)car->distance_m :
                 car->setting == SETTING_SPEED ? (unsigned)car->target : (unsigned)car->counts_per_meter);
        oled_text(64, 16, text);                            /* 独立页面的数值最多占八个字符 */
        oled_words(32, CN_SU, CN_DU, -1, -1);
        snprintf(text, sizeof(text), "M:%u V:%u C:%u", (unsigned)car->distance_m,
                 (unsigned)car->target, (unsigned)car->counts_per_meter);
        oled_text(64, 32, text);
    } else if (car->running) {
        if (car->phase == ROUTE_REMOTE) oled_text(0, 0, "REMOTE");
        else if (car->mode == MODE_SQUARE) oled_words(0, CN_RAO, CN_XING, -1, -1);
        else if (car->phase == ROUTE_STRAIGHT) oled_words(0, CN_ZHI_STRAIGHT, CN_XING, -1, -1);
        else if (car->phase == ROUTE_BRAKE) oled_words(0, CN_TING, CN_ZHI, -1, -1);
        else if (car->phase == ROUTE_CHECK) oled_words(0, CN_JIAN, CN_CE, -1, -1);
        else if (car->phase == ROUTE_SHIFT_IN) oled_words(0, CN_FAN, CN_HUI, -1, -1);
        else if (car->phase == ROUTE_SHIFT_OUT || car->phase == ROUTE_PASS) oled_words(0, CN_RAO, CN_XING, -1, -1);
        else oled_words(0, CN_ZHUAN, CN_WAN, -1, -1);
        if (car->mode == MODE_SQUARE)
            snprintf(text, sizeof(text), "B%u/4", (unsigned)(car->square_edge + 1));
        else if (car->phase != ROUTE_STRAIGHT && car->phase != ROUTE_BRAKE)
            oled_chinese(48, 0, car->detour_side > 0 ? CN_ZUO : CN_YOU);
    } else {
        switch (car->stop_reason) {
        case STOP_DONE: oled_words(0, CN_WAN_DONE, CN_CHENG, -1, -1); break;
        case STOP_IMU: oled_words(0, CN_GUAN, CN_DAO, CN_GU, CN_ZHANG); break;
        case STOP_SENSOR: oled_words(0, CN_CHUAN, CN_GAN, CN_GU, CN_ZHANG); break;
        case STOP_ENDPOINT: oled_words(0, CN_ZHONG, CN_DIAN, CN_SHOU, CN_ZU); break;
        case STOP_BLOCKED: oled_words(0, CN_ZHANG, CN_AI, CN_TING, CN_ZHI); break;
        case STOP_STALL: oled_words(0, CN_DU_STALL, CN_ZHUAN, -1, -1); break;
        case STOP_TURN_TIMEOUT: oled_words(0, CN_ZHUAN, CN_WAN, CN_CHAO, CN_SHI); break;
        case STOP_PATH: oled_words(0, CN_PIAN, CN_YI, -1, -1); break;
        default: oled_words(0, CN_TING, CN_ZHI, -1, -1); break;
        }
    }
    if (!car->setting) {
        int cm = (int)(car->x * 100);
        if (cm < 0) cm = 0;                                /* 短暂反向微调不显示负前进距离 */
        oled_words(16, CN_JU, CN_LI, -1, -1);
        snprintf(text, sizeof(text), "%d.%02d/%um", cm / 100, cm % 100, (unsigned)car->distance_m);
        oled_text(40, 16, text);                            /* 横向绕行不算入纵向终点距离 */
        oled_text(0, 32, "                ");              /* 回退版不显示传感器状态 */
        if (!car->running) oled_words(32, car->mode == MODE_SQUARE ? CN_RAO : CN_ZHI_STRAIGHT,
                                      CN_XING, -1, -1);     /* 停止页显示当前路线模式 */
    }
    if (car->setting != SETTING_GPS) {
        oled_chinese(0, 48, CN_HANG); oled_chinese(16, 48, CN_XIANG);
        int tenths = (int)(yaw * 10);
        snprintf(text, sizeof(text), "%c%3d.%d", tenths < 0 ? '-' : '+',
            (tenths < 0 ? -tenths : tenths) / 10, (tenths < 0 ? -tenths : tenths) % 10);
        oled_text(48, 48, text);                            /* 用整数格式避免嵌入式浮点 printf 依赖 */
    }
    page = column = 0;
    address_pending = 3;
    refreshing = 1;
}

/******************************************************************
 * 函 数 名 称：oled_poll
 * 函 数 说 明：每次发送一个命令或显存字节，主循环持续调用推进刷新
 * 函 数 形 参：无
 * 函 数 返 回：无
 ******************************************************************/
void oled_poll(void)
{
    if (!available || !oled_idle() || !refreshing) return;
    if (address_pending) {
        uint8_t command = address_pending == 3 ? (uint8_t)(0xB0 + page) :
                          address_pending == 2 ? 0x00 : 0x10;
        oled_send(0x00, command);                          /* 页地址、低列地址、高列地址分别发送 */
        --address_pending;
        return;
    }
    oled_send(0x40, gram[page][column]);                    /* 数据传输完成由下一次轮询检查，不阻塞整帧控制 */
    ++column;
    if (column == 128) {
        column = 0;
        address_pending = 3;
        if (++page == 8) refreshing = 0;
    }
}
