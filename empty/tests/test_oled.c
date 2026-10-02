/* 主机自检：模拟 I2C 延迟启动，检查实际 OLED 驱动的发送顺序。 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../board_glue.h"
#define ti_msp_dl_config_h
#define OLED_I2C_INST 0
#define CPUCLK_FREQ 32000000U
#define DL_I2C_CONTROLLER_STATUS_IDLE 1U
#define DL_I2C_CONTROLLER_STATUS_ERROR 2U
#define DL_I2C_CONTROLLER_STATUS_BUSY_BUS 4U
#define DL_I2C_CONTROLLER_DIRECTION_TX 0
static uint32_t now_ms;
static unsigned automatic, active, transfers, packet_size, capacity = 8, never_busy;
static uint8_t packet[8], records[2200][8], lengths[2200];
#define board_millis() now_ms
#define delay_cycles(cycles) ((void)(cycles))
#define DL_I2C_getControllerStatus(inst) mock_get_status()

/******************************************************************
 * 函 数 名 称：mock_get_status
 * 函 数 说 明：模拟启动延迟、总线忙及完成后的空闲状态
 * 函 数 形 参：无
 * 函 数 返 回：模拟的控制器状态
 ******************************************************************/
static uint32_t mock_get_status(void)
{
    if (never_busy || !active) return DL_I2C_CONTROLLER_STATUS_IDLE;
    if (active == 1) { active = 2; return DL_I2C_CONTROLLER_STATUS_IDLE; }
    if (active == 2) { active = 3; return DL_I2C_CONTROLLER_STATUS_BUSY_BUS; }
    if (automatic) { active = 0; return DL_I2C_CONTROLLER_STATUS_IDLE; }
    return DL_I2C_CONTROLLER_STATUS_BUSY_BUS;
}

/******************************************************************
 * 函 数 名 称：mock_fill
 * 函 数 说 明：模拟 FIFO 容量并保存待发数据
 * 函 数 形 参：data - 数据；count - 字节数
 * 函 数 返 回：实际填入字节数
 ******************************************************************/
static unsigned mock_fill(const uint8_t *data, unsigned count)
{
    packet_size = count < capacity ? count : capacity;
    memcpy(packet, data, packet_size);
    return packet_size;
}

/******************************************************************
 * 函 数 名 称：mock_start
 * 函 数 说 明：记录传输，保留启动前的空闲状态以复现硬件延迟
 * 函 数 形 参：count - 本次传输长度
 * 函 数 返 回：无
 ******************************************************************/
static void mock_start(unsigned count)
{
    assert(!active);                                      /* 前一包未完成时禁止启动下一包 */
    assert(count == packet_size && transfers < 2200);
    memcpy(records[transfers], packet, count);
    lengths[transfers++] = (uint8_t)count;
    active = 1;
}
#define DL_I2C_fillControllerTXFIFO(inst, data, count) mock_fill(data, count)
#define DL_I2C_startControllerTransfer(inst, addr, direction, count) mock_start(count)
#include "../oled_status.c"

/******************************************************************
 * 函 数 名 称：screen_check
 * 函 数 说 明：绘制新页面，逐字与原始字库核对，导出实际显存供目视检查
 * 函 数 形 参：car/o - 页面状态；word - 第一行字形索引；name - 图像文件名
 * 函 数 返 回：无
 ******************************************************************/
static void screen_check(CarControl *car, ObstacleState *o, const int word[4], const char *name)
{
    refreshing = 0; now_ms += 100;
    oled_status(car, o, -179.9f);
    for (unsigned i = 0; i < 4; ++i)
        if (word[i] >= 0)
            for (unsigned y = 0; y < 16; ++y)
                for (unsigned x = 0; x < 16; ++x)
                    assert(!!(gram[y / 8][i * 16 + x] & (1U << (y % 8))) ==
                           !!(oled_cn_16x16[word[i]][y * 2 + x / 8] & (0x80U >> (x % 8))));
    FILE *file = fopen(name, "wb"); assert(file);
    fprintf(file, "P5\n512 256\n255\n");
    for (unsigned y = 0; y < 256; ++y)
        for (unsigned x = 0; x < 512; ++x)
            fputc(gram[y / 32][x / 4] & (1U << ((y / 4) % 8)) ? 255 : 0, file);
    fclose(file);
}

/******************************************************************
 * 函 数 名 称：main
 * 函 数 说 明：验证延迟启动、完整刷新、FIFO 不足及超时停刷
 * 函 数 形 参：无
 * 函 数 返 回：0 表示通过
 ******************************************************************/
int main(void)
{
    automatic = 1;
    oled_init();
    assert(available && transfers == 28 + 8 * (3 + 128));
    for (unsigned i = 27; i < transfers - 1; ++i)
        if (records[i][0] == 0x40) assert(records[i][1] == 0); /* 开屏前写完全部空白显存 */
    assert(records[transfers - 1][0] == 0 && records[transfers - 1][1] == 0xAF);
    assert(oled_idle());
    automatic = 0;
    now_ms = 100;
    CarControl car = {0}; car.distance_m = 10; car.target = 30; car.x = 10.05f;
    ObstacleState o = {.ir_left = 1, .sonar_result = SONAR_VALID, .distance_mm = 999};
    const int distance_word[4] = {CN_JU, CN_LI, CN_SHE, CN_ZHI_SET};
    const int speed_word[4] = {CN_SU, CN_DU, CN_SHE, CN_ZHI_SET};
    const int blocked_word[4] = {CN_ZHONG, CN_DIAN, CN_SHOU, CN_ZU};
    const int turn_word[4] = {CN_ZHUAN, CN_WAN, -1, -1};
    const int sensor_word[4] = {CN_CHUAN, CN_GAN, CN_JIAN, CN_CE};
    car.sensor_test = 1;
    o.ir_right_level = 1; o.echo_stage = 2; o.echo_us = 65535; o.sonar_triggers = 999;
    screen_check(&car, &o, sensor_word, "tests/oled_sensor.pgm");
    car.sensor_test = 0;
    car.setting = SETTING_DISTANCE; screen_check(&car, &o, distance_word, "tests/oled_distance.pgm");
    car.setting = SETTING_SPEED; screen_check(&car, &o, speed_word, "tests/oled_speed.pgm");
    car.setting = SETTING_NONE; car.stop_reason = STOP_ENDPOINT;
    screen_check(&car, &o, blocked_word, "tests/oled_blocked.pgm");
    car.running = 1; car.phase = ROUTE_TURN_OUT; car.detour_side = -1;
    o.sonar_result = SONAR_NO_ECHO;
    screen_check(&car, &o, turn_word, "tests/oled_turn.pgm");
    unsigned first = transfers;
    oled_poll();
    assert(transfers == first + 1);
    oled_poll();
    assert(transfers == first + 1);                        /* 已观察到总线忙，本包未完成时不能再发送 */
    automatic = 1;
    while (refreshing) oled_poll();
    assert(oled_idle());
    unsigned record = first;
    for (unsigned p = 0; p < 8; ++p) {
        assert(lengths[record] == 2 && records[record][0] == 0);
        assert(records[record][1] == 0xB0 + p);
        ++record;
        assert(records[record][0] == 0 && records[record++][1] == 0);
        assert(records[record][0] == 0 && records[record++][1] == 0x10);
        for (unsigned x = 0; x < 128; ++x) {
            assert(lengths[record] == 2 && records[record][0] == 0x40);
            assert(records[record++][1] == gram[p][x]);
        }
    }
    assert(record == transfers);                          /* 八页显存均按列顺序完整发送 */
    automatic = 0;
    oled_send(0, 0);
    now_ms += 20;
    assert(!oled_idle() && !available);
    active = pending = 0;
    available = 1;
    capacity = 1;
    first = transfers;
    oled_send(0, 0);
    assert(!available && transfers == first);             /* FIFO 填充不足不能启动残缺传输 */
    capacity = 8;
    never_busy = available = 1;
    oled_send(0, 0);
    assert(!available && !pending);                       /* 总线没有启动时有限等待后退出 */
    puts("OLED self-check: PASS");
    return 0;
}
