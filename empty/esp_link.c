#include "esp_link.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

/******************************************************************
 * 函 数 名 称：esp_init
 * 函 数 说 明：清空 ESP32 串口收发环形缓冲区
 * 函 数 形 参：link - 通信状态
 * 函 数 返 回：无
 ******************************************************************/
void esp_init(ESPLink *link) { memset(link, 0, sizeof(*link)); }

/******************************************************************
 * 函 数 名 称：esp_rx_byte
 * 函 数 说 明：在中断中接收字节，满队列时仅记录错误
 * 函 数 形 参：link - 通信状态；byte - 收到的字节
 * 函 数 返 回：无
 ******************************************************************/
void esp_rx_byte(ESPLink *link, uint8_t byte)
{
    uint16_t next = (link->rx_head + 1U) % ESP_BUFFER_SIZE;
    if (next == link->rx_tail) { link->rx_overflow = 1; return; }
    link->rx[link->rx_head] = byte;
    link->rx_head = next;                                 /* 先写数据，再发布头指针 */
}

/******************************************************************
 * 函 数 名 称：skip_space
 * 函 数 说 明：跳过 JSON 字段之间的空白
 * 函 数 形 参：p - 当前解析位置
 * 函 数 返 回：首个非空白字符位置
 ******************************************************************/
static const char *skip_space(const char *p)
{
    while (isspace((unsigned char)*p)) ++p;
    return p;
}

/******************************************************************
 * 函 数 名 称：read_string
 * 函 数 说 明：读取协议中的短字符串，拒绝转义和控制字符
 * 函 数 形 参：cursor - 解析位置；out - 输出缓冲区；size - 容量
 * 函 数 返 回：1 成功，0 格式错误或超长
 ******************************************************************/
static int read_string(const char **cursor, char *out, unsigned size)
{
    const char *p = skip_space(*cursor);
    unsigned n = 0;
    if (*p++ != '"') return 0;
    while (*p && *p != '"') {
        if ((unsigned char)*p < 32 || *p == '\\' || n + 1 >= size) return 0;
        out[n++] = *p++;
    }
    if (*p != '"') return 0;
    out[n] = 0; *cursor = p + 1;
    return 1;
}

/******************************************************************
 * 函 数 名 称：parse_control
 * 函 数 说 明：校验平面 JSON 控制帧，字段顺序不限，拒绝重复和越界值
 * 函 数 形 参：line - 完整帧；command - 解析结果
 * 函 数 返 回：1 有效，0 丢弃
 * 备       注：仅接受约定四个字段；运动持续时间范围为 1~60000 毫秒
 ******************************************************************/
static int parse_control(const char *line, ESPCommand *command)
{
    const char *p = skip_space(line);
    unsigned seen = 0;
    ESPCommand result = {0};
    if (*p++ != '{') return 0;
    for (;;) {
        char key[20], value[20];
        unsigned flag;
        if (!read_string(&p, key, sizeof(key))) return 0;
        p = skip_space(p);
        if (*p++ != ':') return 0;
        p = skip_space(p);
        if (!strcmp(key, "type") || !strcmp(key, "cmd")) {
            flag = !strcmp(key, "type") ? 1U : 2U;
            if (!read_string(&p, value, sizeof(value))) return 0;
            if (flag == 1) { if (strcmp(value, "control")) return 0; }
            else {
                static const char *const names[] = {"stop", "forward", "backward", "left", "right"};
                unsigned i;
                for (i = 0; i < 5 && strcmp(value, names[i]); ++i) {}
                if (i == 5) return 0;
                result.cmd = (uint8_t)i;
            }
        } else if (!strcmp(key, "speed") || !strcmp(key, "duration_ms")) {
            char *end;
            unsigned long number;
            flag = !strcmp(key, "speed") ? 4U : 8U;
            if (!isdigit((unsigned char)*p) || (*p == '0' && isdigit((unsigned char)p[1]))) return 0;
            number = strtoul(p, &end, 10);
            if (number > (flag == 4 ? 100UL : 60000UL)) return 0;
            if (flag == 4) result.speed = (uint8_t)number;
            else result.duration_ms = (uint32_t)number;
            p = end;
        } else return 0;
        if (seen & flag) return 0;
        seen |= flag;                                    /* 重复字段不能覆盖已校验的值 */
        p = skip_space(p);
        if (*p == '}') { ++p; break; }
        if (*p++ != ',') return 0;
    }
    if (*skip_space(p) || (seen & 3U) != 3U) return 0;
    if (result.cmd != ESP_STOP && (seen != 15U || !result.duration_ms)) return 0;
    *command = result;
    return 1;
}

/******************************************************************
 * 函 数 名 称：esp_poll
 * 函 数 说 明：主循环限量组装换行帧，溢出或超长后丢弃至换行
 * 函 数 形 参：link - 通信状态；command - 最新命令
 * 函 数 返 回：1 有新命令，0 无命令
 * 备       注：单次最多处理 128 字节，同批停止命令优先
 ******************************************************************/
int esp_poll(ESPLink *link, ESPCommand *command)
{
    unsigned budget = 128;
    int ready = 0;
    if (link->rx_overflow) {
        link->rx_tail = link->rx_head;
        link->length = 0; link->dropping = 1;
        link->rx_overflow = 0;                            /* 丢弃不完整帧，不能拼成错误的运动指令 */
    }
    while (link->rx_tail != link->rx_head && budget--) {
        uint8_t byte = link->rx[link->rx_tail];
        link->rx_tail = (link->rx_tail + 1U) % ESP_BUFFER_SIZE;
        if (byte == '\n') {
            ESPCommand parsed;
            link->line[link->length] = 0;
            if (!link->dropping && parse_control(link->line, &parsed)) {
                if (!ready || command->cmd != ESP_STOP) *command = parsed;
                ready = 1;
            }
            link->length = 0; link->dropping = 0;
        } else if (!link->dropping) {
            if (!byte || link->length + 1U >= sizeof(link->line)) link->dropping = 1;
            else link->line[link->length++] = (char)byte;
        }
    }
    return ready;
}

/******************************************************************
 * 函 数 名 称：esp_tx_byte
 * 函 数 说 明：中断从发送环形缓冲区取出一个字节
 * 函 数 形 参：link - 通信状态；byte - 字节输出
 * 函 数 返 回：1 有数据，0 队列空
 ******************************************************************/
int esp_tx_byte(ESPLink *link, uint8_t *byte)
{
    if (link->tx_tail == link->tx_head) return 0;
    *byte = link->tx[link->tx_tail];
    link->tx_tail = (link->tx_tail + 1U) % ESP_BUFFER_SIZE;
    return 1;
}

/******************************************************************
 * 函 数 名 称：esp_position
 * 函 数 说 明：把定位 JSON 换行帧整体入队，容量不足则丢弃整帧
 * 函 数 形 参：link - 通信状态；lat/lng - 经纬度；heading - 航向度；speed - 米每秒
 * 函 数 返 回：1 入队，0 丢弃
 ******************************************************************/
int esp_position(ESPLink *link, float lat, float lng, float heading, float speed)
{
    char frame[160];
    int n;
    uint16_t head = link->tx_head;
    if (!isfinite(lat) || !isfinite(lng) || !isfinite(heading) || !isfinite(speed)) return 0;
    n = snprintf(frame, sizeof(frame), "{\"type\":\"position\",\"lat\":%.6f,\"lng\":%.6f,\"heading\":%.2f,\"speed\":%.3f}\n", lat, lng, heading, speed);
    if (n <= 0 || n >= (int)sizeof(frame)) return 0;
    if ((unsigned)n > (link->tx_tail + ESP_BUFFER_SIZE - head - 1U) % ESP_BUFFER_SIZE) {
        link->tx_overflow = 1; return 0;                   /* 禁止发送半条 JSON，主循环不等待 */
    }
    for (int i = 0; i < n; ++i) { link->tx[head] = (uint8_t)frame[i]; head = (head + 1U) % ESP_BUFFER_SIZE; }
    link->tx_head = head;                                 /* 整帧复制完成后一次发布 */
    return 1;
}
