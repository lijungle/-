#include "gps.h"
#include <stdlib.h>
#include <string.h>

static int field_span(const char *line, unsigned wanted, const char **begin, unsigned *length)
{
    const char *p = line;
    unsigned field = 0;
    while (*p && *p != '*') {
        const char *start = p;
        while (*p && *p != ',' && *p != '*') ++p;
        if (field == wanted) {
            *begin = start;
            *length = (unsigned)(p - start);
            return 1;
        }
        if (*p == ',') ++p;
        ++field;
    }
    return 0;
}

static float field_float(const char *line, unsigned field)
{
    const char *begin;
    unsigned length;
    char text[20];
    if (!field_span(line, field, &begin, &length) || length == 0 || length >= sizeof(text)) return 0.0f;
    memcpy(text, begin, length);
    text[length] = '\0';
    return (float)strtod(text, 0);
}

static char field_char(const char *line, unsigned field)
{
    const char *begin;
    unsigned length;
    return field_span(line, field, &begin, &length) && length ? *begin : '\0';
}

static float nmea_coordinate(float value, char hemisphere)
{
    float degrees = (float)((int)(value / 100.0f));
    float coordinate = degrees + (value - degrees * 100.0f) / 60.0f;
    return (hemisphere == 'S' || hemisphere == 'W') ? -coordinate : coordinate;
}

static int valid_checksum(const char *line)
{
    const char *star = strchr(line, '*');
    unsigned checksum = 0;
    unsigned expected;
    char *end;
    if (!star || star < line + 2 || star[1] == '\0' || star[2] == '\0') return 0;
    for (const char *p = line + 1; p < star; ++p) checksum ^= (unsigned char)*p;
    expected = (unsigned)strtoul(star + 1, &end, 16);
    return (end == star + 3 || *end == '\r' || *end == '\n') && checksum == expected;
}

static void parse_sentence(GPSState *gps, uint32_t now_ms)
{
    const char *type;
    if (!valid_checksum(gps->line)) return;
    type = gps->line + 3;
    gps->last_sentence_ms = now_ms;
    if ((!strncmp(type, "GGA", 3))) {
        uint8_t quality = (uint8_t)field_float(gps->line, 6);
        gps->quality = quality;
        gps->hdop = field_float(gps->line, 8);
        if (quality) {
            gps->latitude = nmea_coordinate(field_float(gps->line, 2), field_char(gps->line, 3));
            gps->longitude = nmea_coordinate(field_float(gps->line, 4), field_char(gps->line, 5));
            gps->valid = 1;
            gps->last_fix_ms = now_ms;
        } else {
            gps->valid = 0;
        }
    } else if (!strncmp(type, "RMC", 3) && field_char(gps->line, 2) == 'A') {
        gps->latitude = nmea_coordinate(field_float(gps->line, 3), field_char(gps->line, 4));
        gps->longitude = nmea_coordinate(field_float(gps->line, 5), field_char(gps->line, 6));
        gps->valid = 1;
        gps->last_fix_ms = now_ms;
    }
}

/******************************************************************
 * 函 数 名 称：gps_init
 * 函 数 说 明：初始化 GPS 接收环形缓冲区和定位状态
 * 函 数 形 参：gps - GPS 状态对象
 * 函 数 返 回：无
 ******************************************************************/
void gps_init(GPSState *gps)
{
    memset(gps, 0, sizeof(*gps));
}

/******************************************************************
 * 函 数 名 称：gps_isr_byte
 * 函 数 说 明：把 UART 收到的一个字节写入环形缓冲区
 * 函 数 形 参：gps - GPS 状态对象；byte - 接收字节
 * 函 数 返 回：无
 * 备       注：中断中只做入队，不解析、不等待，避免阻塞控制任务
 ******************************************************************/
void gps_isr_byte(GPSState *gps, uint8_t byte)
{
    uint16_t next = (uint16_t)((gps->head + 1U) % GPS_RX_BUFFER_SIZE);
    if (next == gps->tail) {
        gps->overflow = 1;                              /* 满队列丢弃当前字节并记录溢出 */
        return;
    }
    gps->rx[gps->head] = byte;
    gps->head = next;                                    /* 写入完成后再推进头指针 */
}

/******************************************************************
 * 函 数 名 称：gps_poll
 * 函 数 说 明：限量取出 GPS 字节并解析 NMEA 定位帧
 * 函 数 形 参：gps - GPS 状态对象；now_ms - 当前毫秒计数
 * 函 数 返 回：无
 * 备       注：单次最多解析 128 字节，保证控制主循环不会被串口数据拖住
 ******************************************************************/
void gps_poll(GPSState *gps, uint32_t now_ms)
{
    unsigned budget = 128;
    while (gps->tail != gps->head && budget--) {
        uint8_t byte = gps->rx[gps->tail];
        gps->tail = (uint16_t)((gps->tail + 1U) % GPS_RX_BUFFER_SIZE);
        if (byte == '$') {
            gps->line_len = 0;
            gps->in_sentence = 1;
        }
        if (!gps->in_sentence) continue;
        if (gps->line_len + 1U >= sizeof(gps->line)) {
            gps->in_sentence = 0;                         /* 超长帧丢弃，等待下一个 '$' */
            continue;
        }
        gps->line[gps->line_len++] = (char)byte;
        if (byte == '\n') {
            gps->line[gps->line_len] = '\0';
            parse_sentence(gps, now_ms);
            gps->in_sentence = 0;
        }
    }
    if (gps->last_fix_ms && now_ms - gps->last_fix_ms > 3000U)
        gps->valid = 0;                                    /* 超过三秒没有有效定位则标记失效 */
}
