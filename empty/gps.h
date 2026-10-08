#ifndef GPS_H
#define GPS_H

#include <stdint.h>

#define GPS_RX_BUFFER_SIZE 512U

typedef struct {
    uint8_t rx[GPS_RX_BUFFER_SIZE];
    volatile uint16_t head, tail;
    volatile uint8_t overflow;
    char line[128];
    uint8_t line_len, in_sentence;
    uint8_t valid, quality;
    float latitude, longitude, hdop;
    uint32_t last_sentence_ms, last_fix_ms;
} GPSState;

void gps_init(GPSState *gps);
void gps_isr_byte(GPSState *gps, uint8_t byte);
void gps_poll(GPSState *gps, uint32_t now_ms);

#endif
